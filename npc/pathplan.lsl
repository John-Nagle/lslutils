//
//  pathplan.lsl -- components of a path building system
//
//  Part of a system for doing pathfinding in Second Life.
//
//  This is the planning task.
//
//  Animats
//  June, 2019
//
#include "npc/pathbuildutils.lsl"                           // common functions
#include "npc/pathmazesolvercall.lsl"
//
//  Constants
//
float MINSEGMENTLENGTH = 0.025; /// 0.10;                   // minimum path segment length (m)

#define PATHPLANMINMEM  3500                                // less than this, quit early and retry

//  Globals

//
//  The state of pathfinding
//
list gPts;                                                  // the point set from static planning
list gPathPoints;                                           // point set being constructed

vector gP0;                                                 // current working point
vector gP1;                                                 // current working point
float gDistalongseg;                                        // distance along curren segment  
integer gCurrentix;                                         // current index
//  Input params
float gTestspacing;
integer gReqPathid;                                         // path ID of planning section
vector gRefPt;                                              // region corner to which points are relative

integer gPathStarttime;                                     // time path planning started, before retries
//
//  pathplan -- plan an obstacle-free path.
//
//  Output is via calls to pathdeliversegment.
//  Errors are reported via pathdeliversegment.
//
//  This constructs a static path, then checks it for open space.
//  When it finds an obstacle, it reports clear points on both sides
//  of the obstacle, which are then fed to the maze solver.
//
//  This is slow; it can take many seconds. But as soon as some results
//  have been delivered via pathdeliversegment, character movement can start.
//
//  This is best-effort; moves will be reported even if the destination cannot be
//  fully reached.
//
//  Can't handle paths that cross region boundaries because it can't
//  find out anything about the next region.
//
pathplan(float stopshort, integer pathid)
{   assert(gPathWidth > 0);                                 // indicates script initialization is complete
    //  Start a new planning cycle
    gPathPoints = [];
    gReqPathid = pathid; 
    ////pathMsg(PATH_MSG_INFO,"Preprocessed points 2: " + llDumpList2String(gPts,","));   // ***TEMP***
    assert(llGetListLength(gPts) >= 2);                     // must have at least 2 points to do planning
    gP0 = llList2Vector(gPts,0);                            // starting position
    gPathPoints = [gP0];                                    // output points
    ////{ if (llVecMag(<tfirstp.x, tfirstp.y,0> - <p0.x,p0.y,0>) > 0.001) {pathMsg(PATH_MSG_WARN, "Pathplan prelim adjustment broke 1st pt: " + (string)tfirstp + " -> " + (string)p0);}} // ***TEMP***

    gP1 = llList2Vector(gPts,1);                            // next position
    gDistalongseg = 0.0;                                    // starting position, one extra width
    gCurrentix = 0;                                         // working on segment 0
    pathplanadvance();                                      // start the planner
}

//
//  pathmazesolverdone -- the maze solver is done.
//
//  So we have to start it again, if there's more work.
//
pathmazesolverdone(integer pathid, integer segmentid, integer status)
{   pathMsg(PATH_MSG_NOTE, "Maze solver done with pathid: " + (string)pathid + " segment: " + (string)segmentid + " status: " + (string)status);
    if (gPts == []) { return; }                             // no more planning needed
    if (pathid != gReqPathid) { return; }                   // from some other pathid, do not use
    if (status != 0)                                        // if an error on what we're working on, stop.
    {   pathMsg(PATH_MSG_WARN,"Maze solver error, stopping planning. Status: " + (string)status);
        gPts = [];                                          // prevent further useless but harmless maze solves.
        return;
    }
    pathplanadvance();                                      // restart the planner
}

//
//  pathplanadvance --  advance path planning one cycle
//
//  A cycle must include one maze solve, or must finish the path.
//  That's because maze solve completions cause the next path advance.
//
//  Returns TRUE if done.
//
//  The occupancy rules of path planning:
//    
//    - The starting point is assumed clear, even if it isn't.
//      However we got there.
//      
//    - Path segments must pass the castbeam test to 
//      one half width beyond the segment end point.
//      This assures room for the character at the
//      segment end point.
//      
//    - If the planner finds an obstacle, it backs up
//      one width, but not further than the beginning
//      of the segment or distance used along the segment.
//      We know there was room there, because that was
//      checked previously on the previous segment
//      check.
//      
//    - The beginning and ending cells of a maze solve
//      are assumed clear, because those are checked
//      before calling the maze solver.
//      
//
integer pathplanadvance()
{   assert(gPts != []);                                         // must not call with no points                                                    
    integer doingmaze = FALSE;                                  // true if we started a maze solve
    while (!doingmaze)                                          // until a maze solve is started
    {
        float fulllength = llVecMag(gP1-gP0);                   // full segment length
        vector dir = llVecNorm(gP1-gP0);                        // direction of segment
        vector pos = gP0 + dir*gDistalongseg;                   // current working position
        ////vector endcast = gP1 + dir * gPathWidth*0.5;                // check to far side of area to avoid missing partial obstacle at end
        vector endcast = gP1 + dir * gPathWidth;                // check to far side of area plus extra halfwidth to avoid missing partial obstacle at end

        float hitdist = castbeam(pos, endcast, gTestspacing, TRUE,
                PATHCASTRAYOPTS);
        pathMsg(PATH_MSG_INFO,"Checked " + (string)pos + " to " + (string)endcast + " for obstacles. Hit dist: "+ (string)hitdist);
        if (hitdist < 0)
        {   gPts = [];                                          // release memory
            pathdeliversegment([], FALSE, TRUE, gReqPathid, PATHERRMAZECASTFAIL);    // empty set of points, no maze, done.
            return(TRUE);                                       // failure
        }
        //  Compute distance from pos to spot before next obstacle.
        float hitbackedup = llVecMag(endcast-pos);              // dist to end if no horiz obstacle                  
        if (hitdist != INFINITY)                                // if there was a horizontal obstacle
        {                                                       // compute point just before it.
            hitbackedup = hitdist-gPathWidth;                   // back up just enough to get clear
        }
        //  Enforce sanity of hitbackedup and gDistalongseg.
        assert(gDistalongseg >= 0);                             // ***TEMP***
        if (hitbackedup < 0.0) { hitbackedup = 0.0; }           // but don't back up through previous point, previously verified clear
        if (hitbackedup + gDistalongseg > fulllength) { hitbackedup = fulllength - gDistalongseg; } // can potentially be off the end, so avoid that.
        //  Scan from pos to hit point, vertical ray casts, for walkables.
        //  This is to catch ground-level non-walkable areas not seen by horizontal casts
        float endz = pos.z;                                     // Z height at start of segment
        {
            vector fullheight = <0,0,gPathHeight>;              // add this for casts from middle of character
            vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;     // subtract this for bottom end of ray cast
            float alongdist;                                    // distance along seg
            for (alongdist = 0; alongdist < hitbackedup; alongdist += gPathWidth) // advance along segment testing vertically
            {
                vector p = pos + alongdist*dir;                 // position on ground
                ////if (obstacleraycastvert(p+fullheight,p-mazedepthmargin) < 0) // if found a non-walkable location 
                float z = obstacleraycastvert(p+fullheight,p-mazedepthmargin);// Z height here, or <0 if not walkable
                if (z < 0)                                      // if found a non-walkable location
                {                                               // problem
                    ////hitdist = alongdist - gPathWidth;           // cut back hitdist to here
                    hitbackedup = alongdist - gPathWidth*1.5;   // cut back hitdist to clear bad spot
                    hitdist = alongdist;                        // force non-infinite case
                    if (hitbackedup < 0.0) { hitbackedup = 0.0; } // but never negative 
                    alongdist = INFINITY;                       // force loop exit now
                    pathMsg(PATH_MSG_WARN,"No walkable during planning at " + (string)p); // ***TEMP***
                } else {                                        // this is the Z height to use
                    endz = z;
                }
            }           
        }
        if (hitdist == INFINITY)                                // completely clear segment
        {   gP1.z = endz;                                       // apply Z adjustment to keep feet on ground
            gPathPoints += [gP1];                               // completely empty segment
            gCurrentix += 1;                                    // advance to next segment
            if (gCurrentix >= llGetListLength(gPts)-1)          // done
            {   pathdeliversegment(gPathPoints, FALSE, TRUE, gReqPathid, 0);// points, not a maze, final.
                return(TRUE);                                   // done
            }
            gP0 = llList2Vector(gPts,gCurrentix);               // starting position in new segment
            gP1 = llList2Vector(gPts,gCurrentix+1);             // next position
            gDistalongseg = 0.0;                                // starting new segment
        } else {                                                // there is an obstruction
            vector interpt0 = pos + dir*(hitbackedup);          // back away from obstacle.
            interpt0.z = endz;                                  // apply Z adjustment to keep feet on ground
            pathMsg(PATH_MSG_INFO,"Hit obstacle at segment #" + (string)gCurrentix + " " + (string) interpt0 + 
                " hit dist along segment: " + (string)(gDistalongseg+hitbackedup)); 
            //  That's it. Now look for the other side of the obstacle.
            list obsendinfo = pathfindclearspace(interpt0, gCurrentix, gPathWidth, gPathHeight, gPathChartype);    // find far side of obstacle
            if (llGetListLength(obsendinfo) < 2)
            {   pathMsg(PATH_MSG_WARN,"Cannot find far side of obstacle at " + (string)interpt0 + " on segment #" + (string)(gCurrentix-1));
                if (interpt0 != llList2Vector(gPathPoints,-1))      // do not duplicate point. Will cause trouble in KFM
                {   gPathPoints += [interpt0];  }                   // best effort result
                gPts = [];                                          // release memory
                if (llGetListLength(gPathPoints) < 2) { gPathPoints = []; } // avoid single-point segment
                pathdeliversegment(gPathPoints, FALSE, TRUE, gReqPathid, PATHERRMAZEBADOBSTACLE);    // final set of points
                return(TRUE);                                    // partial result
            }
            //  Found point on far side, we have something for the maze solver.
            vector interpt1 = llList2Vector(obsendinfo,0);      // clear position on far side
            integer interp1ix = llList2Integer(obsendinfo,1);   // in this segment
            pathMsg(PATH_MSG_INFO,"Open space at seg #" + (string) interp1ix + " " + (string)interpt1); 
            //  Output points so far, then a maze.
            if (gPathPoints != [] && llVecMag(llList2Vector(gPathPoints,-1) - interpt0) >= 0.0001) // avoid zero length and divide by zero
            {
                gPathPoints += [interpt0];                       // segment up to start of maze is non-null
            }
            //  Cases here:
            //  - Two or more points stored - good segments before maze.
            //  - Zero points stored - do nothing. No segment to output.
            //  - One point stored - it should be the same as interpt0
            integer npts = llGetListLength(gPathPoints);
            if (npts > 1)            
            {   pathdeliversegment(gPathPoints, FALSE, FALSE, gReqPathid, 0);       // points so far, no maze, not done.
            } else if (npts == 1)                               // maze started at starting point
            {   // ***TEMP*** really
                ////pathMsg(PATH_MSG_WARN, "One point segment ignored: " + (string)llList2Vector(gPathPoints,-1) + " Maze start: " + (string)interpt0);
            }
            pathdeliversegment([interpt0, interpt1], TRUE, FALSE, gReqPathid, 0);// bounds of a maze area, maze, not done
            doingmaze = TRUE;                                   // maze solver is running, so quit here for now
            gPathPoints = [interpt1];                            // path clears and continues after maze
            if (llVecMag(interpt1 - llList2Vector(gPts,-1)) < 0.01)  // if at final destination
            {   gPts = [];                                      // release memory
                pathdeliversegment([], FALSE, TRUE, gReqPathid, 0);    // done, return final part of path ////****CAN RETURN ONE POINT PATH - BAD***
                return(TRUE);
            }
            assert(interp1ix < llGetListLength(gPts)-1);        // ix must never pass beginning of last segment
            //  Forward progress check to prevent infinite loop. Must either advance segment, or be further from start of current segment.
            assert(interp1ix > gCurrentix || (llVecMag(llList2Vector(gPts, gCurrentix) - gP0) < llVecMag(llList2Vector(gPts, gCurrentix) - interpt1)));
            gCurrentix = interp1ix;                             // continue from point just found
            gP0 = llList2Vector(gPts,gCurrentix);               // starting position in new segment
            gP1 = llList2Vector(gPts,gCurrentix+1);             // next position
            gDistalongseg = (interpt1-gP0) * llVecNorm(gP1-gP0);// how far along seg in segment direction
            assert(gDistalongseg >= 0);                         // should not be negative
            assert(checkcollinear([gP0,interpt1,gP1]));         // ***TEMP*** should be on the P0-P1 line
            assert(llFabs(llVecMag(interpt1 - gP0) - gDistalongseg) < 0.01);    // ***TEMP*** same as old way of calculating this?
            if (pathneedmem(PATHPLANMINMEM))                    // tight memory check, will quit early and retry later if necessary
            {   gPts = [];                                      // release memory
                pathdeliversegment([], FALSE, TRUE, gReqPathid, PATHERRMAZENOMEM);    // early quit, return final part of path with error
                return(TRUE);
            }
            //  End memory check.
        }
    }
    ////pathMsg(PATH_MSG_INFO, "Path advance end, more to do.");
    return(FALSE);                                          // not done, will do another advance later.
}

//
//  pathfindclearspace -- find clear space after obstacle
//
//  Returns [pos, segmentid] or [] if fail.
//
//  This is very difficult, if not impossible, to do perfectly.
//  llCastRay will not tell us if the starting position is inside an obstacle. So there's some guessing involved.
//  If we guess wrong, the problem will be detected when the character follows the path.
//
list pathfindclearspace(vector startpos, integer obstacleix, float width, float height, integer chartype)
{
    //  Dumb version. Just try the same check the maze solver uses, advancing along the path, until we find open space.
    integer len = llGetListLength(gPts);
    assert(obstacleix < len-1);                                     // index sanity check ***TEMP***
    vector p0 = llList2Vector(gPts,obstacleix);
    vector p1 = llList2Vector(gPts,obstacleix+1);
    vector pos = startpos;                                          // start search here
    vector prevpos = pos;                                           // need current and previous points
    //  Starting position is one extra width from start to provide some separation between start and finish.
    float distalongseg = llVecMag(pos - p0) + width;                // starting position, one extra width
    float seglength = llVecMag(p1-p0);
    integer currentix = obstacleix;                                 // working on segment 0
    while (TRUE)
    {   //  Advance one width
        distalongseg = distalongseg + width;                        // advance half width
        prevpos = pos;
        if (distalongseg > seglength)                               // if reached end of segment
        {   if (currentix >= len-2)                                 // must be able to advance
            {   return([]);                                         // reached end with no find. Caller reports error
            }
            currentix += 1;                                         // start next segment
            distalongseg = 0.0;                                     // start at next seg point, although arguably should advance around corner
        }
        p0 = llList2Vector(gPts,currentix);
        p1 = llList2Vector(gPts,currentix+1);
        seglength = llVecMag(p1-p0);                                // current segment length
        vector dir = llVecNorm(p1-p0);
        pos = p0+dir*distalongseg;                                  // next point to try
        // Adjust pos to be an integral number of widths from startpos. Movement is forward.
        float adjdistalongseg = pathcalccellmovedist(p0, dir, startpos, width, distalongseg);
        vector checkvec = (p0 + dir * adjdistalongseg) - startpos;                           // checking only
        float checkvecmag = llVecMag(<checkvec.x,checkvec.y,0.0>);                          // startpos to endpos of maze in XY plane
        if (adjdistalongseg >= 0 && adjdistalongseg <= seglength)   // if still on same segment
        {   assert(adjdistalongseg >= distalongseg);                // must progress forward
            distalongseg = adjdistalongseg;
            pos = p0 + dir * distalongseg;                           // should be an integral number of widths from startpos in 2D plane.
            if (checkvecmag > 2*width)                              // if far enough to be sure of an intervening maze square
            {
                //  Test the new point.  This test is not airtight because we are not testing from open space.
                //  May need further checks here.
                if (!obstaclecheckcelloccupied(prevpos, pos, TRUE))
                {   
                    return([pos,currentix]);                                // success, found open space
                }
            }
        }
    }
    return([]);                                                     // unreachable  
}
//
//  pathdeliversegment -- path planner has a segment to be executed
//
//  Maze segments must be two points. The maze solver will fill in more points.
//  Other segments must be one or more points.
//
pathdeliversegment(list path, integer ismaze, integer isdone, integer pathid, integer status)
{   DEBUGPRINT1("Pathdeliversegment: maze: " + (string)ismaze + " done: " + (string)isdone + " status: " + (string)status + " path: " + llDumpList2String(path,","));
    if (pathid != gPathId)                                  // starting a new path 
    {   gPathId = pathid;                                   // this is new pathid
        gSegmentId = 0;                                     // segment ID resets
    }
    integer length = llGetListLength(path);
    //  Fixed part of the reply. Just add "points" at the end.
    list fixedreplypart = ["reply","path", "pathid", gPathId, "status", status, "hitobj", gPathLastObstacle,
                "target", gPathplanTarget, "speed", gPathplanSpeed, "turnspeed", gPathplanTurnspeed,       // pass speed setting to execution module
                "refpt", gRefPt,                            // reference point for points (region corner)
                "points"];                                  // just add points at the end
    if (ismaze)                                             // maze, add to the to-do list
    {   assert(length == 2);                                // maze must have two endpoints
        vector bp0 = llList2Vector(path,0);
        vector bp1 = llList2Vector(path,1);
        //  Start the maze solver
        integer status = mazesolverstart(gRefPt, bp0, bp1, gPathWidth, gPathHeight, gPathChartype, gPathWidth, gPathLastObstacle, gPathId, gSegmentId); 
        if (status) 
        {   pathMsg(PATH_MSG_WARN,"Unable to start maze solver. Status: " + (string)status); 
            //  Create a dummy maze solve result and send it to path execution just to transmit the status.
            llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT,
            ["reply", "mazesolve", "pathid", pathid, "segmentid", gSegmentId, "status", status, 
                "pos", ZERO_VECTOR, "rot", ZERO_ROTATION, "cellsize", 0.0,
                "refpt",gRefPt, "points",llList2Json(JSON_ARRAY,[])]),"");
            return;
        }
        gSegmentId++;                                       // created a segment
    }
    else                                                    // non-maze, send to execution
    {
        if (path != [])                                     // if not the special EOF case
        {   llMessageLinked(LINK_THIS,MAZEPATHREPLY,
                llList2Json(JSON_OBJECT, ["segmentid", gSegmentId] + fixedreplypart + [llList2Json(JSON_ARRAY,path)]),"");
             gSegmentId++;                                           // created a segment

        } else {
            if (!isdone) { panic("Empty path, not done."); }
        }
    }
    if (isdone)
    {
        llMessageLinked(LINK_THIS,MAZEPATHREPLY,
            llList2Json(JSON_OBJECT, ["segmentid", gSegmentId] + fixedreplypart + [llList2Json(JSON_ARRAY,[ZERO_VECTOR])]),"");    // send one ZERO_VECTOR segment as an EOF.
        gSegmentId++;                                           // created a segment
    }
}


//
//  Globals for message interface
integer gPathId = 0;                                // serial number of path
integer gSegmentId = 0;                             // segment number of path
float gPathplanSpeed = 1.0;                         // defaults usually overridden
float gPathplanTurnspeed = 0.1;
key gPathplanTarget = NULL_KEY;                     // who we are chasing, if not null  
//
//  Main program of the path planning task
//
default
{

    state_entry()
    {   pathinitutils(); }                                              // library init

    //
    //  Incoming link message - will be a plan job request, or a notification of a maze solve completion
    //    
    link_message(integer status, integer num, string jsn, key id)
    {   if (num == PATHPLANPREPPED)                                     // if request for a planning job
        {   gPts = [];                                                  // release space from any previous cycle
            gPathplanTarget = (key)llJsonGetValue(jsn,["target"]);  // get target if pursue
            float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
            gTestspacing = (float)llJsonGetValue(jsn,["testspacing"]);
            integer pathid = (integer)llJsonGetValue(jsn,["pathid"]);
            gPathplanSpeed = (float)llJsonGetValue(jsn,["speed"]);
            gPathplanTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]);
            gPathStarttime = (integer)llJsonGetValue(jsn,["starttime"]);// time entire pathplan started, before retries 
            gRefPt = (vector)llJsonGetValue(jsn,["refpt"]);             // reference point for points (the region corner)
            list points = llJson2List(llJsonGetValue(jsn,["points"]));     // the preprocessed static path points
            ////pathMsg(PATH_MSG_INFO,"Path request: " + jsn); 
            jsn = "";                                               // Release string. We are that tight on space.
            //  Call the planner
            pathMsg(PATH_MSG_INFO,"Pathid " + (string)pathid + " starting."); 
            integer len = llGetListLength(points);
            integer i;
            for (i=0; i<len; i++) { gPts += (vector)llList2String(points,i);}   // convert to list of vectors
            points = [];                                    // release space
            ////pathMsg(PATH_MSG_INFO,"Preprocessed points: " + llDumpList2String(gPts,","));   // ***TEMP***
            pathplan(stopshort, pathid); 
            pathMsg(PATH_MSG_INFO,"Pathid " + (string)pathid + " req done.");   
        }             
        else if (num == MAZESOLVERREPLY)                    // just snooping on maze solves to see if we should do more planning
        {   integer pathid = (integer) llJsonGetValue(jsn, ["pathid"]); 
            integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
            integer status = (integer) llJsonGetValue(jsn, ["status"]);
            jsn = "";                                   // Release string, which is big here. 
            pathmazesolverdone(pathid, segmentid, status);  // maze solver is done, can do some more planning
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn);                            // initialize params
        } else if (num == DEBUG_MSGLEV_BROADCAST)               // set debug message level for this task
        {   debugMsgLevelSet(jsn);
        }
    }
}

