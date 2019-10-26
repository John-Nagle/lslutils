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
#include "npc/mazesolvercall.lsl"
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
float gWidth;                                               // dimensions of character
float gHeight;
integer gChartype;
float gTestspacing;
integer gReqPathid;                                         // path ID of planning section


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
pathplan(float stopshort, integer pathid)
{
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
{   pathMsg(PATH_MSG_INFO, "Maze solver done with pathid: " + (string)pathid + " segment: " + (string)segmentid + " status: " + (string)status);
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
integer pathplanadvance()
{   assert(gPts != []);                                         // must not call with no points                                                    
    integer doingmaze = FALSE;                                  // true if we started a maze solve
    while (!doingmaze)                                          // until a maze solve is started
    {
        float fulllength = llVecMag(gP1-gP0);                   // full segment length
        vector dir = llVecNorm(gP1-gP0);                        // direction of segment
        vector pos = gP0 + dir*gDistalongseg;                   // current working position
        vector endcast = gP1 + dir * gWidth*0.5;                // check to far side of area to avoid missing partial obstacle at end
        float hitdist = castbeam(pos, endcast, gWidth, gHeight, gTestspacing, TRUE,
                PATHCASTRAYOPTS);
        pathMsg(PATH_MSG_INFO,"Checked " + (string)pos + " to " + (string)endcast + " for obstacles. Hit dist: "+ (string)hitdist);
        if (hitdist < 0)
        {   gPts = [];                                          // release memory
            pathdeliversegment([], FALSE, TRUE, gReqPathid, MAZESTATUSCASTFAIL);    // empty set of points, no maze, done.
            return(TRUE);                                       // failure
        }
        if (hitdist == INFINITY)                                // completely clear segment
        {
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
            assert(hitdist >= 0.0);                             // ***TEMP*** 
            assert(gDistalongseg >= 0);                         // ***TEMP***
            float hitbackedup = hitdist-gWidth;                 // back up just enough to get clear
            assert(hitbackedup <= fulllength);                  // must be within segment limits
            vector interpt0 = pos + dir*(hitbackedup);          // back away from obstacle.
            pathMsg(PATH_MSG_INFO,"Hit obstacle at segment #" + (string)gCurrentix + " " + (string) interpt0 + 
                " hit dist along segment: " + (string)(gDistalongseg+hitbackedup)); 
            //  ***THESE TESTS ARE WRONG WHEN TWO MAZE SOLVES IN A ROW HAPPEN***
            //  ***MUST NEVER DECREASE gDistalongseg WITHIN A SEGMENT*******
            //  Constraints: Never back up past the beginning of a segment.
            //               Never back up past gdistalongsgement.
            //
            if (gDistalongseg > hitbackedup)                    // if this would be a backwards move          
            ////if (gDistalongseg + hitbackedup < 0)                // too close to beginning of current segment to back up
            {
                                                                // must search in previous segments
                if ((llVecMag(llList2Vector(gPts,0) - pos)) > (gWidth))  // if we are not very close to the starting point
                { 
                    // If we have to back up through a segment boundary, just give up and let the retry system handle it.
                    gPts = [];                              // release memory
                    if (llGetListLength(gPathPoints) < 2) { gPathPoints = []; } // avoid passing a one-point bogus segment forward
                    pathdeliversegment(gPathPoints,FALSE, TRUE, gReqPathid, MAZESTATUSBACKWARDS); // points, no maze, done
                    return(TRUE);
                    
                } else {                                   // we're at the segment start, and can't back up. Assume start of path is clear. We got there, after all.
                    pathMsg(PATH_MSG_WARN,"Assuming start point of path is clear at " + (string)gP0);
                    interpt0 = gP0;                             // zero length
                }
            }

#ifdef OBSOLETE
            if (gDistalongseg + hitbackedup < 0)                // too close to beginning of current segment to back up
            {
                                                                // must search in previous segments
                ////if ((llVecMag(llList2Vector(gPts,0) - interpt0)) > (width))  // if we are not very close to the starting point
                if ((llVecMag(llList2Vector(gPts,0) - pos)) > (gWidth))  // if we are not very close to the starting point
                { 
                    // If we have to back up through a segment boundary, just give up and let the retry system handle it.
                    gPts = [];                              // release memory
                    pathdeliversegment(gPathPoints,FALSE, TRUE, gReqPathid, MAZESTATUSBADSTART); // points, no maze, done
                    return(TRUE);
                    
                } else {                                   // we're at the segment start, and can't back up. Assume start of path is clear. We got there, after all.
                    pathMsg(PATH_MSG_WARN,"Assuming start point of path is clear at " + (string)gP0);
                    interpt0 = gP0;                             // zero length
                }
            }
#endif // OBSOLETE

            //  Search for the other side of the obstacle.                     
            list obsendinfo = pathfindclearspace(interpt0, gCurrentix, gWidth, gHeight, gChartype);    // find far side of obstacle
            if (llGetListLength(obsendinfo) < 2)
            {   pathMsg(PATH_MSG_WARN,"Cannot find far side of obstacle at " + (string)interpt0 + " on segment #" + (string)(gCurrentix-1));
                if (interpt0 != llList2Vector(gPathPoints,-1))      // do not duplicate point. Will cause trouble in KFM
                {   gPathPoints += [interpt0];  }                   // best effort result
                gPts = [];                                          // release memory
                if (llGetListLength(gPathPoints) < 2) { gPathPoints = []; } // avoid single-point segment
                pathdeliversegment(gPathPoints, FALSE, TRUE, gReqPathid, MAZESTATUSBADOBSTACLE);    // final set of points
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
                pathdeliversegment(gPathPoints, FALSE, FALSE, gReqPathid, 0);       // points so far, no maze, not done.
            }
            pathdeliversegment([interpt0, interpt1], TRUE, FALSE, gReqPathid, 0);// bounds of a maze area, maze, not done
            doingmaze = TRUE;                                   // maze solver is running, so quit here for now
            gPathPoints = [interpt1];                            // path clears and continues after maze
            ////if (llVecMag(interpt1 - llList2Vector(gPts,len-1)) < 0.01)  // if at final destination
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
            ////gDistalongseg = llVecMag(interpt1 - gP0);           // how far along seg 
            if (pathneedmem(PATHPLANMINMEM))                    // tight memory check, will quit early and retry later if necessary
            {   gPts = [];                                      // release memory
                pathdeliversegment([], FALSE, TRUE, gReqPathid, MAZESTATUSNOMEM);    // early quit, return final part of path with error
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
#ifdef OBSOLETE // need the memory space
        pathMsg(PATH_MSG_INFO,"Maze endpoint adjust at " + (string)prevpos + " " 
            + (string)pos + ". Dist along seg " + (string)distalongseg + " -> " + (string)adjdistalongseg + " 2D dist: " + 
            (string)checkvecmag + " seglength: " + (string)seglength);
#endif // OBSOLETE
        if (adjdistalongseg >= 0 && adjdistalongseg <= seglength)   // if still on same segment
        {   assert(adjdistalongseg >= distalongseg);                // must progress forward
            distalongseg = adjdistalongseg;
            pos = p0 + dir * distalongseg;                           // should be an integral number of widths from startpos in 2D plane.
            if (checkvecmag > 2*width)                              // if far enough to be sure of an intervening maze square
            {
                //  Test the new point.  This test is not airtight because we are not testing from open space.
                //  May need further checks here.
                if (!obstaclecheckcelloccupied(prevpos, pos, width, height, chartype, TRUE))
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
                "target", gPathplanTarget, "speed", gPathplanSpeed, "turnspeed", gPathplanTurnspeed,               // pass speed setting to execution module
                "width", gWidth, "height", gHeight, "chartype", gChartype, "msglev", gPathMsgLevel,
                "points"];                                  // just add points at the end
    if (ismaze)                                             // maze, add to the to-do list
    {   assert(length == 2);                                // maze must have two endpoints
        vector bp0 = llList2Vector(path,0);
        vector bp1 = llList2Vector(path,1);
        //  Start the maze solver
        integer status = mazesolverstart(bp0, bp1, gWidth, gHeight, gChartype, gWidth, gPathLastObstacle, gPathId, gSegmentId, gPathMsgLevel); 
        if (status) 
        {   pathMsg(PATH_MSG_ERROR,"Unable to start maze solver. Status: " + (string)status); 
            //  Create a dummy maze solve result and send it to path execution just to transmit the status.
            llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT,
            ["reply", "mazesolve", "pathid", pathid, "segmentid", gSegmentId, "status", status, 
                "pos", ZERO_VECTOR, "rot", ZERO_ROTATION, "cellsize", 0.0,
                "points",llList2Json(JSON_ARRAY,[])]),"");
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
    //
    //  Incoming link message - will be a plan job request, or a notification of a maze solve completion
    //    
    link_message(integer status, integer num, string jsonstr, key id)
    {   if (num == PATHPLANPREPPED)                                     // if request for a planning job
        {   gPts = [];                                                  // release space from any previous cycle
#ifdef OBSOLETE
            //  First, get stopped before we start planning the next move.
            llMessageLinked(LINK_THIS,MAZEPATHSTOP, "",NULL_KEY);       // tell execution system to stop
            integer i = 100;                                            // keep testing for 100 sleeps
            vector startpos = llGetPos();
            float poserr = INFINITY;                                    // position error, if still moving
            do                                                          // until character stops
            {   llSleep(0.3);                                           // allow time for message to execute task to stop KFM
                vector pos = llGetPos();
                poserr = llVecMag(startpos - pos);                      // how far did we move
                startpos = pos;
                ////pathMsg(PATH_MSG_INFO,"Waiting for character to stop moving.");
            } while (i-- > 0 && poserr > 0.001);                        // until position stabilizes 
            if (i <= 0) { pathMsg(PATH_MSG_WARN, "Character not stopping on command, at " + (string)startpos); }       
            //  Starting position and goal position must be on a walkable surface, not at character midpoint. 
            //  We just go down slightly less than half the character height.
            //  We're assuming a box around the character.      
            startpos = llGetPos();                                      // startpos is where we are now
            vector startscale = llGetScale();
            startpos.z = (startpos.z - startscale.z*0.45);              // approximate ground level for start point

            vector goal = (vector)llJsonGetValue(jsonstr,["goal"]);     // get goal point
#endif // OBSOLETE
            gPathplanTarget = (key)llJsonGetValue(jsonstr,["target"]);  // get target if pursue
            gWidth = (float)llJsonGetValue(jsonstr,["width"]);
            gHeight = (float)llJsonGetValue(jsonstr,["height"]);
            float stopshort = (float)llJsonGetValue(jsonstr,["stopshort"]);
            gChartype = (integer)llJsonGetValue(jsonstr,["chartype"]); // usually CHARACTER_TYPE_A, humanoid
            gTestspacing = (float)llJsonGetValue(jsonstr,["testspacing"]);
            integer pathid = (integer)llJsonGetValue(jsonstr,["pathid"]);
            gPathMsgLevel = (integer)llJsonGetValue(jsonstr,["msglev"]);
            gPathplanSpeed = (float)llJsonGetValue(jsonstr,["speed"]);
            gPathplanTurnspeed = (float)llJsonGetValue(jsonstr,["turnspeed"]);
            list points = llJson2List(llJsonGetValue(jsonstr,["points"]));     // the preprocessed static path points
            ////pathMsg(PATH_MSG_INFO,"Path request: " + jsonstr); 
            jsonstr = "";                                               // Release string. We are that tight on space.
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
        {   integer pathid = (integer) llJsonGetValue(jsonstr, ["pathid"]); 
            integer segmentid = (integer)llJsonGetValue(jsonstr,["segmentid"]);
            integer status = (integer) llJsonGetValue(jsonstr, ["status"]);
            jsonstr = "";                                   // Release string, which is big here. 
            pathmazesolverdone(pathid, segmentid, status);  // maze solver is done, can do some more planning
        }
    }
}

