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
#ifndef PATHPLANLSL                                     // include guard, like C/C++
#define PATHPLANLSL
#include "npc/pathbuildutils.lsl"                           // common functions


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
pathplan(vector startpos, vector endpos, float width, float height, float stopshort, integer chartype, float testspacing, integer pathid)
{                              
    //  Use the system's GetStaticPath to get an initial path
    ////list pts = llGetStaticPath(startpos, endpos, width + PATHSTATICTOL, [CHARACTER_TYPE, chartype]);  // generate path
    list pts = pathtrimmedstaticpath(startpos, endpos, stopshort, width + PATHSTATICTOL, chartype);
    pathMsg(PATH_MSG_INFO,"Static path: " + llDumpList2String(pts,","));     // dump list for debug
    integer status = llList2Integer(pts,-1);                // last item is status
    if (status != 0)                                        // static path fail
    {   pathdeliversegment([ZERO_VECTOR], FALSE, TRUE, pathid, status);// report error
        return;
    }
    //  Got path
    pts = llList2List(pts,0,-2);                            // drop status from end of points list
    pts = pathclean(pts);                                   // 
    integer len = llGetListLength(pts);
    if (len < 2)
    {   
        pathdeliversegment([ZERO_VECTOR], FALSE, TRUE, pathid, MAZESTATUSNOPTS);        // empty set of points, no maze, done.
        return;                                             // empty list
    }
    //  We have a valid static path. Now start checking for obstacles.
    pathMsg(PATH_MSG_INFO,"Path check for obstacles. Segments: " + (string)len); 
    vector p0 = llList2Vector(pts,0);                       // starting position
    list pathPoints = [p0];                                 // output points
    vector p1 = llList2Vector(pts,1);                       // next position
    float distalongseg = 0.0;                               // starting position, one extra width
    integer currentix = 0;                                  // working on segment 0
    while (TRUE)                                            // until return
    {   //  Check segment for obstacles, going forward.
        float fulllength = llVecMag(p1-p0);                 // full segment length
        vector dir = llVecNorm(p1-p0);                      // direction of segment
        vector pos = p0 + dir*distalongseg;                 // current working position
        DEBUGPRINT1("Checking " + (string)pos + " to " + (string)p1 + " for obstacles.");
        float hitdist = castbeam(pos, p1, width, height, testspacing, TRUE,
                    PATHCASTRAYOPTS);
        if (hitdist < 0)
        {  
            pathdeliversegment([ZERO_VECTOR], FALSE, TRUE, pathid, MAZESTATUSCASTFAIL);    // empty set of points, no maze, done.
            return;                                         // failure
        }    
        if (hitdist == INFINITY)                            // completely clear segment
        {
            pathPoints += [p1];                             // completely empty segment
            currentix += 1;                                 // advance to next segment
            if (currentix >= len-1)                         // done
            {   pathdeliversegment(pathPoints, FALSE, TRUE, pathid, 0);// points, not a maze, final.
                return;                                     // return strided list of path segments
            }
            p0 = llList2Vector(pts,currentix);              // starting position in new segment
            p1 = llList2Vector(pts,currentix+1);            // next position
            distalongseg = 0.0;                             // starting new segment
        } else {                                            // there is an obstruction
            ////float hitbackup = hitdist-width*0.5;            // back up just enough to get clear
            float hitbackup = hitdist-width;                // back up just enough to get clear
            vector interpt0 = pos + dir*(hitbackup);        // back away from obstacle.
            pathMsg(PATH_MSG_INFO,"Hit obstacle at segment #" + (string)currentix + " " + (string) interpt0 + 
                " hit dist along segment: " + (string)(distalongseg+hitbackup)); 
            if (distalongseg + hitbackup < 0)               // too close to beginning of current segment to back up
            {                                               // must search in previous segments
                if ((llVecMag(llList2Vector(pts,0) - interpt0)) > (width))  // if we are not very close to the starting point
                {   list pinfo =  pathfindunobstructed(pts, currentix, -1, width, height);
                    interpt0 = llList2Vector(pinfo,0);          // open space point before obstacle, in a prevous segment
                    integer newix = llList2Integer(pinfo,1);    // segment in which we found point, counting backwards
                    pathMsg(PATH_MSG_INFO,"Pathcheckobstacles backing up from segment #" + (string)currentix + " to #" + (string) newix);
                    if (newix < 1)
                    {   pathdeliversegment(pathPoints,FALSE, TRUE, pathid, MAZESTATUSBADSTART); // points, no maze, done
                        return;                                 // no open space found, fail
                    }
                    //  Discard points until we find the one that contains the new intermediate point.
                    vector droppedpoint = ZERO_VECTOR;          // point we just dropped
                    do {
                        pathMsg(PATH_MSG_INFO,"Dropping point " + (string)llList2Vector(pathPoints,-1) + " from pathPoints looking for " + (string)interpt0);
                        droppedpoint = llList2Vector(pathPoints,-1);
                        pathPoints = llListReplaceList(pathPoints,[],llGetListLength(pathPoints)-1, llGetListLength(pathPoints)-1);
                        //  ***WRONG*** looks like an off by one error. 
                    } while ( llGetListLength(pathPoints) > 0 && !pathpointinsegment(interpt0,droppedpoint,llList2Vector(pathPoints,-1)));
                } else {                                    // we're at the segment start, and can't back up. Assume start of path is clear. We got there, after all.
                    pathMsg(PATH_MSG_INFO,"Assuming start point of path is clear at " + (string)interpt0);
                    interpt0 = p0;                              // zero length
                }
            }
            //  Search for the other side of the obstacle.                     
            DEBUGPRINT1("Looking for open space on far side of obstacle.");
            list obsendinfo = pathfindclearspace(pts, interpt0, currentix, width, height);    // find far side of obstacle
            if (llGetListLength(obsendinfo) < 2)
            {   pathMsg(PATH_MSG_INFO,"Cannot find open space after obstacle at " + (string)interpt0 + " on segment #" + (string)(currentix-1));
                pathPoints += [interpt0];                       // best effort resul
                pathdeliversegment(pathPoints, FALSE, TRUE, pathid, MAZESTATUSBADOBSTACLE);    // final set of points
                return;                                         // partial result
            }
            //  Found point on far side, we have something for the maze solver.
            vector interpt1 = llList2Vector(obsendinfo,0);      // clear position on far side
            integer interp1ix = llList2Integer(obsendinfo,1);   // in this segment
            pathMsg(PATH_MSG_INFO,"Found open space at segment #" + (string) interp1ix + " " + (string)interpt1); 
            //  Output points so far, then a maze.
            if (pathPoints != [] && llVecMag(llList2Vector(pathPoints,-1) - interpt0) >= 0.0001) // avoid zero length and divide by zero
            {
                pathPoints += [interpt0];                       // segment up to start of maze
            }
            pathdeliversegment(pathPoints, FALSE, FALSE, pathid, 0);       // points so far, no maze, not done.
            pathdeliversegment([interpt0, interpt1], TRUE, FALSE, pathid, 0);// bounds of a maze area, maze, not done
            pathPoints = [interpt1];                            // path clears and continues after maze
            if (llVecMag(interpt1 - llList2Vector(pts,len-1)) < 0.01)  // if at final destination
            {   pathdeliversegment(pathPoints, FALSE, TRUE, pathid, 0);    // done, return final part of path
                return;
            }
            assert(interp1ix < len-1);                          // ix must never pass beginning of last segment
            //  Forward progress check to prevent infinite loop. Must either advance segment, or be further from start of current segment.
            assert(interp1ix > currentix || (llVecMag(llList2Vector(pts, currentix) - p0) < llVecMag(llList2Vector(pts, currentix) - interpt1)));
            currentix = interp1ix;                              // continue from point just found
            p0 = llList2Vector(pts,currentix);                  // starting position in new segment
            p1 = llList2Vector(pts,currentix+1);                // next position
            distalongseg = llVecMag(interpt1 - p0);             // how far along seg 
        }
    }
    //  Not reached.
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
list pathfindclearspace(list pts, vector startpos, integer obstacleix, float width, float height)
{
    //  Dumb version. Just try the same check the maze solver uses, advancing along the path, until we find open space.
    integer len = llGetListLength(pts);
    vector p0 = llList2Vector(pts,obstacleix);
    vector p1 = llList2Vector(pts,obstacleix+1);
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
        p0 = llList2Vector(pts,currentix);
        p1 = llList2Vector(pts,currentix+1);
        seglength = llVecMag(p1-p0);                                // current segment length
        vector dir = llVecNorm(p1-p0);
        pos = p0+dir*distalongseg;                                  // next point to try
        // Adjust pos to be an integral number of widths from startpos. Movement is forward.
        float adjdistalongseg = pathcalccellmovedist(p0, dir, startpos, width, distalongseg);
        vector checkvec = (p0 + dir * adjdistalongseg) - startpos;                           // checking only
        float checkvecmag = llVecMag(<checkvec.x,checkvec.y,0.0>);                          // startpos to endpos of maze in XY plane
        pathMsg(PATH_MSG_INFO,"Maze endpoint adjust. Dist along seg " + (string)distalongseg + " -> " + (string)adjdistalongseg + " 2D dist: " + 
            (string)checkvecmag);
        if (adjdistalongseg >= 0 && adjdistalongseg <= seglength)   // if still on same segment
        {   assert(adjdistalongseg >= distalongseg);                // must progress forward
            distalongseg = adjdistalongseg;
            pos = p0 + dir * distalongseg;                           // should be an integral number of widths from startpos in 2D plane.
            if (checkvecmag > 2*width)                              // if far enough to be sure of an intervening maze square
            {
                //  Test the new point.  This test is not airtight because we are not testing from open space.
                //  May need further checks here.
                if (!obstaclecheckcelloccupied(prevpos, pos, width, height, TRUE))
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
    if (ismaze)                                             // maze, add to the to-do list
    {   assert(length == 2);                                // maze must have two endpoints
        vector bp0 = llList2Vector(path,0);
        vector bp1 = llList2Vector(path,1);
        //  Start the maze solver
        integer status = mazesolverstart(bp0, bp1, gPathWidth, gPathHeight, gPathWidth, gPathId, gSegmentId, gPathMsgLevel); 
        if (status) 
        {   pathMsg(PATH_MSG_ERROR,"Unable to start maze solver. Status: " + (string)status); 
            //  Create a dummy maze solve result and send it to path execution just to transmit the status.
            llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT,
            ["reply", "mazesolve", "pathid", pathid, "segmentid", gSegmentId, "status", status,
                "pos", ZERO_VECTOR, "rot", ZERO_ROTATION, "cellsize", 0.0,
                "points",llList2Json(JSON_ARRAY,[])]),"");
            return;
        }
    }
    else                                                    // non-maze, send to execution
    {
        llMessageLinked(LINK_THIS,MAZEPATHREPLY,
            llList2Json(JSON_OBJECT, ["reply","path", "pathid", gPathId, "segmentid", gSegmentId, "status", status, 
            "speed", gPathplanSpeed, "turnspeed", gPathplanTurnspeed,               // pass speed setting to execution module
            "width", gPathWidth, "height", gPathHeight, "chartype", gPathplanChartype, "msglev", gPathMsgLevel,
            "points", llList2Json(JSON_ARRAY,path)]),"");
    }
    gSegmentId++;                                           // next segment
    if (isdone)
    {
        llMessageLinked(LINK_THIS,MAZEPATHREPLY,
            llList2Json(JSON_OBJECT, ["reply","path", "pathid", gPathId, "segmentid", gSegmentId,
            "speed", gPathplanSpeed, "turnspeed", gPathplanTurnspeed,               // pass speed setting to execution module
            "width", gPathWidth, "height", gPathHeight, "chartype", gPathplanChartype, "msglev", gPathMsgLevel,
            "status",status, "points",
            llList2Json(JSON_ARRAY,[ZERO_VECTOR])]),"");    // send one ZERO_VECTOR segment as an EOF.
    }
}
//
//  Globals for message interface
integer gPathId = 0;                                // serial number of path
integer gSegmentId = 0;                             // segment number of path
float gPathplanSpeed = 1.0;                         // defaults usually overridden
float gPathplanTurnspeed = 0.1;
integer gPathplanChartype = CHARACTER_TYPE_A;         
//
//  Message interface
//
//  pathRequestRecv -- link message starts action here.
//
pathRequestRecv(string jsonstr)
{   
    //  Starting position and goal position must be on a walkable surface, not at character midpoint.
    vector startpos = (vector)llJsonGetValue(jsonstr,["startpos"]);   // get starting position
    vector goal = (vector)llJsonGetValue(jsonstr,["goal"]);   // get goal
    gPathWidth = (float)llJsonGetValue(jsonstr,["width"]);
    gPathHeight = (float)llJsonGetValue(jsonstr,["height"]);
    float stopshort = (float)llJsonGetValue(jsonstr,["stopshort"]);
    gPathplanChartype = (integer)llJsonGetValue(jsonstr,["chartype"]); // usually CHARACTER_TYPE_A, humanoid
    float testspacing = (float)llJsonGetValue(jsonstr,["testspacing"]);
    integer pathid = (integer)llJsonGetValue(jsonstr,["pathid"]);
    gPathMsgLevel = (integer)llJsonGetValue(jsonstr,["msglev"]);
    gPathplanSpeed = (float)llJsonGetValue(jsonstr,["speed"]);
    gPathplanTurnspeed = (float)llJsonGetValue(jsonstr,["turnspeed"]);
    pathMsg(PATH_MSG_INFO,"Path request: " + jsonstr); 

    //  Call the planner 
    pathplan(startpos, goal, gPathWidth, gPathHeight, stopshort, gPathplanChartype, testspacing, pathid);    
}


#endif // PATHPLANLSL
