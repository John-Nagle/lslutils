//
//  pathassemble.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  This is where the path components are assembled and 
//  sent to pathmove, which does the actual keyframe generation.
//
//  Speed and turning rate are set here, too.
//
//  There is asychrony here. Segments arrive from two sources - 
//  the path planner and the maze solver. Each segment has a 
//  sequence number. From each source, the segments are in
//  ascending order, but the two sources come in asychronously.
//
//  Animats
//  June, 2019
//
//  TODO:
//  1. Handle end of list with a ZERO_VECTOR at the end.        [DONE]
//  2. Report completion via link msg.                          [DONE]
//  3. Proper status handling.                                  [DONE]
//  4. Smooth slow to stop at end.                              [DONE]
//  5. Prevent rotation and position error accum between segs.  [DONE]
//  6. Add ray casting to detect obstacles when moving.         [DONE]
//  7. Add timer handing.                                       [DONE]
//
#include "npc/assert.lsl"                                   // assert
#include "npc/patherrors.lsl"
#include "npc/pathmazedefs.lsl"
#include "npc/pathbuildutils.lsl"
#include "npc/pathmovecall.lsl"
//
//  Constants
//
#define PATHMINTURNSECTION  0.5                             // first section of path this length, for a starting turn
#define PATHMAXKFMPOINTS    20                              // no more points than this per move to avoid overloading move task
//
//  Globals
//                                
integer gPathExeId = 0;                                     // current path ID
integer gPathExeNextsegid;                                  // current segment
integer gPathExeMoving = FALSE;                             // true if KFM movement in progress
integer gPathExeActive = FALSE;                             // path system is doing something
integer gPathExeEOF;                                        // EOF seen
integer gPathExePendingStatus;                              // pending error, to be reported at end of motion
////vector  gPathExeLastPos;                                    // last position, for stall check
integer gPathExeFreemem;                                    // amount of free memory left
////integer gPathLastTimetick = 0;                              // last time we tested for motion

//  Avatar params
float gPathExeMaxTurnspeed = 0.2;                           // (radians/sec) max turn rate - overridden
float gPathExeMaxSpeed = 2.0;                               // (meters/sec) max speed
key   gPathExeTarget = NULL_KEY;                            // who we are chasing, if anybody

float gPathExeDisttoend = 1.5;                              // (m) distance to begin turn ahead of corner
float gPathExeProbespacing = 0.333;                         // (m) cast ray spacing when looking for obstacles

//
//  Segment storage
//  Each segment list is of the form [segmentid, pntcount, pnt, pnt ... segmentid, pntcount, pnt, pnt ...]
//
list gClearSegments = [];                                   // path segment list
list gMazeSegments = [];                                    // maze segment list

list gAllSegments = [];                                     // combined segments from above, points only
list gRemainingSegments = [];                               // still need to be sent to move
////vector gPathExeMovegoal;                                    // where we are currently headed


//  Segment storage functions. If only we could pass references.
#define pathexeaddseg(lst, segnum, pts) { lst = lst + [(segnum), llGetListLength(pts)] + (pts); }   // add segment to list

#define pathexegetseg(lst) (llList2List(lst, 2, llList2Integer(lst,1) + 1)) // get first segment from list. Check length first. Expression.

//  Remove first segment of list. llList2List has strange semantics for start > end, so we have to test.
#define pathexedelseg(lst) { if (llList2Integer(lst,1) + 2 >= llGetListLength(lst)) { lst = []; } else { lst = llList2List(lst, llList2Integer(lst,1) + 2,-1); }}


//
//  pathexeinit -- set up path execute parameters
//
pathexeinit(float probespacing)
{   gPathSelfObject = pathGetRoot(llGetKey());              // us
    pathexestop(0);                                         // stop any operation in progress
    gPathExeProbespacing = probespacing;
    gPathExeNextsegid = 0;
    gPathExeFreemem = llGetFreeMemory();
}

//
//  pathexedeliver -- incoming path segment
//
//  Starts motion if necessary
//
//  A new pathid forces a flush of everything.
//
pathexedeliver(list pts, integer pathid, integer segmentid, integer ismaze, integer status, key hitobj)
{   DEBUGPRINT1("patheexedeliver, segment #" + (string)segmentid + " points: " + llDumpList2String(pts,","));
    integer length = llGetListLength(pts);
    if (length == 0 || (length == 1 && llList2Vector(pts,0) != ZERO_VECTOR))    // not enough points and not an EOF marker - why is this happening?
    {   pathMsg(PATH_MSG_WARN,"Bogus path segment, pathid: " + (string)pathid + " ismaze: " + (string)ismaze + 
            " segmentid: " + (string)segmentid + " pts: " +
            llDumpList2String(pts,","));
        gPathExeActive = TRUE;                              // so stop will return an error to pathcall
        pathexestop(PATHEXEBADPATH1); 
        return; 
    } // bogus 1 point path
    if (pathid != gPathExeId)                                // starting a new path, kill any movement
    {   //  Check for stale path ID.  Path ID wraps around but is always positive.
        if (pathid < gPathExeId || pathid-1000 > gPathExeId) { pathMsg(PATH_MSG_WARN,"Stale path segment " + (string)pathid + " ignored."); return; }// segment out of sequence
        //  Legit segment. Must either do it or return a completion to the caller.
        pathexestop(0);                                     // normal start, reset to clear state.
        gPathExeId = pathid;                                // now working on this pathid
        gPathExeEOF = FALSE;                                // not at EOF
        gPathExeActive = TRUE;                              // we are running
        gPathExePendingStatus = 0;                          // no stored status yet
        gPathLastObstacle = NULL_KEY;                       // no last obstacle yet
        ////gPathExeMovegoal = ZERO_VECTOR;                     // no stored goal yet
        if (segmentid == 0 && llList2Vector(pts,0) == ZERO_VECTOR)  // if this is an error situation at start
        {   pathMsg(PATH_MSG_WARN, "Path error at start: " + (string)status); 
            pathexestop(status);                            // and we fail out.
            return; 
        }

        vector verr = llList2Vector(pts,0) - llGetPos();    // get starting point
        if (llVecMag(<verr.x,verr.y,0>) > PATHSTARTTOL && segmentid == 0)     // if too far from current pos 
        {   pathMsg(PATH_MSG_WARN,"Bad start pos, segment " + (string)segmentid +
             " Should be at: " + (string)llList2Vector(pts,0) + " Current pos: " + (string)llGetPos());
            pathexestop(PATHEXEBADSTARTPOS);                // we are not where we are supposed to be.
            return; 
        }
    }
    if (gPathExePendingStatus == 0) { gPathExePendingStatus = status; }   // save any error status sent for later
    if (hitobj != NULL_KEY) { gPathLastObstacle = hitobj; } // save any obstacle id sent for later diagnosis
    if (ismaze)                                             // add to maze or path list
    {   pathexeaddseg(gMazeSegments, segmentid, pts); }
    else
    {   pathexeaddseg(gClearSegments, segmentid, pts);
        DEBUGPRINT1("Clear segments: " + llDumpList2String(gClearSegments,","));    // ***TEMP***
    }
    pathexedomove();                                        // advance path processing as needed 
}

//
//  pathexeextrapoints -- add extra points to path on long segments.
//
//  This is to allow smooth speed and direction changes.
//
list pathexeextrapoints(list pts, float distfromends, integer first)
{
    list newpts = [];
    integer i;
    integer length = llGetListLength(pts);
    assert(length > 0);                             // must have at least one point  
    vector p0 = llList2Vector(pts,0);
    newpts += [p0];                                 // always the initial point     
    for (i=1; i<length; i++)
    {   vector p1 = llList2Vector(pts,i);           // this pt
        //  Do we need to add any intermediate points?
        vector dv = p1-p0;
        vector dir = llVecNorm(dv);
        float dlen = llVecMag(dv);
        float distfromstart = distfromends;         // for first part of segment
        //  For first segment, use a very short first section
        //  so that turning takes place before acceleration.
        if (first && i == 1 && PATHMINTURNSECTION < distfromstart) 
        {   distfromstart = PATHMINTURNSECTION; }                      
        if (dlen >= 3*distfromends)                 // long enough to need 2 intermediate points
        {   newpts += [p0+dir*distfromstart, p1-dir*distfromends]; }
        else if (dlen >= distfromstart + distfromends)
        {   newpts += [(p0+p1)*0.5]; }              // add the midpoint 
        newpts += [p1];
        p0 = p1;                                    // for next cycle
    }
    assert(llList2Vector(newpts,-1) == llList2Vector(pts,-1));  // must preserve last point exactly
    return(newpts);
}



//
//  pathexegetsegment -- get next segment from either queue.
//
//  Must be segment segid.
//
list pathexegetsegment(integer segid)
{   DEBUGPRINT1("Getting segment #" + (string)segid);
    //  Try path segment queue
    if ((llGetListLength(gClearSegments) > 0) && llList2Integer(gClearSegments,0) == segid)
    {   list nextseg = pathexegetseg(gClearSegments); pathexedelseg(gClearSegments); return(nextseg); }
    //  Try maze segment queue
    if ((llGetListLength(gMazeSegments) > 0) && llList2Integer(gMazeSegments,0) == segid)
    {   list nextseg = pathexegetseg(gMazeSegments);  pathexedelseg(gMazeSegments); return(nextseg); }
    return([]); 
}
//
//  pathexeassemblesegs  -- combine segments into one big list.
//
//  A segment of [ZERO_VECTOR] means EOF.
//  That will result in a gAllSegments list of nothing but [ZERO_VECTOR] as the last
//
pathexeassemblesegs()
{   while (TRUE)
    {   list nextseg = pathexegetsegment(gPathExeNextsegid);    // get next segment if any
        DEBUGPRINT1("Assembling segment #" + (string)gPathExeNextsegid + ": " + llDumpList2String(nextseg,","));
        if (llGetListLength(nextseg) == 1 && llList2Vector(nextseg,0) == ZERO_VECTOR) // EOF sentinel - list with ZERO_VECTOR only
        {   gPathExeEOF = TRUE;                                 // we are at EOF
            DEBUGPRINT1("Segment EOF");
            nextseg = [];                                       // but must deliver the EOF flag later
        }
        if (nextseg == []) 
        {   if (gAllSegments == [] && gPathExeEOF) { gAllSegments = [ZERO_VECTOR]; } // done, deliver the EOF signal
            DEBUGPRINT1("Assembly complete: " + llDumpList2String(gAllSegments,","));
            return;                                             // caught up with input
        }
        if (llGetListLength(nextseg) == 1 && llList2Vector(nextseg,0) == ZERO_VECTOR && gAllSegments != [])
        { return; }                                             // if EOF marker, but segments left to do, use assembled segments
        //  We are going to use this segment.
        integer first =  gPathExeNextsegid==0;      // first segment?
        if (gAllSegments == [])
        {   gAllSegments = pathexeextrapoints(nextseg, gPathExeDisttoend, first);  // first segment
            DEBUGPRINT1("Started gAllSegments from empty: " + llDumpList2String(gAllSegments,",")); // ***TEMP***
            nextseg = [];
        } else {
            DEBUGPRINT1("Adding to gAllSegments: " + llDumpList2String(gAllSegments,",")); // ***TEMP***
            vector lastpt = llList2Vector(gAllSegments,-1);
            vector firstpt = llList2Vector(nextseg,0);
            vector errvec = lastpt-firstpt;
            //  Points are supposed to match. Unclear why they sometimes do not.
            //  Z after a maze solve may not quite match that of the path going in. Tolerance is half a height in mazesolve.
            if (llFabs(errvec.z) > gPathHeight*0.5 || llVecMag(<errvec.x,errvec.y,0.0>) > 0.05)
            {   ////pathMsg(PATH_MSG_ERROR, "Path assemble - endpoints do not match: " + (string)lastpt + (string)firstpt +
                ////" pathid: " + (string)gPathExeId + " segid: " + (string)gPathExeNextsegid);
                string s = "Path assemble - endpoints do not match: " + (string)lastpt + (string)firstpt + // ***TEMP***
                " pathid: " + (string)gPathExeId + " segid: " + (string)gPathExeNextsegid;              // ***TEMP*** because assert fail loses messages in queue
                llOwnerSay(s);                                  // backup way to get info out before panic //***TEMP***
                pathMsg(PATH_MSG_ERROR,s);
            }
            assert(llVecMag(lastpt-firstpt) < 1.00);            // 1m sanity check
            ////assert(llVecMag(<errvec.x,errvec.y,0.0>) < 0.20);   // 2D endpoints should match
            nextseg = llList2List(nextseg,1,-1);        // discard new duplicate point
            //  If we can take a short-cut at the join between two segments, do so.
            //  Also add "extra points" on long segments here for speed control.
            if (obstaclecheckpath(llList2Vector(gAllSegments,-2), llList2Vector(nextseg,0), gPathExeProbespacing))
            {   gAllSegments = llList2List(gAllSegments,0,-2) + pathexeextrapoints(nextseg, gPathExeDisttoend, first); }
            else
            {   gAllSegments += pathexeextrapoints(nextseg, gPathExeDisttoend, first);                // no, can't drop point
            }
        }
        gPathExeNextsegid++;                                    // advance seg ID
    }
}
//
//  pathexedomove -- feed in next section if any
//
pathexedomove()
{   if (gPathExeMoving) { return; }                     // we are moving, do nothing
    if (!gPathExeActive) { return; }                    // system is idle, do nothing
    pathexeassemblesegs();                              // have work to do?
    if (gRemainingSegments == [])                       // if none remain undone 
    {   if (llGetListLength(gAllSegments) == 1 && llList2Vector(gAllSegments,0) == ZERO_VECTOR) // if the EOF signal
        {   
            pathMsg(PATH_MSG_WARN,"Execute done. Lowest free mem: " + (string)gPathExeFreemem);
            gAllSegments = [];                              // all done 
            pathexestop(0);                                 // all done, normal stop
            return;
        } else {
           gRemainingSegments = gAllSegments;               // all segments become remaining segments
           gAllSegments = [];                               // not at EOF yet.
        }
    }
    if (gRemainingSegments != [])                       // if work
    {   
        ////vector kfmstart = llList2Vector(gAllSegments,0);    // first point, which is where we should be
        ////assert(kfmstart != ZERO_VECTOR);                    // must not be EOF marker
        ////gAllSegments = llListReplaceList(gAllSegments,[pos-<0,0,gPathHeight*0.5>],0,0);   // always start from current position
        //  Start keyframe motion and obstacle detection.
        //  Offloads the space explosion of keyframe generation to another script with more free space.
        {   list segstodo = [];                             // do these now
            if (llGetListLength(gRemainingSegments) > PATHMAXKFMPOINTS)                     // if too many to do all at once
            {   segstodo = llList2List(gRemainingSegments, 0, PATHMAXKFMPOINTS-1);          // do these now
                gRemainingSegments = llList2List(gRemainingSegments, PATHMAXKFMPOINTS-1,-1);// do these later. Overlap so obstacle scan works.
                pathMsg(PATH_MSG_WARN, "Too many points to move all at once, splitting list.");
            } else {                                                                // not too many
                segstodo = gRemainingSegments;                                            // do them all now
                gRemainingSegments = [];                        // fully consumed
            }
            vector pos = llGetPos();                            // we are here
            segstodo = llListReplaceList(segstodo,[pos-<0,0,gPathHeight*0.5>],0,0);   // always start from current position
            assert(llGetListLength(segstodo) >= 2);     // must always have at least two points
            pathmovestart(segstodo, gPathExeTarget, gPathExeMaxSpeed, gPathExeMaxTurnspeed, gPathExeId); 
        }     
        ////gPathExeMovegoal = llList2Vector(gAllSegments,-1);  // where we are supposed to be going
        ////assert(gPathExeMovegoal != ZERO_VECTOR);        // must not be EOF marker         
        gPathExeMoving = TRUE;                          // movement in progress
        integer freemem = llGetFreeMemory();            // how much memory left here, at the worst place       
        if (freemem < gPathExeFreemem) { gPathExeFreemem = freemem; }   // record free memory
        ////gAllSegments = [];                              // segments have been consumed
    } else {
        pathMsg(PATH_MSG_WARN,"Waiting for maze solver/planner to deliver pathid: " + (string)gPathExeId + " segid: " + (string)gPathExeNextsegid);    // solver running behind action
    }
}

//
//  pathexemovementend -- movement has finished, feed in next section if any
//
pathexemovementend()
{   if (gPathExeMoving)                                     // if was moving (KFM operation in progress)
    {
        gPathExeMoving = FALSE;                             // not moving now
        pathMsg(PATH_MSG_INFO,"Movement end");
        pathexedomove();                                    // get next KFM section if any and keep going
    } else {                                                // movement end event but we were not moving
        pathMsg(PATH_MSG_WARN,"Bogus movement end.");
    }
}

//
//  pathexestopkey -- trouble, stop and abort keyframe motion, with key
//
pathexestopkey(integer status, key hitobj)
{
    if (gPathExeMoving || (status != 0)) { pathMsg(PATH_MSG_WARN,"Movement stop. Status: " + (string)status); }
    ////llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);   // stop whatever is going on
    gClearSegments = [];                                    // reset state
    gMazeSegments = [];
    gAllSegments = [];
    gRemainingSegments = [];
    gPathExeNextsegid = 0; 
    gPathExeMoving = FALSE;                                 // not moving
    if (gPathExeActive)                                     // if we are active
    {   if (status == 0) { status = gPathExePendingStatus; }// send back any stored status
        pathdonereply(status,hitobj, gPathExeId);           // tell caller about result
        gPathExeActive = FALSE;                             // no longer active
        pathmovestop();                                     // turn off path moving
    }
}
//
//  pathexestop -- trouble, stop and abort keyframe motion, short form
//
pathexestop(integer status)
{   pathexestopkey(status, gPathLastObstacle); }

//
//  pathexemazedeliver  -- incoming maze result
//
//  This is a set of points, but they're in integer maze cell coordinates, and have to be converted to region coords.
//
pathexemazedeliver(string jsn) 
{
    DEBUGPRINT1("Maze deliver: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "mazesolve") { pathexestop(MAZESTATUSFORMAT); return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    //  If a move is stopped, the maze solver may still be running and sending maze solves.
    //  Discard such stale maze solves.
    if (pathid != gPathExeId && segmentid != 0)                     // if stale result
    {   ////pathMsg(PATH_MSG_WARN,"Discarding stale maze solve result.");
        return;
    }
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    ///if (status != 0) { pathexestop(status); return; }                  // error status from other side
    if (status != 0) { pathexedeliver([ZERO_VECTOR],pathid, segmentid, TRUE, status, NULL_KEY); return; } // EOF with error, needed if first segment is bad.
    float cellsize = (float)llJsonGetValue(jsn,["cellsize"]); // get maze coords to world coords info
    vector pos = (vector)llJsonGetValue(jsn,["pos"]);
    rotation rot = (rotation)llJsonGetValue(jsn,["rot"]);
    key hitobj = (key)llJsonGetValue(jsn,["hitobj"]);           // obstacle which started (not stopped) maze solve
    list ptsmaze = llJson2List(llJsonGetValue(jsn, ["points"])); // points, one per word
    vector p0 = (vector)llJsonGetValue(jsn,["p0"]);            // for checking only
    vector p1 = (vector)llJsonGetValue(jsn,["p1"]);            // for checking only
    list ptsworld = [];
    integer i;
    integer length = llGetListLength(ptsmaze);              // number of points
    assert(length >= 2);                                    // must have at least 2 points
    for (i=0; i<length; i++)
    {   integer val = llList2Integer(ptsmaze,i);            // X and Y encoded into one integer
        DEBUGPRINT1("Maze solve pt: (" + (string)mazepathx(val) + "," + (string)mazepathy(val) + ")");
        vector cellpos = mazecellto3d(mazepathx(val), mazepathy(val), cellsize, pos, rot);                        // convert back to 3D coords
        cellpos.z = mazepathz(val,pos.z);                   // get Z value from the maze solver.
        ptsworld += [cellpos];                              // accum list of waypoints
    }
    pathMsg(PATH_MSG_INFO, "Maze solve pts: " + llDumpList2String(ptsworld,","));       // ***TEMP*** detailed debug
    assert(llGetListLength(ptsworld) >= 2);                             // must have at least two points    
    assert(llVecMag(llList2Vector(ptsworld,0) - p0) < 0.01);            // maze endpoints must match
    vector p1err = llList2Vector(ptsworld,-1) - p1;                     // ***TEMP***
    if (llVecMag(<p1err.x,p1err.y,0>) >= 0.01)                           // ***TEMP*** Z isn't an exact match - check
    {   pathMsg(PATH_MSG_ERROR,"p1 wrong at end of maze solve: p1: " + (string)p1 + " maze last pt: " + (string)llList2Vector(ptsworld,-1));
        llSleep(2.0);                                                   // allow time for message processing before assert fails.
    }                                                                   // ***END TEMP***
    ////assert(llVecMag(llList2Vector(ptsworld,-1) - p1) < 0.01);            // maze endpoints must match
    //  Straighten path. Maze solver paths are all right angles. Here we try short cuts.
    ptsworld = pathstraighten(ptsworld, gPathExeProbespacing);          // postprocess path
    pathexedeliver(ptsworld, pathid, segmentid, TRUE, 0, hitobj);      // deliver maze solution
}
//
//  pathexepathdeliver  -- JSON from path planner
//
pathexepathdeliver(string jsn) 
{   pathMsg(PATH_MSG_INFO,"Path deliver received: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "path") { pathexestop(MAZESTATUSFORMAT); return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    //  Results from the path planner also contain some misc, parameter updates.
    gPathExeTarget = (key)llJsonGetValue(jsn,["target"]);                               // used by move task to check if pursue target moves
    gPathExeMaxSpeed = (float)llJsonGetValue(jsn,["speed"]); 
    gPathExeMaxTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]); 
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    if (status != 0) 
    {   pathMsg(PATH_MSG_WARN,"Path deliver with status " + (string)status); 
        if (gPathExePendingStatus == 0)                     // save error status for later
        {   gPathExePendingStatus = status;
        }   // save any error status sent for later
    }
    key hitobj = (key)llJsonGetValue(jsn, ["hitobj"]);      // what stopped us, if anything
    ////if (hitobj != NULL_KEY) { gPathLastObstacle = hitobj; } // save last obstacle
    list ptsstr = llJson2List(llJsonGetValue(jsn, ["points"])); // points, as strings
    list pts = [];
    integer i;
    integer len = llGetListLength(ptsstr);
    for (i=0; i<len; i++) { pts += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors  
    pathexedeliver(pts, pathid, segmentid, FALSE, status, hitobj);      // deliver path segment
}

//
//  pathexemovereply -- reply from path mover
//
//  Move ending, collisons, and ray cast obstacle detections come in this way
//
pathexemovereply(string jsn)
{
    string replytype = llJsonGetValue(jsn,["reply"]);   // request type
    if (replytype != "movedone") {  return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer status = (integer)llJsonGetValue(jsn, ["status"]);
    key hitobj = (key)llJsonGetValue(jsn,["hitobj"]);       // what was hit, if anything
    if (pathid != gPathExeId)                               // ignore stale event
    {   pathMsg(PATH_MSG_WARN, "Stale reply from path move, status " + (string)status); 
        return;
    }
    if (status == 0)                                        // status zero, no problem
    {   pathexemovementend(); return; }                     // normal movement end   
    pathexestopkey(status, hitobj);                         // otherwise hit something   
}

//
float TESTSPACING = 0.33;                                   // 3 test points per meter in height
float PATHSTARTTOL = 0.5;                                   // if we are out of position, can't start 
//
//  The main program of the execute task.
//
default
{
    state_entry()
    {   pathinitutils();                                    // init library
        pathexeinit(TESTSPACING);                           // init our KFM system        
    }

    link_message(integer status, integer num, string jsn, key id )
    {   if (num == MAZESOLVERREPLY)                     // maze solve result
        {   DEBUGPRINT1("Maze solver deliver: " + jsn);
            pathexemazedeliver(jsn);
        } else if (num == MAZEPATHREPLY)
        {   DEBUGPRINT1("Path deliver: " + jsn);
            pathexepathdeliver(jsn); 
        } else if (num == LINKMSGMOVEREPLY)
        {   DEBUGPRINT1("Move reply: " + jsn);
            pathexemovereply(jsn);
        } else if (num == MAZEPATHSTOP)                 // planner wants us to stop
        {   pathexestop(0);                             // normal stop commanded
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                      // initialize params

    }
}
