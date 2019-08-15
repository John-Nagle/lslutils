//
//  pathexecute.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  This is where the path components are assembled and keyframe
//  animation lists are created and executed.
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
//  2. Report completion via link msg
//  3. Proper status handling.
//  4. Smooth slow to stop at end.
//  5. Prevent rotation and position error accumulation between segs.
//  6. Add ray casting to detect obstacles when moving.
//  7. Add timer handing.
//
#ifndef PATHEXECUTE                                         // include guard, like C/C++
#define PATHEXECUTE
#include "npc/assert.lsl"                                   // assert
#include "npc/mazedefs.lsl"
//
//  Constants
//
//  Error codes. Move to pathdefs.
#define PATHEXEBADPATH1     -2001
#define PATHEXESEGOUTOFSEQ2 -2002
#define PATHEXESEGOUTOFSEQ1 -2003
#define PATHEXEBADSTARTPOS  -2004
#define PATHEXECOLLISION    -2005
//
//  Idle timer timeout
//
#define PATHEXETIMEOUT      2.0                             // check this often for progress
//
//  Globals
//                                
float gMaxTurnRate = 0.2;                                   // (radians/sec) max turn rate
float gMaxSpeed = 2.0;                                      // (meters/sec) max speed
integer gPathExeId = 0;                                     // current path ID
integer gPathExeNextsegid;                                  // current segment
integer gPathExeMoving = FALSE;                             // true if KFM movement in progress
integer gPathExeActive = FALSE;                             // path system is doing something
integer gPathExeEOF;                                        // EOF seen
integer gPathExePendingStatus;                              // pending error, to be reported at end of motion
vector  gPathExeLastPos;                                    // last position, for stall check
integer gPathExeVerbose;                                    // verbose mode
integer gPathExeFreemem;                                    // amount of free memory left

//  Avatar params
float gPathExeWidth;
float gPathExeHeight;
float gPathExeProbespacing;
integer gPathExeChartype;
float gPathExeDisttoend = 1.5;                              // (m) distance to begin turn ahead of corner
//
//  Segment storage
//  Each segment list is of the form [segmentid, pntcount, pnt, pnt ... segmentid, pntcount, pnt, pnt ...]
//
list gClearSegments = [];                                   // path segment list
list gMazeSegments = [];                                    // maze segment list

list gAllSegments = [];                                     // combined segments from above, points only

//  Segment storage functions. If only we could pass references.
#define pathexeaddseg(lst, segnum, pts) { lst = lst + [(segnum), llGetListLength(pts)] + (pts); }   // add segment to list

#define pathexegetseg(lst) (llList2List(lst, 2, llList2Integer(lst,1) + 1)) // get first segment from list. Check length first. Expression.

//  Remove first segment of list. llList2List has strange semantics for start > end, so we have to test.
#define pathexedelseg(lst) { if (llList2Integer(lst,1) + 2 >= llGetListLength(lst)) { lst = []; } else { lst = llList2List(lst, llList2Integer(lst,1) + 2,-1); }}

rotation NormRot(rotation Q)
{    
    float MagQ = llSqrt(Q.x*Q.x + Q.y*Q.y +Q.z*Q.z + Q.s*Q.s);    
    Q.x = Q.x/MagQ;    
    Q.y = Q.y/MagQ;    
    Q.z = Q.z/MagQ;    
    Q.s = Q.s/MagQ;
    return Q;
}

//
//  Debug marker generation
//
#ifdef MARKERS
string MARKERLINE = "Path marker, rounded (TEMP)";
integer LINKMSGMARKER = 1001;                               // for sending to marker service

//  Colors (RGBA)
rotation TRANSGREEN = <0,0.35,0,0.5>;                          // translucent green
rotation TRANSRED   = <1,0,0,0.5>;
rotation TRANSYELLOW = <1,1,0,0.5>;



//
//  Create a marker with the requested parameters
//
//  Rez now, wait for message, send details to marker.
//  We can't send enough data during the rez. We have to use a message for the params.
//
//
//  placesegmentmarker - mark one segment of a path
//
//  "Color" is RGBA.
//
placesegmentmarker(string markername, vector p0, vector p1, rotation rgba, float thickness)
{
    //  Draw marker for segment
    vector midpoint = (p0+p1)*0.5;          // midpoint
    float length = llVecMag(p1-p0);         // how long
    
    vector color = <rgba.x, rgba.y, rgba.z>;
    float alpha = rgba.s;
    vector scale = <length,CHARRADIUS*2,thickness>;    // size of marker
    list params = [ "pos", midpoint, "rot", rotperpenonground(p0,p1), "scale", scale, "color", color, "alpha", alpha];
    llMessageLinked(LINK_THIS, LINKMSGMARKER, llList2Json(JSON_OBJECT,params), markername);   // ask marker service to place a marker.   
}
#endif // MARKERS

//
//  pathexeinit -- set up path execute parameters
//
pathexeinit(float speed, float turnrate, float width, float height, float probespacing, integer chartype, integer verbose)
{
    pathexestop(0);                                          // stop any operation in progress
    gMaxSpeed = speed;
    gMaxTurnRate = turnrate; 
    gPathExeWidth = width;
    gPathExeHeight = height;
    gPathExeProbespacing = probespacing;
    gPathExeChartype = chartype;
    gPathExeNextsegid = 0;
    gPathExeVerbose = verbose;
    gPathExeFreemem = llGetFreeMemory();   
}


//
//  pathexedeliver -- incoming path segment
//
//  Starts motion if necessary
//
//  A new pathid forces a flush of everything.
//
pathexedeliver(list pts, integer pathid, integer segmentid, integer ismaze, integer status)
{   DEBUGPRINT1("patheexedeliver, segment #" + (string)segmentid + " points: " + llDumpList2String(pts,","));
    if (llGetListLength(pts) < 2 && pts != [ZERO_VECTOR]) { pathexestop(PATHEXEBADPATH1); return; } // bogus path
    if (pathid != gPathExeId)                                // starting a new segment, kill any movement
    {   if (segmentid != 0) { pathexestop(PATHEXESEGOUTOFSEQ1); return; }// segment out of sequence
        pathexestop(0);                                     // normal start
        gPathExeId = pathid;                                // reset state to empty
        gPathExeEOF = FALSE;                                // not at EOF
        gPathExeActive = TRUE;                              // we are running
        gPathExePendingStatus = 0;                          // no stored status yet
        llSetTimerEvent(PATHEXETIMEOUT);                    // periodic stall timer
    }
    if (segmentid == 0)                                     // starting a new path
    {   DEBUGPRINT1("Starting new path."); // ***TEMP***
        if (gClearSegments != [] || gMazeSegments != [])    // so why do we have stored segments?
        {   pathexestop(PATHEXESEGOUTOFSEQ2); return; }     // bad
        DEBUGPRINT1("First point: " + (string)llList2Vector(pts,0) + " Current pos: " + (string)llGetPos());
        vector verr = llList2Vector(pts,0) - llGetPos();    // get starting point
        if (llVecMag(<verr.x,verr.y,0>) > PATHSTARTTOL)     // if too far from current pos
        {   pathexestop(PATHEXEBADSTARTPOS); return; }      // bad start position
    }
    if (gPathExePendingStatus == 0) { gPathExePendingStatus = status; }   // save any error status sent for later
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
list pathexeextrapoints(list pts, float distfromend)
{
    list newpts = [];
    integer i;
    integer length = llGetListLength(pts);   
    vector p0 = llList2Vector(pts,0);
    newpts += [p0];                                 // always the initial point     
    for (i=1; i<length; i++)
    {   vector p1 = llList2Vector(pts,i);           // this pt
        //  Do we need to add any intermediate points?
        vector dv = p1-p0;
        vector dir = llVecNorm(dv);
        float dlen = llVecMag(dv);                          
        if (dlen >= 3*distfromend)                  // long enough to need 2 intermediate points
        {   newpts += [p0+dir*distfromend, p1-dir*distfromend]; }
        else if (dlen >= 2*distfromend)
        {   newpts += [(p0+p1)*0.5]; }              // add the midpoint 
        newpts += [p1];
        p0 = p1;                                    // for next cycle
    }
    return(newpts);
}

//
//  pathexebuildkfm  -- build keyframe motion list from points
//
list pathexebuildkfm(vector startpos, rotation startrot, list pts)
{
    list kfmdata = [];                          // [pos, rot, time ... ]
    integer i;
    integer length = llGetListLength(pts);
    vector pos = startpos;
    rotation rot = startrot;
    //  Vectors off the end are ZERO_VECTOR. Code relies on this.
    for (i=1; i<length; i++)                    // skip 1, we are already there.
    {   vector pprev = llList2Vector(pts,i-1);  // previous point
        vector p0 = llList2Vector(pts,i);       // point we are going to
        vector p1 = llList2Vector(pts,i+1);
        kfmdata += pathexecalckfm(pos, rot, pprev, p0, p1);
        pos += llList2Vector(kfmdata,-3);       // update pos in world coords
        rot *= llList2Rot(kfmdata,-2);          // update rot in world coords      
        pprev = p0;
    }
    return(kfmdata);                            // list ready for KFM
}

//
//  pathexecalckfm -- calc the keyframe parameters for one point
//
//  We are moving from pprev to p0.
//
//  ***NEED TO PREVENT ROTATION ERROR ACCUMULATION***
//  ***NEED TO ROTATE BEFORE THE FIRST MOVE***
//
list pathexecalckfm(vector pos, rotation rot, vector pprev, vector p0, vector p1)
{
#ifdef MARKERS
    if (gPathExeVerbose)
    {   placesegmentmarker(MARKERLINE, pprev, p0, TRANSGREEN, 0.20); }   // place a temporary line on the ground in-world.
#endif // MARKERS
    vector rp = p0 - pos;                       // p0 in relative coords - advances us to p0
    rp.z += gPathExeHeight * 0.5;               // add half-height, because path is at ground level
    //  Rotation is to the average direction of the previous and next sections in the XY plane.
    vector invec = p0-pprev;                    // incoming direction
    vector outvec = p1-p0;                      // outgoing direction
    float inveclen = llVecMag(invec);           // distance of this move
    vector invecnorm = llVecNorm(<invec.x, invec.y, 0>);
    vector outvecnorm = llVecNorm(<outvec.x,outvec.y,0>);
    if (p1 == ZERO_VECTOR) { outvecnorm = invecnorm; } // last section, no turn
    vector dir = llVecNorm(invecnorm+outvecnorm);// next direction
    rotation rr = llRotBetween(<1,0,0>, dir) / rot; // relative rotation
    rr = NormRot(rr);                           // why is this necessary?
    //  Time computation. Speed is limited by rotation rate.
    float angle = llFabs(llAngleBetween(ZERO_ROTATION, rr));    // how much rotation is this?
    float rsecs = angle / gMaxTurnRate;         // minimum time for this move per rotation limit
    float rt = inveclen / gMaxSpeed;           // minimum time for this move per speed limit
    if (rsecs > rt) { rt = rsecs; }             // choose longer time
    if (rt < 0.15) { rt = 0.15; }               // minimum time for KFM step
    DEBUGPRINT1("angle: " + (string)angle + " inveclen: " + (string)inveclen + " rt: " + (string)rt); // ***TEMP***
    return([rp, rr, rt]);                       // [rel pos, rel rot, rel time]
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
        if (nextseg == [ZERO_VECTOR])
        {   gPathExeEOF = TRUE;                                 // we are at EOF
            nextseg = [];                                       // but must deliver the EOF flag later
        }
        if (nextseg == []) 
        {   if (gAllSegments == [] && gPathExeEOF) { gAllSegments = [ZERO_VECTOR]; } // done, deliver the EOF signal
            DEBUGPRINT1("Assembly complete: " + llDumpList2String(gAllSegments,","));
            return;                                             // caught up with input
        }
        DEBUGPRINT1("Assembling segment #" + (string)gPathExeNextsegid + ": " + llDumpList2String(nextseg,","));
        if (nextseg == [ZERO_VECTOR] && gAllSegments != []) { return; } // if EOF marker, but segments left to do, use assembled segments
        gPathExeNextsegid++;                                    // advance seg ID
        if (gAllSegments == [])
        {   gAllSegments = pathexeextrapoints(nextseg, gPathExeDisttoend);  // first segment
            DEBUGPRINT1("Started gAllSegments from empty: " + llDumpList2String(gAllSegments,",")); // ***TEMP***
            nextseg = [];
        } else {
            DEBUGPRINT1("Adding to gAllSegments: " + llDumpList2String(gAllSegments,",")); // ***TEMP***
            vector lastpt = llList2Vector(gAllSegments,-1);
            vector firstpt = llList2Vector(nextseg,0);
            assert(llVecMag(lastpt-firstpt) < 0.01);    // endpoints should match
            nextseg = llList2List(nextseg,1,-1);        // discard new duplicate point
            //  If we can take a short-cut at the join between two segments, do so.
            //  Also add "extra points" on long segments here for speed control.
            if (obstaclecheckpath(llList2Vector(gAllSegments,-2), llList2Vector(nextseg,0), gPathExeWidth, gPathExeHeight, gPathExeProbespacing, gPathExeChartype))
            {   gAllSegments = llList2List(gAllSegments,0,-2) + pathexeextrapoints(nextseg, gPathExeDisttoend); }
            else
            {   gAllSegments += pathexeextrapoints(nextseg, gPathExeDisttoend);                // no, can't drop point
            }
        }
    }
}
//
//  pathexedomove -- feed in next section if any
//
pathexedomove()
{   if (gPathExeMoving) { return; }                     // we are moving, do nothing
    pathexeassemblesegs();                              // have work to do?
    if (gAllSegments == [ZERO_VECTOR])                  // if EOF signal
    {   if (gPathExeVerbose)
        {   llOwnerSay("Normal end of path. Free mem: " + (string)gPathExeFreemem); }
        pathexestop(0);                                 // all done, normal stop
    }
    else if (gAllSegments != [])                             // if work
    {   DEBUGPRINT1("Input to KFM: " + llDumpList2String(gAllSegments,","));   // what to take in
        list kfmmoves = pathexebuildkfm(llGetPos(), llGetRot(), gAllSegments);   // build list of commands to do
        DEBUGPRINT1("KFM: " + llDumpList2String(kfmmoves,","));  // dump the commands
        llSetKeyframedMotion(kfmmoves, [KFM_MODE, KFM_FORWARD]);             // begin motion
        gPathExeMoving = TRUE;                          // movement in progress
        integer freemem = llGetFreeMemory();            // how much memory left here, at the worst place       
        if (freemem < gPathExeFreemem) { gPathExeFreemem = freemem; }   // record free memory
        gAllSegments = [];                              // segments have been consumed
    } else {
        DEBUGPRINT1("Waiting for maze solver to catch up.");    // solver running behind action
    }
}

//
//  pathexemovementend -- movement has finished, feed in next section if any
//
pathexemovementend()
{   gPathExeMoving = FALSE;                                 // not moving
    gPathExeLastPos = ZERO_VECTOR;                          // no last moving pos
    DEBUGPRINT1("Movement end");
    pathexedomove();
}

//
//  pathexetimer  -- timer event, check progress
//
pathexetimer()
{
    if (gPathExeMoving)                                     // if we are supposed to be moving
    {   vector pos = llGetPos();
        if (llVecMag(pos - gPathExeLastPos) > 0.01)         // if moving at all
        {   return; }                                       // OK
        //  No KFM movement. Something has gone wrong
        pathexestop(MAZESTATUSKFMSTALL);                    // stalled
    }
}

//
//  pathexestop -- trouble, stop and abort keyframe motion
//
pathexestop(integer status)
{
    if (gPathExeMoving || (status != 0)) { DEBUGPRINT1("Forced movement stop. Status: " + (string)status); }
    llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);   // stop whatever is going on
    gClearSegments = [];                                    // reset state
    gMazeSegments = [];
    gAllSegments = [];
    gPathExeNextsegid = 0; 
    gPathExeMoving = FALSE;                                 // not moving
    gPathExeLastPos = ZERO_VECTOR;                          // no last position
    llSetTimerEvent(0.0);                                   // stop timing 
    if (gPathExeActive)                                     // if we are active
    {   if (status == 0) { status = gPathExePendingStatus; }// send back any stored status
        pathUpdateCallback(status,[]);                      // tell caller about result
        gPathExeActive = FALSE;                             // no longer active
    }
}
//
//  pathexemazedeliver  -- incoming maze result
//
pathexemazedeliver(string jsn) 
{
    DEBUGPRINT1("Maze deliver: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "mazesolve") { pathexestop(MAZESTATUSFORMAT); return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    if (status != 0) { pathexestop(status); return; }                  // error status from other side
    float cellsize = (float)llJsonGetValue(jsn,["cellsize"]); // get maze coords to world coords info
    vector pos = (vector)llJsonGetValue(jsn,["pos"]);
    rotation rot = (rotation)llJsonGetValue(jsn,["rot"]);
    list ptsmaze = llJson2List(llJsonGetValue(jsn, ["points"])); // points, one per word
    list ptsworld = [];
    integer i;
    integer length = llGetListLength(ptsmaze);              // number of points
    for (i=0; i<length; i++)
    {   integer val = llList2Integer(ptsmaze,i);            // X and Y encoded into one integer
        DEBUGPRINT1("Maze solve pt: (" + (string)mazepathx(val) + "," + (string)mazepathy(val) + ")");
        vector cellpos = mazecellto3d(mazepathx(val), mazepathy(val), cellsize, pos, rot);                        // convert back to 3D coords 
        ptsworld += [cellpos];                              // accum list of waypoints
    }
#ifdef MARKERS  
    if (gPathExeVerbose)
    {   integer i;
        for (i=0; i < llGetListLength(ptsworld)-1; i++)
        {   placesegmentmarker(MARKERLINE, llList2Vector(ptsworld,i), llList2Vector(ptsworld,i+1), TRANSYELLOW, 0.20); }   // place a temporary line on the ground in-world.
    }
#endif // MARKERS
    //  Straighten path. Maze solver paths are all right angles. Here we try short cuts.
    ptsworld = pathstraighten(ptsworld, gPathExeWidth, gPathExeHeight, gPathExeProbespacing, gPathExeChartype);   // postprocess path
    pathexedeliver(ptsworld, pathid, segmentid, TRUE, 0);      // deliver maze solution
}
//
//  pathexepathdeliver  -- JSON from path planner
//
//
pathexepathdeliver(string jsn) 
{   DEBUGPRINT1("Path deliver: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "path") { pathexestop(MAZESTATUSFORMAT); return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    ////if ((integer)serial != gMazeSerial) { return([MAZESTATUSCOMMSEQ]); }            // out of sequence 
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    if (status != 0) 
    {   if (gPathExeVerbose) { llOwnerSay("Path deliver with status " + (string)status); }
        if (gPathExePendingStatus == 0) { gPathExePendingStatus = status; }   // save any error status sent for later
    }
    list ptsstr = llJson2List(llJsonGetValue(jsn, ["points"])); // points, as strings
    list pts = [];
    integer i;
    integer len = llGetListLength(ptsstr);
    for (i=0; i<len; i++) { pts += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors  
    pathexedeliver(pts, pathid, segmentid, FALSE, status);      // deliver path segment
}

//
//  pathexecollision -- collided with something
//
pathexecollision(integer num_detected)
{   ////llOwnerSay("Collision");    // ***TEMP***
    if (!gPathExeMoving) { return; }    // not moving, not our fault
    integer i;
    for (i=0; i<num_detected; i++)
    {   key hitobj = llDetectedKey(i);
        if (hitobj != NULL_KEY)              // null key is land
        {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // hit a non-walkable
            {   if (gPathExeVerbose) { llOwnerSay("Hit " + llDetectedName(i)); }
                pathexestop(PATHEXECOLLISION);                      // stop
            }
        }
    }
}
#endif // PATHEXECUTE
