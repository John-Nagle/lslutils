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
//  2. Report completion via link msg.                          [DONE]
//  3. Proper status handling.                                  [DONE]
//  4. Smooth slow to stop at end.                              [DONE]
//  5. Prevent rotation and position error accum between segs.  [DONE]
//  6. Add ray casting to detect obstacles when moving.         [DONE]
//  7. Add timer handing.                                       [DONE]
//
#ifndef PATHEXECUTELSL                                        // include guard, like C/C++
#define PATHEXECUTELSL
#include "npc/assert.lsl"                                   // assert
#include "npc/patherrors.lsl"
#include "npc/mazedefs.lsl"
#include "npc/pathbuildutils.lsl"
//
//  Constants
//

//
//  Idle timer timeout
//
#define PATHEXETIMEOUT      2.0                             // check this often for progress
#define PATHEXERAYTIME      0.2                             // do a cast ray for obstacles this often
#define PATHMINTURNSECTION  0.5                             // first section of path this length, for a starting turn
#define PATHMOVECHECKSECS   2.0                             // check this often for progress
#define PATHEXELOOKAHEADDIST    10.0                        // (m) distance to look ahead for obstacles while moving
#define PATHEXEMAXCREEP     0.10                            // (m) max positional error allowed after keyframe motion

//
//  Globals
//                                
integer gPathExeId = 0;                                     // current path ID
integer gPathExeNextsegid;                                  // current segment
integer gPathExeMoving = FALSE;                             // true if KFM movement in progress
integer gPathExeActive = FALSE;                             // path system is doing something
integer gPathExeEOF;                                        // EOF seen
integer gPathExePendingStatus;                              // pending error, to be reported at end of motion
vector  gPathExeLastPos;                                    // last position, for stall check
integer gPathExeFreemem;                                    // amount of free memory left
integer gPathLastTimetick = 0;                              // last time we tested for motion

//  Avatar params
float gPathExeWidth = 1.0;                                  // defaults, overridden by messages
float gPathExeHeight = 1.0;
integer gPathExeChartype = CHARACTER_TYPE_A;
float gPathExeMaxTurnspeed = 0.2;                           // (radians/sec) max turn rate - overridden
float gPathExeMaxSpeed = 2.0;                               // (meters/sec) max speed

float gPathExeDisttoend = 1.5;                              // (m) distance to begin turn ahead of corner
float gPathExeProbespacing = 0.333;                         // (m) cast ray spacing when looking for obstacles

//
//  Segment storage
//  Each segment list is of the form [segmentid, pntcount, pnt, pnt ... segmentid, pntcount, pnt, pnt ...]
//
list gClearSegments = [];                                   // path segment list
list gMazeSegments = [];                                    // maze segment list

list gAllSegments = [];                                     // combined segments from above, points only
list gKfmSegments = [];                                     // segments being executed by current KFM operation
integer gKfmSegmentCurrent = 0;                             // which segment we are currently on


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
    vector scale = <length,gPathExeWidth,thickness>;    // size of marker
    list params = [ "pos", midpoint, "rot", rotperpenonground(p0,p1), "scale", scale, "color", color, "alpha", alpha];
    llMessageLinked(LINK_THIS, LINKMSGMARKER, llList2Json(JSON_OBJECT,params), markername);   // ask marker service to place a marker.   
}
#endif // MARKERS

//
//  pathexeinit -- set up path execute parameters
//
pathexeinit(float probespacing)
{   gPathSelfObject = llGetKey();                           // us
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
list pathexeextrapoints(list pts, float distfromends, integer first)
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
list pathexecalckfm(vector pos, rotation rot, vector pprev, vector p0, vector p1)
{
#ifdef MARKERS
    if (gPathMsgLevel >= PATH_MSG_INFO)
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
    float rsecs = angle / gPathExeMaxTurnspeed; // minimum time for this move per rotation limit
    float rt = inveclen / gPathExeMaxSpeed;     // minimum time for this move per speed limit
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
            assert(llVecMag(lastpt-firstpt) < 0.01);    // endpoints should match
            nextseg = llList2List(nextseg,1,-1);        // discard new duplicate point
            //  If we can take a short-cut at the join between two segments, do so.
            //  Also add "extra points" on long segments here for speed control.
            if (obstaclecheckpath(llList2Vector(gAllSegments,-2), llList2Vector(nextseg,0), gPathExeWidth, gPathExeHeight, gPathExeProbespacing, gPathExeChartype))
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
    //// if (gAllSegments == [ZERO_VECTOR])                  // if EOF signal
    if (llGetListLength(gAllSegments) == 1 && llList2Vector(gAllSegments,0) == ZERO_VECTOR) // if EOF signal
    {   pathMsg(PATH_MSG_WARN,"Normal end of path. Free mem: " + (string)gPathExeFreemem); 
        pathexestop(0);                                 // all done, normal stop
    }
    else if (gAllSegments != [])                             // if work
    {   
        vector kfmstart = llList2Vector(gAllSegments,0);    // first point, which is where we should be
        vector pos = llGetPos();                            // we are here
        if (llVecMag(<kfmstart.x, kfmstart.y, 0> - <pos.x, pos.y, 0>) > PATHEXEMAXCREEP)     // we are out of position
        {   pathMsg(PATH_MSG_ERROR, "Out of position. At " + (string)pos + ". Should be at " + (string)kfmstart); // probably user doing an edit
        }
        gAllSegments = llListReplaceList(gAllSegments,[pos-<0,0,gPathExeHeight*0.5>],0,0);   // always start from current position
        pathMsg(PATH_MSG_INFO,"Input to KFM: " + llDumpList2String(gAllSegments,","));   // what to take in
        list kfmmoves = pathexebuildkfm(pos, llGetRot(), gAllSegments);   // build list of commands to do
        DEBUGPRINT1("KFM: " + llDumpList2String(kfmmoves,","));  // dump the commands
        if (kfmmoves != [])                             // if something to do (if only one point stored, nothing happens)
        {   llSetKeyframedMotion(kfmmoves, [KFM_MODE, KFM_FORWARD]);             // begin motion
            gPathExeMoving = TRUE;                          // movement in progress
            llSetTimerEvent(PATHEXERAYTIME);                // switch to fast timer for ray casts for obstructions
            gKfmSegments = gAllSegments;                    // what we are currently doing
            gKfmSegmentCurrent = 0;                         // we are at the beginning
            integer freemem = llGetFreeMemory();            // how much memory left here, at the worst place       
            if (freemem < gPathExeFreemem) { gPathExeFreemem = freemem; }   // record free memory
        }
        gAllSegments = [];                              // segments have been consumed
    } else {
        pathMsg(PATH_MSG_WARN,"Waiting for maze solver to catch up.");    // solver running behind action
    }
}

//
//  pathexemovementend -- movement has finished, feed in next section if any
//
pathexemovementend()
{   gPathExeMoving = FALSE;                                 // not moving
    gKfmSegments = [];                                      // no current segments
    gPathExeLastPos = ZERO_VECTOR;                          // no last moving pos
    
    pathMsg(PATH_MSG_INFO,"Movement end");
    pathexedomove();
}

//
//  pathobstacleraycast -- check for obstacle ahead
//
//  Cast ray from p to p1.
//
pathobstacleraycast(vector p, vector p1)
{   
    //  One simple ahead ray cast for now.
    list castresult = castray(p, p1, PATHCASTRAYOPTSOBS);
    //  We have to do the whole analysis drill. Ground or walkable, OK. Self, OK.
    //  Anything else is an obstacle
    list castanalysis = pathanalyzecastresult(castresult, FALSE);
    if (castanalysis == []) return;                         // no problem
    if (llGetListLength(castanalysis) == 1)                 // error status
    {   pathexestop(llList2Integer(castanalysis,0)); return; }  // report error
    key hitobj = llList2Key(castanalysis,0);                // result is [obj, hitpt]
    vector hitpt = llList2Vector(castanalysis,1);
    pathMsg(PATH_MSG_WARN,"Stopped by obstacle while moving: " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) 
                    + " at " + (string)(hitpt) + " by ray cast from " + (string)p + " to " + (string)p1);
    pathexestopkey(PATHEXEOBSTRUCTED, hitobj);  // report trouble
}
//
//  pathcheckdynobstacles  -- check for dynamic obstacles encountered while moving.
//
//  Such as other KFM characters, which are not collidable.
//
pathcheckdynobstacles()
{
    //  We need to find out which segment of the path we are currently in.
    float lookaheaddist = PATHEXELOOKAHEADDIST;     // distance to look ahead
    float PATHEXESEGDISTTOL = 0.20;                 // how close to segment to be in it. Keyframe error causes trouble here.
    float HUGE = 99999999999.0;                     // huge number, but INFINITY is bigger
    vector posoffset = <0,0,gPathExeHeight*0.5>;    // pos is at object midpoint, points are at ground
    vector pos = llGetPos() - posoffset;            // where we are now
    integer i;
    integer foundseg = FALSE;
    vector startpos = llList2Vector(gKfmSegments,0);   // start of path
    //  Start at segment where last found the position.  
    //  Stop at end of list, finished lookaheaddist, or no longer moving.
    //  pos and all segment points are at ground level.
    for (i=gKfmSegmentCurrent; i<llGetListLength(gKfmSegments)-1 && lookaheaddist > 0 && gPathExeMoving != 0; i++)
    {   vector p0 = llList2Vector(gKfmSegments,i);
        vector p1 = llList2Vector(gKfmSegments,i+1);
        if (!foundseg) 
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHEXESEGDISTTOL); // distance along seg, or INFINITY
            if (distalongseg < HUGE)    
            {   
                gKfmSegmentCurrent = i;                 // advance current segment pos
                foundseg = TRUE;                        // start checking from here                
            }
        }
        if (foundseg)                                   // if pos is in this or a previous segment
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHEXESEGDISTTOL); // distance along seg, or NAN
            if (distalongseg < HUGE)                    // point is in capsule around segment
            {   
                float seglength = llVecMag(p0-p1);
                if (distalongseg < 0) { distalongseg = 0; } // bound to segment
                if (distalongseg > seglength) { distalongseg = seglength; }
                float castdist = seglength - distalongseg;  // cast to end of segment
                if (lookaheaddist < castdist) { castdist = lookaheaddist; } // if running out of cast distance
                if  (castdist <= 0) { return; };        // at end
                vector pos2 = pos + llVecNorm(p1-p0)*castdist; // how far to cast            
                pathobstacleraycast(pos+posoffset, pos2+posoffset);          // look ahead horizontally       
                lookaheaddist -= castdist;              // reduce distance ahead
                pos = p1;                               // start of next segment is start of next cast
            }
        }   
    }
    if (!foundseg)
    {   pathMsg(PATH_MSG_ERROR,"Unable to find segment containing current position: " + (string)pos + " in " + llDumpList2String(gKfmSegments,",")); } // off the path?
}
//  
//
//  pathexetimer  -- timer event, check progress and do ray casts
//
pathexetimer()
{   if (gPathExeMoving && gKfmSegments != [])                   // if we are moving and have a path
    {   pathcheckdynobstacles(); }                              // ray cast for obstacles
    if (gPathExeMoving)                                         // if we are supposed to be moving
    {   
        integer now = llGetUnixTime();                          // time now
        if (now - gPathLastTimetick > PATHMOVECHECKSECS)
        {   gPathLastTimetick = now;                            // update last check time
            vector pos = llGetPos();
            if (llVecMag(pos - gPathExeLastPos) > 0.01)         // if moving at all
            {   return; }                                       // OK
            //  No KFM movement. Something has gone wrong
            pathexestop(MAZESTATUSKFMSTALL);                    // stalled
        }
    }
}

//
//  pathexestopkey -- trouble, stop and abort keyframe motion, with key
//
pathexestopkey(integer status, key hitobj)
{
    if (gPathExeMoving || (status != 0)) { pathMsg(PATH_MSG_WARN,"Movement stop. Status: " + (string)status); }
    llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);   // stop whatever is going on
    gClearSegments = [];                                    // reset state
    gMazeSegments = [];
    gAllSegments = [];
    gKfmSegments = [];
    gPathExeNextsegid = 0; 
    gPathExeMoving = FALSE;                                 // not moving
    gPathExeLastPos = ZERO_VECTOR;                          // no last position
    llSetTimerEvent(0.0);                                   // stop timing 
    if (gPathExeActive)                                     // if we are active
    {   if (status == 0) { status = gPathExePendingStatus; }// send back any stored status
        pathUpdateCallback(status,hitobj);                  // tell caller about result
        gPathExeActive = FALSE;                             // no longer active
    }
}
//
//  pathexestop -- trouble, stop and abort keyframe motion, short form
//
pathexestop(integer status)
{   pathexestopkey(status, NULL_KEY); }

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
    //  If a move is stopped, the maze solver may still be running and sending maze solves.
    //  Discard such stale maze solves.
    if (pathid != gPathExeId && segmentid != 0)                     // if stale result
    {   pathMsg(PATH_MSG_WARN,"Discarding stale maze solve result.");
        return;
    }
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
    if (gPathMsgLevel >= PATH_MSG_INFO)
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
pathexepathdeliver(string jsn) 
{   pathMsg(PATH_MSG_INFO,"Path deliver received: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "path") { pathexestop(MAZESTATUSFORMAT); return; }              // ignore, not our msg
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    //  Results from the path planner also contain some misc, parameter updates.
    gPathExeMaxSpeed = (float)llJsonGetValue(jsn,["speed"]); 
    gPathExeMaxTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]); 
    gPathExeWidth = (float)llJsonGetValue(jsn,["width"]);
    gPathExeHeight = (float)llJsonGetValue(jsn,["height"]);
    gPathMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    if (status != 0) 
    {   pathMsg(PATH_MSG_WARN,"Path deliver with status " + (string)status); 
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
{   
    if (!gPathExeMoving) { return; }    // not moving, not our fault
    integer i;
    for (i=0; i<num_detected; i++)
    {   key hitobj = llDetectedKey(i);
        if (hitobj != NULL_KEY)              // null key is land
        {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // hit a non-walkable
            {   pathMsg(PATH_MSG_INFO,"Collided with " + llDetectedName(i));
                pathexestopkey(PATHEXECOLLISION, llDetectedKey(i)); // stop
                return;
            }
        }
    }
}
#endif // PATHEXECUTELSL
