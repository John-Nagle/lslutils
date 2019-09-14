//
//  pathscan.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  Raycasts for obstacles while character is moving.
//
//  Handles stopping when there's a problem - obstacle or collision.
//  Also handles movement_end messages.
//
//  Also starts the actual keyframe movement operation, because thatg
//  involves generating a very large list, one pathexecute does not
//  have space for.
//
//  Communicates with pathexecute.lsl, which got too big, so
//  it had to be split.
//
//  Animats
//  August, 2019
//
//  License: GPLv3
//
///
#include "npc/assert.lsl"                                   // assert
#include "npc/patherrors.lsl"
#include "npc/mazedefs.lsl"
#include "npc/pathbuildutils.lsl"
#include "npc/pathscancall.lsl"
//
//  Constants
//
//  Look ahead constants
//
#define PATHSCANRAYTIME      0.2                            // (secs) do a cast ray for obstacles this often
#define PATHMOVECHECKSECS   2.0                             // (secs) check this often for progress
#define PATHEXELOOKAHEADDIST    10.0                        // (m) distance to look ahead for obstacles while moving
#define PATHSCANMINTARGETMOVE   4.0                         // (m) target must move this much to be re-chased
#define PATHSCANMINTARGETFRACT  0.5                         // (fraction) target must move this much as fract of dist to go to be re-chased.

//
//  Globals
//
integer gPathScanId = 0;                                    // current path ID
integer gPathScanActive = FALSE;                            // scan system active
integer gPathScanMoving = FALSE;                            // character should be moving
integer gPathScanTimetick = 0;                              // last time we tested for motion
vector gPathScanLastpos = ZERO_VECTOR;                      // last place we tested for motion
integer gPathScanFreemem = 9999999;                         // smallest free memory seen

//  Avatar params
float gPathScanWidth = 1.0;                                  // defaults, overridden by messages
float gPathScanHeight = 1.0;
float gPathScanMaxTurnspeed = 0.2;                           // (radians/sec) max turn rate - overridden
float gPathScanMaxSpeed = 2.0;                               // (meters/sec) max speed

integer gPathScanChartype = CHARACTER_TYPE_A;
key gPathScanTarget = NULL_KEY;                             // who we are chasing, if anybody
vector gPathScanTargetPos = ZERO_VECTOR;                    // last loc of target

//
//  Segment storage
//  Just a list of vectors
//
list gKfmSegments = [];                                     // segments being executed by current KFM operation
integer gKfmSegmentCurrent = 0;                             // which segment we are currently on

//
//  pathscaninit -- set up path execute parameters
//
pathscaninit()
{   gPathSelfObject = llGetKey();                           // us
    gPathScanFreemem = llGetFreeMemory();   
}

//
//  pathscandone - done here, report to execute task
//
//  Zero status here means ordinary movement end, no problems.
//
pathscandone(integer status, key hitobj)
{   if (!gPathScanActive) { return; }                       // we are not running, ignore
    if (gPathScanMoving && status != 0)                     // if something bad happened
    {   ////llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);// stop whatever is going on
        pathMsg(PATH_MSG_WARN, "Stopped by obstacle " + llKey2Name(hitobj) + " status: " + (string)status);    
    }
    //  Return "scandone" to exec module
    list params = ["reply", "scandone", "status", status, "pathid", gPathScanId, "hitobj", hitobj];
    llMessageLinked(LINK_THIS, LINKMSGSCANREPLY, llList2Json(JSON_OBJECT,params), "");   // Return result to execute task
    gPathScanMoving = FALSE;                                    // no longer active
}
//
//  pathscanmovementend -- movement has finished, feed in next section if any
//
pathscanmovementend()
{   gPathScanMoving = FALSE;                                    // not moving
    gKfmSegments = [];                                          // no current segments
    gPathScanLastpos = ZERO_VECTOR;                             // no last moving pos   
    pathMsg(PATH_MSG_INFO,"Movement end");
    pathscandone(0, "");                                        // normal event
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
    {   pathscandone(llList2Integer(castanalysis,0), NULL_KEY); return; }  // report error
    key hitobj = llList2Key(castanalysis,0);                // result is [obj, hitpt]
    vector hitpt = llList2Vector(castanalysis,1);
    pathMsg(PATH_MSG_WARN,"Move stopped by obstacle: " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) 
                    + " at " + (string)(hitpt) + " by ray cast from " + (string)p + " to " + (string)p1);
    pathscandone(PATHEXEOBSTRUCTED, hitobj);  // report trouble
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
    vector posoffset = <0,0,gPathScanHeight*0.5>;    // pos is at object midpoint, points are at ground
    vector pos = llGetPos() - posoffset;            // where we are now
    integer i;
    integer foundseg = FALSE;
    vector startpos = llList2Vector(gKfmSegments,0);   // start of path
    //  Start at segment where last found the position.  
    //  Stop at end of list, finished lookaheaddist, or no longer moving.
    //  pos and all segment points are at ground level.
    for (i=gKfmSegmentCurrent; i<llGetListLength(gKfmSegments)-1 && lookaheaddist > 0 && gPathScanActive; i++)
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
    {   pathMsg(PATH_MSG_WARN,"Unable to find " + (string)pos + " in " + llDumpList2String(gKfmSegments,",")); } // off the path?
}
//
//  pathchecktargetmoved -- check if pursuit target moved.
//
//  This makes the character chase the target. 
//  Must not replan too often; replanning is slow.
//
pathchecktargetmoved()
{   if (gPathScanTarget == NULL_KEY) { return; }                // not in pursuit, no check
    list details = llGetObjectDetails(gPathScanTarget, [OBJECT_POS]);   // get object position
    if (details == [])
    {   pathMsg(PATH_MSG_WARN, "Pursue target left sim."); 
        pathscandone(PATHEXETARGETGONE, gPathScanTarget);       // target is gone, abort pursue
        return;
    }
    //  If pursue target moved more than half the distance to the goal, but at least 2m, replan.
    vector pos = llGetPos();                                    // where we are
    vector targetpos = llList2Vector(details,0);                // where target is
    float disttotarget = llVecMag(targetpos - pos);
    float distmoved = llVecMag(gPathScanTargetPos - targetpos); // distance target moved since replan
    if (distmoved < PATHSCANMINTARGETMOVE) { return; }          // has not moved enough to replan
    if (distmoved < disttotarget * PATHSCANMINTARGETFRACT) { return; } // has not moved enough to replan
    pathscandone(PATHEXETARGETMOVED, gPathScanTarget);          // target moved, must replan and chase
}
//  
//
//  pathscantimer  -- timer event, check progress and do ray casts
//
pathscantimer()
{   if (gPathScanActive && gKfmSegments != [])                  // if we are moving and have a path
    {   pathcheckdynobstacles(); }                              // ray cast for obstacles
    if (gPathScanActive && gKfmSegments != [])                  // if we are moving and have a path
    {   pathchecktargetmoved(); }                               // check if pursuit target moved
    if (gPathScanActive)                                        // if we are turned on
    {   
        integer now = llGetUnixTime();                          // time now
        if (now - gPathScanTimetick > PATHMOVECHECKSECS)
        {   gPathScanTimetick = now;                            // update last check time
            vector pos = llGetPos();
            if (llVecMag(pos - gPathScanLastpos) > 0.01)        // if moving at all
            {   return; }                                       // OK
            //  No KFM movement. Something has gone wrong. 
            pathscandone(MAZESTATUSKFMSTALL, NULL_KEY);         // stalled
        }
    }
}

//
//  pathscanrequest  -- JSON from path execution
//
//  We get a list of the points the character is currently following. 
//  That tells us what direction to look for obstacles.
//
//  Protocol is to send a "startscan" as each KFM segment starts.
//  Send a "stopscan" when the entire path is finished or when not moving.
//  This will result in a "stall" event being sent if something goes wrong.
//
pathscanrequest(string jsn) 
{   pathMsg(PATH_MSG_INFO,"Path scan request: " + jsn);
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type  
    if (requesttype == "startscan")                         // start scanning
    {   //  Set up for ray casting.
        gPathScanId = (integer)llJsonGetValue(jsn, ["pathid"]);
        gPathScanTarget = (key)llJsonGetValue(jsn, ["target"]); // who we are chasing, if anybody
        gPathScanWidth = (float)llJsonGetValue(jsn,["width"]);
        gPathScanHeight = (float)llJsonGetValue(jsn,["height"]);
        gPathMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);
        gPathScanMaxSpeed = (float)llJsonGetValue(jsn,["speed"]); 
        gPathScanMaxTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]); 

        list ptsstr = llJson2List(llJsonGetValue(jsn, ["points"])); // points, as strings
        gKfmSegments = [];                                  // clear stored path used for ray cast direction
        gKfmSegmentCurrent = 0;
        integer i;
        integer len = llGetListLength(ptsstr);
        assert(len >= 2);                                   // required to have a start point and a dest at least
        for (i=0; i<len; i++) { gKfmSegments += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors
        //  Get position of pursuit target if tracking
        if (gPathScanTarget != NULL_KEY) 
        {   list details = llGetObjectDetails(gPathScanTarget, [OBJECT_POS]);   // get object position
            if (details == [])                              // avatar not found
            {   pathMsg(PATH_MSG_WARN, "Pursue target not found."); gPathScanTargetPos = ZERO_VECTOR; }   // gone from sim, timer will detect
            else
            {   gPathScanTargetPos = llList2Vector(details,0);   }   // where target is
        }
        pathexedokfm();                                     // actually do the avatar movement      
        gPathScanActive = TRUE;                             // scan system is active
        gPathScanMoving = TRUE;                             // character is moving
        gPathScanTimetick = llGetUnixTime();                // reset stall timer
        llSetTimerEvent(PATHSCANRAYTIME);                   // switch to fast timer for ray casts for obstructions
    } else if (requesttype == "stopscan")                   // stop scanning
    {   gKfmSegments = [];
        llSetTimerEvent(0.0);                               // shut down and stop timer
        gPathScanActive = FALSE;                            // scan system is active
        gPathScanMoving = FALSE;                            // character is moving
    } else {
        pathMsg(PATH_MSG_ERROR,"Path stop rcvd bad msg: " + jsn);
    }
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
    rp.z += gPathScanHeight * 0.5;              // add half-height, because path is at ground level
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
    float rsecs = angle / gPathScanMaxTurnspeed; // minimum time for this move per rotation limit
    float rt = inveclen / gPathScanMaxSpeed;     // minimum time for this move per speed limit
    if (rsecs > rt) { rt = rsecs; }             // choose longer time
    if (rt < 0.15) { rt = 0.15; }               // minimum time for KFM step
    DEBUGPRINT1("angle: " + (string)angle + " inveclen: " + (string)inveclen + " rt: " + (string)rt); // ***TEMP***
    return([rp, rr, rt]);                       // [rel pos, rel rot, rel time]
} 

//
//  pathexedokfm -- start the keyframe movement operation
//
//  Finally, the real movement gets done.
//
pathexedokfm()
{   
    vector kfmstart = llList2Vector(gKfmSegments,0);    // first point, which is where we should be
    assert(kfmstart != ZERO_VECTOR);                    // must not be EOF marker  
    vector pos = llGetPos();                            // we are here
#ifdef OBSOLETE       
    if (pathvecmagxy(kfmstart - pos) > PATHEXEMAXCREEP)     // we are out of position
    {   pathMsg(PATH_MSG_WARN, "KFM out of position. At " + (string)pos + ". Should be at " + (string)kfmstart); // not serious, but happens occasionally
            ////pathexestop(PATHEXEBADMOVEEND);                 // error, must start a new operation to recover
            ////return; 
    }
    ////gKfmSegments = llListReplaceList(gKfmSegments,[pos-<0,0,gPathExeHeight*0.5>],0,0);   // always start from current position
#endif // OBSOLETE
    pathMsg(PATH_MSG_DEBUG,"Input to KFM: " + llDumpList2String(gKfmSegments,","));     // what to take in
    list kfmmoves = pathexebuildkfm(pos, llGetRot(), gKfmSegments);   // build list of commands to do
    if (kfmmoves != [])                             // if something to do (if only one point stored, nothing happens)
    {   llSetKeyframedMotion(kfmmoves, [KFM_MODE, KFM_FORWARD]);            // begin motion  
        integer freemem = llGetFreeMemory();            // how much memory left here, at the worst place       
        if (freemem < gPathScanFreemem) 
        {   gPathScanFreemem = freemem; 
            pathMsg(PATH_MSG_WARN, "Scan task free memory: " + (string)freemem);
        }   // record free memory
    }
}

//
//  pathscancollision -- collided with something
//
pathscancollision(integer num_detected)
{   
    if (!gPathScanMoving) { return; }    // not moving, not our fault
    integer i;
    for (i=0; i<num_detected; i++)
    {   key hitobj = llDetectedKey(i);
        if (hitobj != NULL_KEY)              // null key is land
        {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // hit a non-walkable
            {   pathMsg(PATH_MSG_WARN,"Collided with " + llDetectedName(i));
                pathscandone(PATHEXECOLLISION, llDetectedKey(i)); // stop
                return;
            }
        }
    }
}

//
//  The main program of the scan task.
//
default
{
    state_entry()
    {  pathscaninit();                                // init our KFM system        
    }

    on_rez(integer rezparam) 
    {   llResetScript(); }

    link_message(integer status, integer num, string jsn, key id )
    {   if (num == LINKMSGSCANREQUEST)                           // maze solve result
        {   pathscanrequest(jsn); }
    }
    
    timer()
    {   pathscantimer();   }                                     // pass timer event
    
    moving_end()
    {   pathscanmovementend(); }   
    
    collision_start(integer num_detected)
    {   pathscancollision(num_detected); }
}
