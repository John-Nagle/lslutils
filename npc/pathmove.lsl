//
//  pathmove.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  Starts the actual keyframe movement operation, because that
//  involves generating a very large list, one pathexecute does not
//  have space for.
//
//  Raycasts for obstacles while character is moving.
//
//  Handles stopping when there's a problem - obstacle or collision.
//  Also handles movement_end messages.
//
//  Communicates with pathassemble.lsl, which got too big, so
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
#include "npc/pathmazedefs.lsl"
#include "npc/pathbuildutils.lsl"
#include "npc/pathmovecall.lsl"
//
//  Constants
//
//  Look ahead constants
//
#define PATHMOVERAYTIME      0.2                            // (secs) do a cast ray for obstacles this often
#define PATHMOVECHECKSECS   2.0                             // (secs) check this often for progress
#define PATHERRLOOKAHEADDIST    10.0                        // (m) distance to look ahead for obstacles while moving
#define PATHMOVEMINTARGETMOVE   4.0                         // (m) target must move this much to be re-chased
#define PATHMOVEMINTARGETFRACT  0.5                         // (fraction) target must move this much as fract of dist to go to be re-chased.
#define PATHMAXSAVEDGOODPOS 20                              // (count) number of previous good positions to save
#define PATHERRMAXCREEP     0.10                            // (m) max positional error allowed after keyframe motion
#define PATHERRMINGOODPOS   1.00                            // (m) minimum distance between good points


//
//  Globals
//
integer gPathMoveId = 0;                                    // current path ID
integer gPathMoveActive = FALSE;                            // move system active
integer gPathMoveMoving = FALSE;                            // character should be moving
integer gPathMoveRecovering = FALSE;                        // recovery in progress, do not move
integer gPathMoveTimetick = 0;                              // last time we tested for motion
vector gPathMoveLastpos = ZERO_VECTOR;                      // last place we tested for motion
vector gPathMoveLastdest = ZERO_VECTOR;                     // last destination of KFM string
integer gPathMoveFreemem = 9999999;                         // smallest free memory seen
vector gPathInitialRegionCorner;                            // corner of initial region, used as reference point
vector gPathStartRegionCorner;                              // region corner where path started
list gPathMoveLastgoodpos = [];                             // last good position, relative to initial region corner


//  Avatar params
float gPathMoveMaxTurnspeed = 0.2;                          // (radians/sec) max turn rate - overridden
float gPathMoveMaxSpeed = 2.0;                              // (meters/sec) max speed

key gPathMoveTarget = NULL_KEY;                             // who we are chasing, if anybody
vector gPathMoveTargetPos = ZERO_VECTOR;                    // last loc of target

//
//  Segment storage
//  Just a list of vectors
//
list gKfmSegments = [];                                     // segments being executed by current KFM operation
integer gKfmSegmentCurrent = 0;                             // which segment we are currently on

//
//  pathmoveinit -- set up path execute parameters
//
pathmoveinit()
{   gPathSelfObject = pathGetRoot(llGetKey());              // us
    gPathMoveFreemem = llGetFreeMemory();
    gPathInitialRegionCorner = llGetRegionCorner();             // ref point for recovery points
}

//
//  pathmovedone - done here, report to execute task
//
//  Zero status here means ordinary movement end, no problems.
//
pathmovedone(integer status, key hitobj)
{   if (gPathMoveRecovering)                                        // done during recovery?
    {
        //  There's a move system and a recovery system, and they're 
        //  not supposed to be active at the same time. But it's quite
        //  possible for a new move request to happen during recovery.
        //  When that happens, we have to reject the new move request.
        //  The new request gets all the way through the planner and
        //  maze solver, which is asynchronous with actual recovery.
        //  Then it gets caught in pathmoverequestrcvd, which calls pathmovedone
        //  with a status of PATHERRREQOUTOFSYNC. This passes the error status
        //  back up the pipeline, killing off the move request.
        // 
        //  That happens when recovery is in progress and a new move,
        //  typically from a higher priority behavior, preempts what's
        //  going on. The recovery has to complete, and the new move
        //  has to be rejected. That's what this is for. 
        // 
        pathMsg(PATH_MSG_WARN,"Path done during recovery, status " + (string)status);
        assert(status != 0);                                        // must have error status
        assert(!gPathMoveMoving);                                   // must not be both recovering and moving
        assert(!gPathMoveActive);
    } else {                                                        // normal case
        if (!gPathMoveActive) { return; }                           // we are not running, ignore
        if (gPathMoveMoving && (status != 0))                       // if something bad happened
        {   llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);   // stop whatever is going on. This is the only KFM_CMD_STOP.
            pathMsg(PATH_MSG_WARN, "Stopped by obstacle " + llKey2Name(hitobj) + " status: " + (string)status);
            integer newstatus = pathcheckforwalkable();             // recover position if necessary
            if (newstatus != 0) { status = newstatus; }             // use walkable recovery status if walkable problem   
        } else {                                                    // KFM finished normally
            if (gPathStartRegionCorner != llGetRegionCorner())      // if changed region
            {   status = PATHERRREGIONCROSS;                        // region cross status, force error and retry
            } else {                                                // normal move completion
                //  We should be at the KFM destination now.
                vector pos = llGetPos() - <0,0,gPathHeight * 0.5>;  // pos is at midpoint, points are at ground level
                if (gPathMoveLastdest != ZERO_VECTOR && llVecMag(pos-gPathMoveLastdest) > 0.10)         // if not at desired point, allow 10cm error
                {   pathMsg(PATH_MSG_WARN, "KFM did not reach destination. At " + (string)pos + ", should be at " + (string)gPathMoveLastdest);
                //  May need to take corrective action here. For now, just log.
                }
                //  Final check - are we some place we should't be. Fix it now, rather than getting stuck.
                status = pathcheckforwalkable();                        // if at non-walkable destination and can't recover
            }
        }
    }
    //  Return "movedone" to exec module
    list params = ["reply", "movedone", "status", status, "pathid", gPathMoveId, "hitobj", hitobj];
    llMessageLinked(LINK_THIS, LINKMSGMOVEREPLY, llList2Json(JSON_OBJECT,params), "");   // Return result to execute task
    gPathMoveMoving = FALSE;                                    // no longer active
    gPathMoveActive = FALSE;
    gPathMoveLastdest = ZERO_VECTOR;                            // used up destination
}
//
//  pathmovemovementend -- movement has finished, feed in next section if any
//
pathmovemovementend()
{   if (!gPathMoveMoving) { return; }                           // not moving, not our fault
    gPathMoveMoving = FALSE;                                    // not moving
    gKfmSegments = [];                                          // no current segments
    gPathMoveLastpos = ZERO_VECTOR;                             // no last moving pos 
#ifdef OBSOLETE  
    integer status = 0;                                         // normal completion so far
    if (gPathStartRegionCorner != llGetRegionCorner())          // if changed region
    {   status = PATHERRREGIONCROSS; }                          // region cross status
    pathMsg(PATH_MSG_INFO,"Movement end, status: " + (string)status);
    pathmovedone(status, "");                                   // normal event unless changed region
#endif // OBSOLETE
    pathmovedone(0, "");                                        // normal end
}
//
//  pathcheckforwalkable  -- is there a walkable below here?
//
//  Returns 0 or error status
//
//
integer pathcheckforwalkable()
{   vector pos = llGetPos();                                // we are here
    vector fullheight = <0,0,gPathHeight>;              // add this for casts from middle of character
    vector halfheight = fullheight*0.5;
    vector p = pos-halfheight;                              // position on ground
    vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
    if (obstacleraycastvert(p+fullheight,p-mazedepthmargin) >= 0)  
    {   return(0); }                                        // no problem   
    //  Trouble, there is no walkable here
    pathMsg(PATH_MSG_WARN,"No walkable below after move to " + (string)p);
    return(PATHERRWALKABLETROUBLE);                         // fail for now, may recover later
}


//
//  pathobstacleraycast -- check for obstacle ahead
//
//  Cast ray from p to p1.
//
integer pathobstacleraycast(vector p, vector p1)
{   
    //  One simple ahead ray cast for now.
    list castresult = castray(p, p1, PATHCASTRAYOPTS);          // don't need normal
    //  We have to do the whole analysis drill. Ground or walkable, OK. Self, OK.
    //  Anything else is an obstacle
    list castanalysis = pathanalyzecastresult(castresult, FALSE);
    if (castanalysis != [])                                     // if problem
    {
        if (llGetListLength(castanalysis) == 1)                 // error status
        {   pathmovedone(llList2Integer(castanalysis,0), NULL_KEY); return(TRUE); }  // report error
        key hitobj = llList2Key(castanalysis,0);                // result is [obj, hitpt]
        vector hitpt = llList2Vector(castanalysis,1);
        pathMsg(PATH_MSG_WARN,"Move stopped by obstacle: " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) 
                    + " at " + (string)(hitpt) + " by ray cast from " + (string)p + " to " + (string)p1);
        pathmovedone(PATHERROBSTRUCTED, hitobj);  // report trouble
        return(TRUE);                              // trouble
    }
    return(FALSE);                                   // no trouble
}
//
//  pathcheckdynobstacles  -- check for dynamic obstacles encountered while moving.
//
//  Such as other KFM characters, which are not collidable.
//
pathcheckdynobstacles()
{
    //  We need to find out which segment of the path we are currently in.
    float lookaheaddist = PATHERRLOOKAHEADDIST;     // distance to look ahead
    float PATHERRSEGDISTTOL = 0.20;                 // how close to segment to be in it. Keyframe error causes trouble here.
    float HUGE = 99999999999.0;                     // huge number, but INFINITY is bigger
    vector halfheight = <0,0,gPathHeight*0.5>;      // pos is at object midpoint, points are at ground
    vector fullheight = <0,0,gPathHeight>;          // pos is at object midpoint, points are at ground
    vector pos = llGetPos() - halfheight;           // where we are now
    vector groundpos = pos;                         // pos on ground
    integer i;
    integer foundseg = FALSE;
    vector startpos = llList2Vector(gKfmSegments,0);   // start of path
    //  Start at segment where last found the position.  
    //  Stop at end of list, finished lookaheaddist, or no longer moving.
    //  pos and all segment points are at ground level.
    for (i=gKfmSegmentCurrent; i<llGetListLength(gKfmSegments)-1 && lookaheaddist > 0 && gPathMoveActive; i++)
    {   vector p0 = llList2Vector(gKfmSegments,i);
        vector p1 = llList2Vector(gKfmSegments,i+1);
        if (!foundseg) 
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHERRSEGDISTTOL); // distance along seg, or INFINITY
            if (distalongseg < HUGE)    
            {   
                gKfmSegmentCurrent = i;                 // advance current segment pos
                foundseg = TRUE;                        // start checking from here                
            }
        }
        if (foundseg)                                   // if pos is in this or a previous segment
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHERRSEGDISTTOL); // distance along seg, or NAN
            if (distalongseg < HUGE)                    // point is in capsule around segment
            {   
                float seglength = llVecMag(p0-p1);
                if (distalongseg < 0) { distalongseg = 0; } // bound to segment
                if (distalongseg > seglength) { distalongseg = seglength; }
                float castdist = seglength - distalongseg;  // cast to end of segment
                if (lookaheaddist < castdist) { castdist = lookaheaddist; } // if running out of cast distance
                if  (castdist <= 0) { return; };        // at end
                vector aheadvec = llVecNorm(p1-p0);     // vector ahead
                vector pos2 = pos + aheadvec*castdist;  // how far to cast
                vector sidevec = (aheadvec % <0,0,1>)*(gPathWidth*0.5);    // sideways offset to check        
                if (pathobstacleraycast(pos+halfheight, pos2+halfheight)) { return; }         // look ahead horizontally at waist level
                if (pathobstacleraycast(pos+fullheight, pos2+fullheight)) { return; }  // look ahead horizontally at head level
#ifdef NOTYET // Do we really need this overhead? Vehicle avoid should prevent bumping into physical objects.
                ////pathobstacleraycast(pos+groundheight, pos2+groundheight);          // look ahead horizontally at ground level
                if (pathobstacleraycast(pos+halfheight+sidevec, pos2+halfheight+sidevec)) { return; }  // look ahead horizontally on left and right
                if (pathobstacleraycast(pos+halfheight-sidevec, pos2+halfheight-sidevec)) { return; }  // look ahead horizontally     
                if (pathobstacleraycast(pos+fullheight+sidevec, pos2+fullheight+sidevec)) { return; }  // look ahead horizontally on left and right
                if (pathobstacleraycast(pos+fullheight-sidevec, pos2+fullheight-sidevec)) { return; }  // look ahead horizontally 
#endif // NOTYET            
                lookaheaddist -= castdist;              // reduce distance ahead
                pos = p1;                               // start of next segment is start of next cast
            }
        }   
    }
    if (!foundseg)
    {   pathMsg(PATH_MSG_WARN,"Unable to find " + (string)pos + " in " + llDumpList2String(gKfmSegments,","));  // off the path?
        pathmovedone(PATHERROFFPATH, NULL_KEY);                     // KFM crept out of position. Retry.
        return;                                                     // and stop here
    }
    //  Check for walkable support under the current position
    integer status = pathcheckforwalkable();
    if (status)
    {   pathmovedone(status, NULL_KEY);                         // big trouble. Probably stuck here
        return;
    }
    if (llVecMag(llList2Vector(gPathMoveLastgoodpos,-1) - groundpos) > PATHERRMINGOODPOS)  // if moved to a new good pos
    {   groundpos += (llGetRegionCorner() - gPathInitialRegionCorner);   // saved points are relative to initial ground pos
        gPathMoveLastgoodpos += [groundpos];                    // save this ground level position for recovery
        if (llGetListLength(gPathMoveLastgoodpos) > PATHMAXSAVEDGOODPOS)    // limit list length
        {   gPathMoveLastgoodpos = llDeleteSubList(gPathMoveLastgoodpos,0,0); } // by removing oldest entry
    }

}
//
//  pathchecktargetmoved -- check if pursuit target moved.
//
//  This makes the character chase the target. 
//  Must not replan too often; replanning is slow.
//
pathchecktargetmoved()
{   if (gPathMoveTarget == NULL_KEY) { return; }                // not in pursuit, no check
    list details = llGetObjectDetails(gPathMoveTarget, [OBJECT_POS]);   // get object position
    if (details == [])
    {   pathMsg(PATH_MSG_WARN, "Pursue target left sim."); 
        pathmovedone(PATHERRTARGETGONE, gPathMoveTarget);       // target is gone, abort pursue
        return;
    }
    //  If pursue target moved more than half the distance to the goal, but at least 2m, replan.
    vector pos = llGetPos();                                    // where we are
    vector targetpos = llList2Vector(details,0);                // where target is
    float disttotarget = llVecMag(targetpos - pos);
    float distmoved = llVecMag(gPathMoveTargetPos - targetpos); // distance target moved since replan
    if (distmoved < PATHMOVEMINTARGETMOVE) { return; }          // has not moved enough to replan
    if (distmoved < disttotarget * PATHMOVEMINTARGETFRACT) { return; } // has not moved enough to replan
    pathmovedone(PATHERRTARGETMOVED, gPathMoveTarget);          // target moved, must replan and chase
}
//  
//
//  pathmovetimer  -- timer event, check progress and do ray casts
//
pathmovetimer()
{   float interval = llGetAndResetTime();                       // time since last tick 
    if (interval > 0.5) { pathMsg(PATH_MSG_NOTE,"Timer ticks slow: " + (string)interval + "s."); } // ***TEMP*** log really slow operations
    if (gPathMoveActive)                                        // if moving
    {   if (gPathStartRegionCorner != llGetRegionCorner())      // if we changed regions
        {   pathmovedone(PATHERRREGIONCROSS, NULL_KEY); return; } // must abort this move, restart will continue later
    }
    if (gPathMoveActive && gKfmSegments != [])                  // if we are moving and have a path
    {   pathcheckdynobstacles(); }                              // ray cast for obstacles
    if (gPathMoveActive && gKfmSegments != [])                  // if we are moving and have a path
    {   pathchecktargetmoved(); }                               // check if pursuit target moved
    if (gPathMoveActive)                                        // if we are turned on
    {   
        integer now = llGetUnixTime();                          // time now
        if (now - gPathMoveTimetick > PATHMOVECHECKSECS)
        {   gPathMoveTimetick = now;                            // update last check time
            vector pos = llGetPos();
            if (llVecMag(pos - gPathMoveLastpos) > 0.01)        // if moving at all
            {   return; }                                       // OK
            //  No KFM movement. Something has gone wrong. 
            pathmovedone(PATHERRMAZEKFMSTALL, NULL_KEY);         // stalled
        }
    }
}

//
//  pathmoverequestrcvd  -- JSON from path execution
//
//  We get a list of the points the character is currently following. 
//  That tells us what direction to look for obstacles.
//
//  Protocol is to send a "startmove" as each KFM segment starts.
//  Send a "stopmove" when the entire path is finished or when not moving.
//  This will result in a "stall" event being sent if something goes wrong.
//
pathmoverequestrcvd(string jsn) 
{   ////pathMsg(PATH_MSG_INFO,"Path move request: " + jsn);
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type 
    pathMsg(PATH_MSG_NOTE,"Path move request: " + jsn + " Request type: \"" + requesttype + "\""); 
    if (requesttype == "startmove")                         // start moving
    {   //  Set up for ray casting.
        gPathMoveId = (integer)llJsonGetValue(jsn, ["pathid"]);
        gPathMoveTarget = (key)llJsonGetValue(jsn, ["target"]); // who we are chasing, if anybody
        gPathMoveMaxSpeed = (float)llJsonGetValue(jsn,["speed"]); 
        gPathMoveMaxTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]); 
        gPathStartRegionCorner = (vector)llJsonGetValue(jsn,["refpt"]);  // region corner to which points are referenced
        list ptsstr = llJson2List(llJsonGetValue(jsn, ["points"])); // points, as strings
        gKfmSegments = [];                                  // clear stored path used for ray cast direction
        gKfmSegmentCurrent = 0;
#ifdef OBSOLETE // moved closer to actual move
        if (gPathStartRegionCorner != llGetRegionCorner())  // if crossed a region boundary during planning
        {   pathMsg(PATH_MSG_ERROR,"Crossed region boundary during planning, no move."); // ***TEMP** as error, checking for unusual event
            pathmovedone(PATHERRREGIONCROSS, NULL_KEY);     // crossed a region during planning, start over
            return;
        }
#endif // OBSOLETE
        integer i;
        integer len = llGetListLength(ptsstr);
        assert(len >= 2);                                   // required to have a start point and a dest at least
        for (i=0; i<len; i++) { gKfmSegments += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors
        //  Get position of pursuit target if tracking
        if (gPathMoveTarget != NULL_KEY) 
        {   list details = llGetObjectDetails(gPathMoveTarget, [OBJECT_POS]);   // get object position
            if (details == [])                              // avatar not found
            {   pathMsg(PATH_MSG_WARN, "Pursue target not found."); gPathMoveTargetPos = ZERO_VECTOR; }   // gone from sim, timer will detect
            else
            {   gPathMoveTargetPos = llList2Vector(details,0);   }   // where target is
        }
        ////if (gPathMoveActive || gPathMoveMoving || gPathMoveRecovering)
        if (gPathMoveMoving || gPathMoveRecovering)         // if actively doing something else
        {   pathmovedone(PATHERRREQOUTOFSYNC,NULL_KEY);     // request is out of sync; we are doing something else.
            return;
        }
        gPathMoveActive = TRUE;                             // move system is active
        gPathMoveMoving = TRUE;                             // character is moving
        gPathMoveTimetick = llGetUnixTime();                // reset stall timer
        llResetTime();                                      // ***TEMP***
        llSetTimerEvent(PATHMOVERAYTIME);                   // switch to fast timer for ray casts for obstructions
        pathexedokfm();                                     // actually do the avatar movement      
    } else if (requesttype == "stopmove")                   // stop moving
    {   
        if (!gPathMoveActive) { return; }                   // we are not running, ignore
        assert(!gPathMoveRecovering);                       // should not be active and recovering at same time
        if (gPathMoveMoving)                                // if we were moving, this is a forced stop
        {   pathMsg(PATH_MSG_WARN,"Unexpected stop requested at " + (string)llGetPos());  // this should not normally happen
            pathmovedone(PATHERRSTOPREQ,NULL_KEY);          // stop motion
        }
        gKfmSegments = [];
        llSetTimerEvent(0.0);                               // shut down and stop timer
        gPathMoveActive = FALSE;                            // move system is active
        gPathMoveMoving = FALSE;                            // character is moving
    } else if (requesttype == "recover")                    // recover to known good position, requested by pathprep
    {   //  This is done in move because it's an in-world move, and we do all in-world moves here.
        //  The actual work gets done in the recover task, but we make sure here that we are stopped.
        pathMsg(PATH_MSG_WARN,"Recover request: " + jsn);   
        integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]); // must have pathid so caller can match
        if (gPathMoveActive || gPathMoveMoving || gPathMoveRecovering)   // motion in progress, can't do a forced recovery
        {   pathMsg(PATH_MSG_WARN,"Recover move requested while in motion"); // Recovery during recovery, usually. Fail and restart
            pathdonereply(PATHERRREQOUTOFSYNC, NULL_KEY, pathid);            // end entire operation  
        }     
        else 
        {   gPathMoveRecovering = TRUE;                     // now in recovery mode, until other task replies.             
            //  Pass buck to recover task, which has enough empty space for the full obstacle test.
            llMessageLinked(LINK_THIS, LINKMSGRECOVERREQUEST,
                llList2Json(JSON_OBJECT,["request","recover", "pathid", pathid, "refpt", gPathInitialRegionCorner,
                        "recoverpoints", llList2Json(JSON_ARRAY,  gPathMoveLastgoodpos)]),"");
        }      
    } else {
        pathMsg(PATH_MSG_ERROR,"Bad request: " + jsn);
    }
}
//
//  pathrecoverreplyrcvd -- reply from recovery
//
pathrecoverreplyrcvd(string jsn, key hitobj)
{
    pathMsg(PATH_MSG_WARN,"Path recover reply: " + jsn);
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type  
    if (requesttype == "recover")                       // recovery complete
    {  
        integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
        integer status = (integer)llJsonGetValue(jsn, ["status"]);
        gPathMoveLastgoodpos = llDeleteSubList(gPathMoveLastgoodpos,-1,-1); // consume recovery point just used
        gPathMoveRecovering = FALSE;                    // no longer recovering
        pathdonereply(status, hitobj, pathid);          // report move completion
    } else {
        pathMsg(PATH_MSG_ERROR,"Bad reply: " + jsn);
    }
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
    gPathMoveLastdest = llList2Vector(pts,-1);  // last point. We should end up here.
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
    placesegmentmarker(MARKERLINE, pprev, p0, gPathWidth, TRANSGREEN, 0.20);      // place a temporary line on the ground in-world.
#endif // MARKERS
    vector rp = p0 - pos;                       // p0 in relative coords - advances us to p0
    rp.z += gPathHeight * 0.5;              // add half-height, because path is at ground level
    //  Rotation is to the average direction of the previous and next sections in the XY plane.
    vector invec = p0-pprev;                    // incoming direction
    vector outvec = p1-p0;                      // outgoing direction
    float inveclen = llVecMag(invec);           // distance of this move
    vector invecnorm = llVecNorm(<invec.x, invec.y, 0>);
    vector outvecnorm = llVecNorm(<outvec.x,outvec.y,0>);
    if (p1 == ZERO_VECTOR) { outvecnorm = invecnorm; } // last section, no turn
    vector dir = llVecNorm(invecnorm+outvecnorm);// next direction
    rotation rr = RotFromXAxis(dir) / rot;      // relative rotation
    rr = NormRot(rr);                           // why is this necessary?
    //  Time computation. Speed is limited by rotation rate.
    float angle = llFabs(llAngleBetween(ZERO_ROTATION, rr));    // how much rotation is this?
    float rsecs = angle / gPathMoveMaxTurnspeed; // minimum time for this move per rotation limit
    float rt = inveclen / gPathMoveMaxSpeed;     // minimum time for this move per speed limit
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
{   ////pathMsg(PATH_MSG_WARN,"Entering pathexddokfm. Segments:" + llDumpList2String(gKfmSegments,","));                // ***TEMP***
    vector kfmstart = llList2Vector(gKfmSegments,0);    // first point, which is where we should be
    assert(kfmstart != ZERO_VECTOR);                    // must not be EOF marker  
    vector pos = llGetPos();                            // we are here
    if (pathvecmagxy(kfmstart - pos) > PATHERRMAXCREEP)     // we are out of position
    {   pathMsg(PATH_MSG_WARN, "KFM out of position at start. At " + (string)pos + ". Should be at " + (string)kfmstart); 
        pathmovedone(PATHERRBADMOVEEND, NULL_KEY);       // error, must start a new operation to recover
        return; 
    }
    //  Last-chance check for being in wrong region when starting keyframe motion.
    if (gPathStartRegionCorner != llGetRegionCorner())  // if crossed a region boundary during planning
    {   pathMsg(PATH_MSG_ERROR,"Crossed region boundary while setting up motion, no move."); // ***TEMP** as error, checking for unusual event
        pathmovedone(PATHERRREGIONCROSS, NULL_KEY);     // crossed a region during planning, start over
        return;
    }
    pathMsg(PATH_MSG_NOTE,"Input to KFM: " + llDumpList2String(gKfmSegments,","));     // what to take in
    list kfmmoves = pathexebuildkfm(pos, llGetRot(), gKfmSegments);   // build list of commands to do
    if (kfmmoves != [])                             // if something to do (if only one point stored, nothing happens)
    {   llSetKeyframedMotion(kfmmoves, [KFM_MODE, KFM_FORWARD]);            // begin motion
        ////pathMsg(PATH_MSG_NOTE,"Starting motion, KFM commands: " + llDumpList2String(kfmmoves,","));   // ***TEMP***
        integer freemem = llGetFreeMemory();            // how much memory left here, at the worst place       
        if (freemem < gPathMoveFreemem) 
        {   gPathMoveFreemem = freemem; 
            pathMsg(PATH_MSG_NOTE, "Move task free memory: " + (string)freemem);
        }   // record free memory
    }
}

//
//  pathmovecollision -- collided with something
//
pathmovecollision(integer num_detected)
{   
    if (!gPathMoveMoving) { return; }    // not moving, not our fault
    integer i;
    for (i=0; i<num_detected; i++)
    {   key hitobj = llDetectedKey(i);
        if (hitobj != NULL_KEY)              // null key is land
        {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // hit a non-walkable
            {   pathMsg(PATH_MSG_WARN,"Collided with " + llDetectedName(i) + " at " + (string)llGetPos());
                pathmovedone(PATHERRCOLLISION, llDetectedKey(i)); // stop
                return;
            }
        }
    }
}
//
//  pathchangedregion -- region crossing
//
pathchangedregion()
{   
    pathMsg(PATH_MSG_WARN,"Crossed region boundary.");
    if (!gPathMoveMoving) { return; }               // not moving, don't have to kill move
    pathmovedone(PATHERRREGIONCROSS, NULL_KEY);     // stop, let retry pick up in the new region
}

//
//  The main program of the move task.
//
default
{
    state_entry()
    {   pathinitutils();                                        // init library
        pathmoveinit();                                         // init our KFM system 
        gPathMsgLevel = PATH_MSG_INFO;                          // will be overridden at first move but needed if first event is something else       
    }

    on_rez(integer rezparam) 
    {   llResetScript(); }

    link_message(integer status, integer num, string jsn, key id )
    {   if (num == LINKMSGMOVEREQUEST)                          // request to move
        {   pathmoverequestrcvd(jsn); }
        else if (num == LINKMSGRECOVERREPLY)                    // recovery complete or failed
        {   pathrecoverreplyrcvd(jsn, id); 
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn);                                // initialize params
        } else if (num == DEBUG_MSGLEV_BROADCAST)               // set debug message level for this task
        {    debugMsgLevelSet(jsn);
        }
    }
    
    timer()
    {   pathmovetimer();   }                                     // pass timer event
    
    moving_end()
    {   pathmovemovementend(); }   
    
    collision_start(integer num_detected)
    {   pathmovecollision(num_detected); }
    
    changed(integer change)
    {   if (change & CHANGED_REGION)                            // if we just crossed a region boundary
        {   ////llOwnerSay("Region: " + llGetRegionName());         // ***TEMP***
            pathchangedregion();                                // changed region, handle
        }       
    }
}
