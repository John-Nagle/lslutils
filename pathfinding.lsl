//
//  Pathfinding library
//
//  ALPHA TEST version
//
//  Makes Second Life pathfinding work reliably.
//
//  Animats
//  January, 2019
//  License: GPL
//  (c) 2019 John Nagle
//
//  Adds checking and retry, which is essential to getting anything done reliably.
//
//  Works around Second Life bugs BUG-5021, BUG-226061.
//
//  To use this, change your code as follows:
//
//      llCreateCharacter -> pathCreateCharacter
//      llUpdateCharacter -> pathUpdateCharacter
//      llNavigateTo -> pathNavigateTo
//      llPursue -> pathPursue
//      llWander -> pathWander
//      llEvade -> pathEvade
//      llFlee -> pathFlee
//
//  Also:
//      path_update -> pathUpdateCallback - define this for your own callback
//      In your path_update, call pathUpdate.
//
//      (llPatrol is not implemented. Suggest using llNavigateTo for each goal point.)
//
//  and call pathTick from a timer or sensor at least once every 2 seconds when pathfinding.
//
//  Optionally, call pathCollide(num_detected) when the character hits an obstacle.
//  This will switch to a more conservative mode needed in overloaded sims.
//
//  Retrying will happen when necessary.
//
//  Call pathStop() to stop any operation in progress. It is not necessary to call pathStop before
//  starting a new operation.
//
//  llNavigateTo and llPursue will always produce a callback to pathUpdateCallback when they are done or
//  have failed to reach their goal; this package provides timeouts, retries, and checks for them
// 
//  llWander, llEvade, and llFleeFrom do not have timeouts, retries, or checks, because they
//  have no well-defined goal. 
//
//  Common definitions
#include "pathdefs.lsl"
//
//  Configuration
//
integer PATH_STALL_TIME = 90;                           // (secs) really stalled if greater than this
integer PATH_START_TIME = 2;                            // (secs) allow this long for path operation to start
float   PATH_GOAL_TOL = 1.0;                            // (m) how close to dest is success?
float   PATH_GOOD_MOVE_DIST = 1.5;                      // (m) must move at least this far
integer PATH_GOOD_MOVE_TIME = 2;                        // (secs) this often
float   PATH_ROTATION_EASE = 0.5;                       // (0..1) Rotation ease-in/ease out strength
float   PATH_MIN_PATHFINDING_PCT = 25.0;                // (0..100) Minimum pathfinding steps executed to get out of slow mode
float   PATH_RANDOM_UNSTICK_MOVE = 0.25;                // (m) max distance to move randomly to get unstuck
//
//  Globals internal to this module
//
                                                            // the current request
integer gPathPathmode = PATHMODE_OFF;                       // what are we doing right now.  
key gPathTarget = NULL_KEY;                                 // target avatar, when pursuing
vector gPathGoal;                                           // goal when navigating
vector gPathDist;                                           // distance when wandering
list gPathOptions;                                          // options for all operations
float gPathTolerance;                                       // how close to end is success
list gPathCreateOptions;                                    // options from create and update
 
                                                            // other state
integer gPathRequestStartTime;                              // UNIX time path operation started
integer gPathActionStartTime;                               // UNIX time path operation started
vector gPathRequestStartPos;                                // position at start
vector gPathLastGoodPos;                                    // position of last good move
integer gPathLastGoodTime;                                  // time of last good move
integer gPathRetries;                                       // retries after zero speed
integer gPathCollisionTime;                                 // time of last collision where skipped frames mode turned off, or 0
key gPathSafeParcelKey;                                     // key of last parcel known to be safe to enter
key gPathSafeParcelOwner;                                   // owner of last parcel known to be safe to enter
key gPathSafeParcelGroup;                                   // group of last parcel known to be safe to enter
vector gPathLastSafePos;                                    // last safe position - go back if real trouble
float gPathFindingPct;                                      // check for sim in severe overload
//
//  Call these functions instead of the corresponding LSL functions
//
#ifdef OBSOLETE // now called via pathfinder.lsl because this is run in a seperate script due to size problems.
pathCreateCharacter(list options)
{   gPathCreateOptions = options;                           // save options for later checks
    llCreateCharacter(options);
}

pathUpdateCharacter(list options)
{   gPathCreateOptions = options;                           // save options for later checks
    llUpdateCharacter(options);
}

pathNavigateTo(vector goal, list options)
{   if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathTolerance = PATH_GOAL_TOL;                         // default tolerance
    gPathGoal = goal;                                       // head here
    gPathOptions = options;                                 // using these options
    pathActionStart(PATHMODE_NAVIGATE_TO);                  // use common function
}

pathPursue(key target, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathTolerance = PATH_GOAL_TOL;                         // default tolerance
    gPathOptions = options;                                 // options for operation
    integer i;
    for (i=0; i<llGetListLength(gPathOptions); i++)         // search strided list
    {   if (llList2Integer(gPathOptions,i) == PURSUIT_GOAL_TOLERANCE) // if goal tolerance item
        {   gPathTolerance = llList2Float(gPathOptions,i+1);// get tolerance
            gPathTolerance = gPathTolerance + PATH_GOAL_TOL;    // add safety margin
            pathMsg(PATH_MSG_INFO, "Pursue tolerance (m): " + (string) gPathTolerance);
        }
    }
    gPathTarget = target;
    pathActionStart(PATHMODE_PURSUE);                       // use common function
}
                
pathWander(vector goal, vector dist, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathGoal = goal;                                       // wander around here
    gPathDist = dist;                                       // within this distance
    gPathOptions = options;                                 // using these options
    pathActionStart(PATHMODE_WANDER);                       // use common function
}

pathEvade(key target, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing  
    gPathTarget = target;
    gPathOptions = options;
    pathActionStart(PATHMODE_EVADE);
}

pathFleeFrom(vector goal, float distmag, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathGoal = goal;
    gPathDist = <distmag,0,0>;                              // because we only use the magnitude
    gPathOptions = options;
    pathActionStart(PATHMODE_FLEE_FROM);                        // use common fn
}
#endif // OBSOLETE

//
//  pathInit -- call this at startup/rez to initialize the variables
//
pathInit()
{
    gPathPathmode = PATHMODE_UNINITIALIZED;                     // not ready yet
    gPathTarget = NULL_KEY;                                     // no goal yet
    gPathSafeParcelKey = NULL_KEY;                                  // no known safe parcel yet
    gPathSafeParcelOwner = NULL_KEY;
    gPathSafeParcelGroup = NULL_KEY;
    gPathLastGoodPos = ZERO_VECTOR;                             // no known good position yet
    gPathLastSafePos = ZERO_VECTOR;                             // no known safe position yet
    gPathFindingPct = 100.0;                                    // assume pathfinding system OK
}
//
//  pathTick --  call this at least once every 2 seconds.
//
pathTick()
{
    if (gPathPathmode == PATHMODE_UNINITIALIZED) { return; }    // need startup
    if (gPathPathmode == PATHMODE_OFF) { return; }              // not pathfinding, nothing to do.
    gPathFindingPct = llGetSimStats(SIM_STAT_PCT_CHARS_STEPPED);// check for sim in severe overload
    integer status = pathStallCheck();                          // are we stuck?
    if (status == PATHSTALL_NONE) 
    {   if (gPathCollisionTime != 0 && llGetUnixTime() - gPathCollisionTime > PATH_GOOD_MOVE_TIME * (100.0 / (gPathFindingPct + 1.0))) // if going OK for a while
        {   ////llOwnerSay("Pathfinding pct: " + (string)gPathFindingPct);    // ***TEMP***
            if (gPathFindingPct > PATH_MIN_PATHFINDING_PCT)
            {   pathResetCollision(); }  
            else 
            {   pathMsg(PATH_MSG_WARN, "Staying in slow mode, pathfinding running too slow: " + (string)gPathFindingPct  + "% of normal");
                gPathCollisionTime = llGetUnixTime() + 60;      // don't recheck for a full minute
            }
        }                           
        return;                                                 // no problem, keep going
    }
    if (status == PU_GOAL_REACHED)                              // success
    {   pathUpdateCallback(status,[]);                          // call user-defined completion fn
        return;
    }
    if ((status == PATHSTALL_STALLED) || (status == PATHSTALL_NOPROGRESS) || (status == PATHSTALL_UNREACHABLE)) // total fail
    {   pathStop();                                             // stop operation in progress
        pathUpdateCallback(status,[]);                          // call user defined completion function
        return;
    }
    //  Possible recoverable problem
    if (status == PATHSTALL_UNSTICK)                            // if unstick needed
    {
        pathMsg(PATH_MSG_WARN, "Retrying " + pathActionName(gPathPathmode));
        integer success = pathUnstick();                        // try to unstick situation
        if (!success)
        {   pathStop();                                         // fails
            pathUpdateCallback(PATHSTALL_CANNOT_MOVE,[]);       // we are stuck
            return;
        }
    }
    if ((status == PATHSTALL_OUT_OF_BOUNDS) || (status == PU_FAILURE_PARCEL_UNREACHABLE)) // somehow in a bad place
    {   
        integer status = pathRecoverOutOfBounds(gPathLastSafePos);
        if (status != PU_GOAL_REACHED)                          // if that didn't work
        {   pathUpdateCallback(status,[]);                      // big trouble
            return;
        }
    }
    //  Recoverable error, try again.
    pathActionRestart();         
}

//
//  pathCollide --  call when character hits something it shouldn't.
//
//  This slows it down for a while by not skipping frames and making big jumps.
//
pathCollide(integer num_detected)
{   if (gPathPathmode == PATHMODE_UNINITIALIZED) { return; }    // need startup
    if (gPathPathmode == PATHMODE_OFF) { return; }              // not pathfinding, nothing to do.
    //  Did we collide with a non-walkable object?
    integer collided = FALSE;
    integer hitavatar = FALSE;
    integer i;
    for (i=0; i<num_detected; i++)
    {   key id = llDetectedKey(i);                              // object hit 
        list hitinfo = llGetObjectDetails(id, [OBJECT_PATHFINDING_TYPE]); // what did we hit?
        if (llGetListLength(hitinfo) == 1)
        {   integer hitobjtype = llList2Integer(hitinfo,0);     // get pathfinding type
            if (hitobjtype != OPT_WALKABLE)                     // if not a walkable
            {   collided = TRUE;                                // hit a non walkable object
                if (hitobjtype == OPT_AVATAR)                   // if hit an avatar
                {   hitavatar = TRUE;                           // want to avoid that
                    llSay(0,"Excuse me.");
                }
                string name = llList2String(llGetObjectDetails(id, [OBJECT_NAME]),i);
                if (name == "") { name = (string)id; }  // unnamed object
                pathMsg(PATH_MSG_WARN, "Hit obstacle " + name + ".");
            } 
        } else {
            pathMsg(PATH_MSG_ERROR, "Unable to get pathfinding type for object."); // unlikely
        }
    }
    if (!collided) { return; }                                  // not a real collision
    //  Hit something. Must do something about it.
    if (gPathCollisionTime != 0)                                // if already in slow mode
    {   gPathCollisionTime = llGetUnixTime();                   // update time of last collision
        return;
    }
    gPathCollisionTime = llGetUnixTime();                       // entering slow mode
    //  More accurate motion, but slower. Needed when sim is overloaded.
    //  If we hit something, avoid characters and dynamic obstacles briefly.
    list newopts = pathReplaceOption(gPathCreateOptions, [CHARACTER_ACCOUNT_FOR_SKIPPED_FRAMES, FALSE]); 
    if (hitavatar) 
    {   newopts = pathReplaceOption(newopts, [CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS | AVOID_DYNAMIC_OBSTACLES]); }
    //  And switch to "slow mode".
    llUpdateCharacter(newopts); 
    pathMsg(PATH_MSG_INFO, "Collision - slow mode.");
}
//  
//
//  Internal functions
//


pathPursueToleranceUpdate(list options)                         // update pursue tolerance from options
{   gPathTolerance = PATH_GOAL_TOL;                             // default tolerance
    list tolopt = pathGetOption(options, PURSUIT_GOAL_TOLERANCE); // get goal tolerance
    if (llGetListLength(tolopt) == 1)
    {   gPathTolerance = llList2Float(tolopt,0);    // get tolerance
        gPathTolerance = gPathTolerance + PATH_GOAL_TOL;    // add safety margin
        ////pathMsg(PATH_MSG_INFO, "Pursue tolerance (m): " + (string) gPathTolerance);
    }
}


pathStop()
{   if (gPathPathmode == PATHMODE_UNINITIALIZED) { return; }    // need startup
    if (gPathPathmode == PATHMODE_OFF) { return; }              // not doing anything
    llExecCharacterCmd(CHARACTER_CMD_STOP,[]);                  // stop whatever is going on
    pathResetCollision();                                       // reset slow mode if needed
    gPathPathmode = PATHMODE_OFF;                               // not pathfinding
}

pathActionStart(integer pathmode, list options)
{
    pathMsg(PATH_MSG_INFO, "Starting " + pathActionName(pathmode));
    if (gPathPathmode == PATHMODE_UNINITIALIZED && pathmode != PATHMODE_CREATE_CHARACTER)   // must create character first 
    {
        pathMsg(PATH_MSG_WARN, "Not initialized");
        pathUpdateCallback(PATHSTALL_UNINITIALIZED,[]);
        return;
    }
    gPathPathmode = pathmode;
    gPathOptions = options;                                     // options for this operation
    gPathRequestStartTime = llGetUnixTime();                    // start time of path operation
    gPathRetries = 0;                                           // no retries yet
    gPathCollisionTime = 0;                                     // no recent collision
    gPathRequestStartPos = llGetPos();                          // starting position
    pathResetCollision();                                       // reset slow mode if needed
    pathActionRestart();                                        // start the task
}

//
//  pathActionRestart  -- restart whatever was running
//
pathActionRestart()
{
    gPathActionStartTime = llGetUnixTime();                     // start timing
    gPathLastGoodTime = gPathActionStartTime;                   // for progress detector
    gPathLastGoodPos = llGetPos();
    if (gPathPathmode == PATHMODE_NAVIGATE_TO)                  // depending on operation in progress
    {   pathMsg(PATH_MSG_INFO, "llNavigateTo " + (string)gPathGoal);
        llNavigateTo(gPathGoal, gPathOptions);                  // restart using stored values
    } else if (gPathPathmode == PATHMODE_PURSUE)
    {   pathMsg(PATH_MSG_INFO, "llPursue of " + llKey2Name(gPathTarget));
        pathPursueToleranceUpdate(gPathOptions);                // need some info from the options
        llPursue(gPathTarget, gPathOptions);
    } else if (gPathPathmode == PATHMODE_WANDER)
    {   pathMsg(PATH_MSG_INFO, "llWander around " + (string)gPathGoal);
        llWanderWithin(gPathGoal, gPathDist, gPathOptions);
    } else if (gPathPathmode == PATHMODE_EVADE)
    {   pathMsg(PATH_MSG_INFO, "llEvade of " + llKey2Name(gPathTarget));
        llEvade(gPathTarget, gPathOptions); 
    } else if (gPathPathmode == PATHMODE_FLEE_FROM)
    {   pathMsg(PATH_MSG_INFO, "llFleeFrom " + (string)gPathGoal);
        llFleeFrom(gPathGoal, llVecMag(gPathDist), gPathOptions);
    } else if (gPathPathmode == PATHMODE_OFF)
    {   pathStop();
    } else if (gPathPathmode == PATHMODE_CREATE_CHARACTER)      // only use at startup
    {   gPathCreateOptions = gPathOptions;
        llDeleteCharacter();                                    // in case already created
        llCreateCharacter(gPathCreateOptions); 
        gPathPathmode = PATHMODE_OFF;                           // we are now ready to go
        list heredata = llGetParcelDetails(llGetPos(), [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP]);
        gPathSafeParcelOwner = llList2Key(heredata,0);          // assume that where user turns us on is safe
        gPathSafeParcelGroup = llList2Key(heredata,1);  
        gPathLastSafePos = ZERO_VECTOR;                         // no safe pos after restart              
    } else if (gPathPathmode == PATHMODE_UPDATE_CHARACTER)
    {   gPathCreateOptions = gPathOptions;
        llUpdateCharacter(gPathCreateOptions);
    } else {
        pathMsg(PATH_MSG_ERROR, "pathActionRestart called incorrectly");
    }
}

//
//  pathResetCollision -- go back to fast mode if no collisions for a while
//
pathResetCollision()
{   llUpdateCharacter(gPathCreateOptions);              // back to normal mode
    if (gPathCollisionTime != 0) { pathMsg(PATH_MSG_INFO, "Slow mode turned off."); }
    gPathCollisionTime = 0;                             // end of slow mode
}

//
//  pathRecoverOffNavmesh -- off the navmesh, emergency recovery
//
integer pathRecoverOffNavmesh()
{   vector pos = llGetPos();
    vector charsize = llGetScale();            
    float charaboveground = charsize.z * 0.5;   // must query close to ground level
    list points = llGetClosestNavPoint(pos - <0,0,charaboveground>,[]);   // get navmesh point near object.
    if (llGetListLength(points) < 1)
    {   pathMsg(PATH_MSG_ERROR, "Off the navmesh and can't get back."); return(PU_FAILURE_INVALID_START); } // failed
    vector newpos = llList2Vector(points,0);        // get nearest point on navmesh
    if (!pathValidDest(newpos))                     // if recovery pos is off parcel
    {   pathMsg(PATH_MSG_WARN, "Off the navmesh and off parcel."); return(PATHSTALL_OUT_OF_BOUNDS); }
    newpos.z = newpos.z + charaboveground;          // put back above ground, not in it.
    llSetPos(newpos);                               // FORCED MOVE - try to go there
    pathMsg(PATH_MSG_INFO, "Off navmesh, recovered");
    return(PU_GOAL_REACHED);                        // assume success    
}
//
//  pathRecoverOutOfBounds  -- somehow got out of bounds
//
//  Do a non-physical move back to a safe place.
//
integer pathRecoverOutOfBounds(vector safepos)
{
    if ((safepos == ZERO_VECTOR) || (llVecMag(safepos - llGetPos()) < 0.05))     // if no better pos available
    {  pathMsg(PATH_MSG_ERROR, "Out of bounds and no recovery position."); // not recoverable
       pathUpdateCallback(PATHSTALL_UNREACHABLE,[]);          // report unreachable, probably won't work
       return(PU_FAILURE_PARCEL_UNREACHABLE);                   // really stuck
    }
    pathMsg(PATH_MSG_WARN, "Forced move to " + (string) safepos);
    //  Recovery sequence for ban lines and such
    llDeleteCharacter();
    llSleep(0.5);
    llSetPos(safepos);                              // move to last good position
    //  Total reset of pathfinding      
    llSleep(0.5);
    llCreateCharacter(gPathCreateOptions);          // recreate after forced move
    llSleep(0.5);
    if (llGetStatus(STATUS_PHYSICS))                // if physics turned back on
    {   return(PU_GOAL_REACHED); }                  // well, our local goal, anyway
    pathMsg(PATH_MSG_ERROR, "Out of bounds and recovery failed."); // not recoverable
    return(PU_FAILURE_PARCEL_UNREACHABLE);          // report unreachable, do something else
}
//
//  pathStallCheck  --  is pathfinding stalled?
//
//  Checks:
//      1. Are we out of our allowed area?
//      2. Stall timeout - entire operation took too long. Usual limit is about 2 minutes.
//      3. We're at the goal, so stop.
//      4. Zero velocity and zero angular rate - pathfinding quit on us.
//      5. No significant progress in last few seconds - pathfinding is stalled dynamically.
//
integer pathStallCheck()
{
    if ((gPathPathmode == PATHMODE_OFF) || (gPathPathmode == PATHMODE_UNINITIALIZED)) { return(PATHSTALL_NONE); } // idle, cannot stall
    vector pos = llGetPos();
    if ((llGetStatus(STATUS_PHYSICS) == 0) || (!pathValidDest(pos)))
    {   pathMsg(PATH_MSG_WARN,"Out of bounds at " + (string)pos);
        return(PATHSTALL_OUT_OF_BOUNDS);            // we are somewhere we should not be. Force back to a good location
    }
    gPathLastSafePos = pos;                         // this position is safe, save it for recovery  
    if ((gPathPathmode == PATHMODE_WANDER) 
    || (gPathPathmode == PATHMODE_EVADE) 
    || (gPathPathmode == PATHMODE_FLEE_FROM)) { return(PATHSTALL_NONE); }  // no meaningful completion criterion 
    //  Stall timeout
    integer now = llGetUnixTime();                  // UNIX time since epoch, secs
    if (now - gPathRequestStartTime > (PATH_STALL_TIME * (100.0 / (gPathFindingPct + 1.0))))     // if totally stalled
    {                                               // stop whatever is going on
        pathMsg(PATH_MSG_ERROR, pathActionName(gPathPathmode) + " stalled after " + (string)(now - gPathRequestStartTime) + " seconds."); 
        pathStop(); 
        return(PATHSTALL_STALLED);
    }
    //  At-goal check. Usually happens when start and goal are too close
    if (pathAtGoal(gPathTolerance))                 // must be close to force an abort
    {   pathStop();                                 // request complete
        return(PU_GOAL_REACHED);                    // Happy ending
    }

    //  Zero velocity check
    if (now - gPathActionStartTime > (PATH_START_TIME * (100.0 / (gPathFindingPct + 1.0)))) // if enough time for pathfinding to engage
    {   if (llVecMag(llGetVel()) < 0.01 && llVecMag(llGetOmega()) < 0.01)
        {   pathMsg(PATH_MSG_WARN, "Not moving after " + (string) (now-gPathActionStartTime) 
                + " secs. Vel: " + (string)(llVecMag(llGetVel())));
            //  Check to see if we can't get there from here.
            if (gPathPathmode == PATHMODE_PURSUE)
            {   list v = llGetObjectDetails(gPathTarget, [OBJECT_POS]); 
                if (v == []) { return(PATHSTALL_UNREACHABLE); }    // can't find target. Maybe it left.
                ////vector targetpos = llList2Vector(v,0);
                ////if (!pathReachableStatic(pos, targetpos))
                ////{   return(PATHSTALL_UNREACHABLE); }       // abandon pursuit, target is not reachable
            }
            ////if (gPathPathmode == PATHMODE_NAVIGATE_TO)
            ////{   if (!pathReachableStatic(pos, gPathGoal))
            ////    {   return(PU_FAILURE_UNREACHABLE); }   // destination is unreachable but we do not know why
            ////}
            //  Check for pursue fails to start. This usually means the destination is unreachable.
            if (gPathPathmode == PATHMODE_PURSUE) 
            {   vector pos = llGetPos();                // we are here
                pathMsg(PATH_MSG_WARN, "Pursuit not moving, started at " + (string)gPathRequestStartPos + " and now at " + (string)pos);
                if (llVecMag(gPathRequestStartPos - llGetPos()) < 0.01)
                {   return(PATHSTALL_UNREACHABLE); }           // pursuit will not start, don't try
                if (gPathRetries > 2)
                {   return(PATHSTALL_UNREACHABLE); }           // pursuit retry failed, give up
            }
            //  Retry for other operations. 
             if (gPathRetries == 0)                     // if have not retried
            {   gPathRetries++;              
                return(PATHSTALL_RETRY);                // just restart operation
            } else if (gPathRetries > 2)                // retrying is not helping
            {  return(PATHSTALL_UNREACHABLE);           // navigate retry failed, give up
            } else {                                    // tried that already         
                gPathRetries++;
                return(PATHSTALL_UNSTICK);              // force unstick
            }
        }
        ////gPathRetries = 0;                               // we are moving, reset stuck counter
    }
    //  Check for dynamically stalled, moving but not making progress. Usually means blocked by a new obstacle
    if ((llVecMag(pos - gPathLastGoodPos)) > PATH_GOOD_MOVE_DIST)   // if making some progress
    {   gPathLastGoodPos = pos;                     // we are good
        gPathLastGoodTime = now;
        gPathRetries = 0;                           // no need to retry
    } else {
        if ((now - gPathLastGoodTime) > PATH_GOOD_MOVE_TIME * (100.0 / (gPathFindingPct + 1.0))) // not getting anywhere, scaled by pathfinding slowdown
        {   pathMsg(PATH_MSG_WARN, pathActionName(gPathPathmode) + " moving but not making progress.");       
            if (gPathRetries == 0)                      // if have not retried
            {   gPathRetries++;              
                return(PATHSTALL_RETRY);                // just restart operation
            } else {                                    // tried that already
                gPathRetries++;                           
                return(PATHSTALL_UNSTICK);              // force unstick
            }
        }
    }
    return(PATHSTALL_NONE);                         // we are OK
}

integer pathAtGoal(float tolerance)                                // are we at the goal?
{   if (llVecMag(llGetVel()) < 0.01) { tolerance = 2*tolerance; } // if stopped, allow more error - pathfinding gave up
    //  At-goal check. Usually happens when start and goal are too close
    if (gPathPathmode == PATHMODE_NAVIGATE_TO)          // only if Navigate To
    {   float dist = llVecMag(llGetPos() - gPathGoal);  // distance to target
        if (dist <= tolerance) // if arrived.
        {   pathMsg(PATH_MSG_INFO, "At Navigate To goal, error distance: " + (string)dist);
            return(TRUE);                    // Happy ending
        } else {
            return(FALSE);
        }
    } else if (gPathPathmode == PATHMODE_PURSUE)
    {   float dist = llVecMag(llGetPos() - llList2Vector(llGetObjectDetails(gPathTarget, [OBJECT_POS]),0));
        if (dist <= tolerance)
        {   pathMsg(PATH_MSG_INFO, "At Pursuit goal, error distance: " + (string)dist);
            return(TRUE);                       // Happy ending
        } else {
            ////pathMsg(PATH_MSG_INFO, "Not at Pursuit goal, error distance: " + (string)dist + ", tol: " + (string)tolerance);
            return(FALSE);                      // not at goal
        }
    }
    return(TRUE);                               // not meaningful for this operation
}

//
//  pathUnstickFindPos -- find place to move when stuck
//
//  Returns ZERO_VECTOR if fail
//
vector pathUnstickFindPos(vector pos)
{
    integer i;
    vector wpos = pos;
    for (i=0; i<10; i++)                            // 10 tries
    {   wpos.x = pos.x + llFrand(2.0*PATH_RANDOM_UNSTICK_MOVE) - PATH_RANDOM_UNSTICK_MOVE;   // ***NEEDS TO BE SAFER ABOUT OBSTACLES***
        wpos.y = pos.y + llFrand(2.0*PATH_RANDOM_UNSTICK_MOVE) - PATH_RANDOM_UNSTICK_MOVE;   // 
        if (pathValidDest(wpos)) { return(wpos); }   // success if random pos looks OK
    }
    return(ZERO_VECTOR);                            // fail
 
}
//
//  pathUnstick  -- get stuck object moving again
//
//  Unstick steps:
//      1. Wander a small amount.
//      2. Do a small move with llPos.
//      3. Wander some more.  
//
integer pathUnstick()
{
    pathMsg(PATH_MSG_WARN,"Attempting recovery for " + pathActionName(gPathPathmode) + " - wandering briefly.");
    vector pos = llGetPos();                        // wander near current position
    ////if (pos == ZERO_VECTOR)
    ////{   pathMsg(PATH_MSG_ERROR, "Unable to find a navigatable place for unstick."); 
    ////    return(FALSE);
    ////}
    //  Have to disable avatar avoidance for this or it won't start when too close to an avi.
    llExecCharacterCmd(CHARACTER_CMD_STOP, []);
    llUpdateCharacter(pathReplaceOption(gPathCreateOptions, [CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS])); 
    llWanderWithin(pos, <2.0, 2.0, 1.0>,[]);        // move randomly to get unstuck
    llSleep(2.0);
    llExecCharacterCmd(CHARACTER_CMD_STOP, []);
    llSleep(1.0);
    vector newpos = llGetPos();
    llOwnerSay("Unstick: pos was " + (string) pos + " and now is " + (string) newpos); // ***TEMP***
    if (llVecMag(newpos - pos) < 0.001)             // if character did not move at all
    {   pathMsg(PATH_MSG_WARN,"Wandering recovery did not work.  Trying forced move."); // far too common.
        pos = pathUnstickFindPos(pos);              // try to find someplace to go by random search
        if (pos == ZERO_VECTOR)
        {   pathMsg(PATH_MSG_ERROR,"Stuck and no place to go"); return(FALSE); }
        llSetPos(pos);                              // force a small move.
        llSleep(2.0);                               // give physics time to settle if we collided
#ifdef OBSOLETE // don't wander here again,  try restarting the operation
        //  Try second wander. If it doesn't move, we're really stuck.
        llWanderWithin(pos, <2.0, 2.0, 1.0>,[]);    // move randomly to get unstuck
        llSleep(2.0);
        llExecCharacterCmd(CHARACTER_CMD_STOP, []);
        llSleep(1.0);
        if (llVecMag(llGetPos() - pos) < 0.01) // if character did not move
        {   pathMsg(PATH_MSG_ERROR,"Second recovery did not work. Help.");
            llUpdateCharacter(gPathCreateOptions);      // restore old state
            return(FALSE);  // failed
        }
#endif // OBSOLETE
    }
    llUpdateCharacter(gPathCreateOptions);      // restore old state
    return(TRUE);                               // successful recovery
}

//  
//  pathUpdate  -- Pathfinding system reports completion
//
pathUpdate(integer status, list reserved)
{
    if (gPathPathmode == PATHMODE_OFF)                  // idle, should not happen
    {   pathMsg(PATH_MSG_WARN, "Unexpected path_update call, status " + pathErrMsg(status));
        llSleep(2.0);                                   // avoid compute loop when stuck
        return;
    }
    if (status == PU_GOAL_REACHED)                      // may be at goal, or not.
    {   //  Pathfinding system thinks goal reached. 
        if (pathAtGoal(gPathTolerance)) 
        {   pathStop();
            pathUpdateCallback(PU_GOAL_REACHED,[]);     // tell user
            return;                                     // done
        } else {                                        // need to retry, not there yet.
            if (llVecMag(llGetPos() - gPathLastGoodPos) < 0.01) // if we are stuck
            {   pathMsg(PATH_MSG_WARN, "Pathfinding reported success, but goal not reached and stalled.");
                pathStop();                             // stop motion
                pathUpdateCallback(PATHSTALL_NOPROGRESS,[]);    // tell user
            } else {
                ////pathMsg(PATH_MSG_INFO, "Pathfinding reported success, but goal not reached. Retry.");
                pathActionRestart();                    // try again
            }
            return;
        }
        //  Not at goal,  
    } else if ((status == PU_EVADE_HIDDEN)
            || (status == PU_EVADE_SPOTTED) 
            || (status == PU_SLOWDOWN_DISTANCE_REACHED))
    {   pathMsg(PATH_MSG_INFO, "Misc. path update status report: " + pathErrMsg(status));
        pathUpdateCallback(status,[]);                  // tell user
        return;
    }
    //  Check for went off the allowed area
    if (status == PU_FAILURE_INVALID_START)                 // off the navmesh
    {   integer stat = pathRecoverOffNavmesh();             // try to recover
        if (stat == PU_GOAL_REACHED)                        // if successful recovery back in bounds    
        {   pathActionRestart();                            // try again
            return;
        } else if (stat == PU_FAILURE_INVALID_START) {
            pathMsg(PATH_MSG_INFO, "Path operation failed: " + pathErrMsg(status));
            pathStop();
            pathUpdateCallback(status,[]);                  // tell user       
        }      
    }
    vector pos = llGetPos();
    if (!pathValidDest(pos))                                // outside the parcel
    {   {   pathMsg(PATH_MSG_WARN,"Out of bounds, status " + pathErrMsg(status) + ", at " + (string)pos);
            integer stat = pathRecoverOutOfBounds(gPathLastSafePos);            // try to recover             
            if (stat == PU_GOAL_REACHED)                    // if successful recovery back in bounds    
            {   pathActionRestart();                        // try again
                return;
            }
        }          
    }
    //  Not done, and not a misc. status report. Fail.
    pathMsg(PATH_MSG_INFO, "Path operation failed: " + pathErrMsg(status));
    pathStop();
    pathUpdateCallback(status,[]);                                         // tell user
}


//
//  pathValidDest - is target valid (same sim and same parcel ownership as at start)
//
integer pathValidDest(vector pos) 
{   float REGION_SIZE = 256;                            // SL only, not for OpenSim
    if ((pos.x <= 0) || (pos.x >= REGION_SIZE) || (pos.y <= 0) || (pos.y >= REGION_SIZE)) { return(FALSE); }
    list theredata = llGetParcelDetails(pos, [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP, PARCEL_DETAILS_ID]);
    key therekey = llList2Key(theredata,2);             // key of this parcel
    if (therekey == gPathSafeParcelKey) { return(TRUE); }   // previously checked, OK
    //  New parcel, must make all the checks.
    integer thereflags = llGetParcelFlags(pos);         // flags for dest parcel
    key thereowner = llList2Key(theredata,0);
    key theregroup = llList2Key(theredata,1);
    if ((gPathSafeParcelOwner != thereowner)                 // dest parcel must have same ownership
    && (gPathSafeParcelGroup != theregroup))
    {   return(FALSE); } // different group and owner at dest parcel
    //  Check for no-script area
    if (thereflags & PARCEL_FLAG_ALLOW_SCRIPTS == 0)            // if scripts off for almost everybody
    {   if (gPathSafeParcelOwner != thereowner)
        {   if ((thereflags & PARCEL_FLAG_ALLOW_GROUP_SCRIPTS == 0) || (gPathSafeParcelGroup != theregroup))
            { return(FALSE); }                                  // would die
        }
    }                                // no script area, we would die
    //  Can we enter the destination parcel?
    if (thereflags && PARCEL_FLAG_ALLOW_ALL_OBJECT_ENTRY == 0) 
    {    if (gPathSafeParcelOwner == thereowner) { return(TRUE); } // same owner, OK
        {   if ((thereflags & PARCEL_FLAG_ALLOW_GROUP_OBJECT_ENTRY) || (gPathSafeParcelGroup != theregroup))
            { return(FALSE); }
        }
    }
    gPathSafeParcelKey = therekey;                          // this parcel is OK
    return(TRUE);  // OK to enter destination parcel
}
//
//  pathActionName -- name of path action in quotes, for messages
//
string pathActionName(integer pathmode)
{    return("\"" + llList2String(PATHMODE_NAMES, pathmode) + "\""); } // name in quotes
//
//  pathGetOption -- get option as list of 1 element, or empty list
//
list pathGetOption(list options, integer k)
{
    integer i;
    integer length = llGetListLength(options);      // length of options list
    for (i=0; i<length; i = i+2)                    // remove old value
    {   if (llList2Integer(options,i) == k) 
        {   
            return(llList2List(options, i+1,i+1));  // return matching item
        }
    }
    return([]);                                     // no find
}

//
//  pathReplaceOption  -- replace option in strided integer key/value list with new value
//
list pathReplaceOption(list options, list update)
{
    integer k = llList2Integer(update,0);           // the key
    integer i;
    integer length = llGetListLength(options);      // length of options list
    integer startpos = length;                      // add at end if no find
    integer endpos = startpos;                      // add at end if no find
    for (i=0; i<length; i = i+2)                    // remove old value
    {   if (llList2Integer(options,i) == k) 
        {   startpos = i;                               // replace here
            endpos = i+1;                               // replace here
        }
    }
    list outlist = llListReplaceList(options, update, startpos, endpos);
    return(outlist);                                // insert new value
}
#ifdef OBSOLETE
//
//  Useful utility functions. Optional.
//
//
//  pathList2String -- convert list to string with commas as delimiters. Useful for debug.
//
string pathList2String(list src)
{   string out = "[";               
    integer i;
    for (i=0; i<llGetListLength(src); i++)                  // for each listitem
    {   if (i>0) { out = out + ","; }
        out = out + (string)llList2List(src, i,i);          // append to string with comma separator
    }
    return(out + "]");                                      // and enclosing brackets
}

//
//  pathClosestNavPoint -- get closest nav point to current position.
//
//  This is mostly to find the point on the ground below the target.
//
vector pathClosestNavPoint(vector pos)
{
    vector pos = llGetPos();
    vector charsize = llGetScale();            
    float charaboveground = charsize.z * 0.5;   // must query close to ground level
    list points = llGetClosestNavPoint(pos - <0,0,charaboveground>, [GCNP_RADIUS, charsize.z]);   // get navmesh point under object.
    if (!llGetListLength(points))
    {   pathMsg(PATH_MSG_WARN, "No nav point close to " + (string)pos);
        return(ZERO_VECTOR);
    }
    vector pnt = llList2Vector(points,0);
    pathMsg(PATH_MSG_INFO, "Closest nav point to " + (string)pos + " is " + (string)pnt);
    return(pnt);    
}
//
//  pathReachableStatic  -- can we get there from here, as a static path?
//
//  Uses llGetStaticPath to test.
//
//  Too generous. Will consider off-parcel places reachable.
//
integer pathReachableStatic(vector pt0, vector pt1)
{   pt0 = pathClosestNavPoint(pt0);
    pt1 = pathClosestNavPoint(pt1);
    list path = llGetStaticPath(pt0, pt1, OUR_CHARACTER_RADIUS, []);
    integer listlength = llGetListLength(path);  // last item is error code
    if (listlength < 1)
    {   pathMsg(PATH_MSG_ERROR, "LSL INTERNAL ERROR: llGetStaticPath zero length list."); // broken
        return(FALSE);
    }
    ////pathMsg(PATH_MSG_INFO, "Path reachable: " + pathList2String(path)); // ***TEMP***
    integer status = llList2Integer(path,-1);       // get status
    if (status == 0) {  return(TRUE);   }           // sucesss, can at least try this move
    pathMsg(PATH_MSG_WARN, "No path from "          // failed
        + (string)pt0 + " to " + (string)pt1
        + " status " + pathErrMsg(status)); 
    return(FALSE);  
}

//
//  pathNoObstacleBetween  -- is there an obstacle in a straight line between two points?
//
//  Uses llGetStaticPath to test
//
//  Assumes we are near our own character for z-height purposes
//
integer pathNoObstacleBetween(vector targetpos, vector lookatpos)
{   targetpos = pathClosestNavPoint(targetpos);
    lookatpos = pathClosestNavPoint(lookatpos);
    list path = llGetStaticPath(targetpos, lookatpos, OUR_CHARACTER_RADIUS, []);
    integer listlength = llGetListLength(path);  // last item is error code
    if (listlength < 1)
    {   pathMsg(PATH_MSG_ERROR, "LSL INTERNAL ERROR: llGetStaticPath returned zero length list."); // broken
        return(FALSE);
    }
    integer status = llList2Integer(path,-1);    // get status
    if (status != 0)
    {   pathMsg(PATH_MSG_INFO, "Clear sightline check from "  
        + (string)targetpos + " to " + (string) lookatpos
        + " failed, status " + (string) status); 
        return(FALSE);  
    }
    pathMsg(PATH_MSG_INFO, "Clear sightline path has " + (string) (listlength-1) 
        + " components: " + (string)path); 
    if (listlength <= 3) { return(TRUE); }
    //  Longer path, but points may be collinear. Length check.
    integer i;
    float total = 0.0;
    for (i=0; i<listlength-2; i++)      // sum length of path components
    {   total = total + llVecMag(llList2Vector(path,i) -llList2Vector(path,i+1)); } 
    float overall = llVecMag(llList2Vector(path,0) - llList2Vector(path, listlength-2));
    pathMsg(PATH_MSG_INFO,"Direct path length: " + (string)overall + " Total path length: " + (string)total); // ***TEMP***
    if (total < (overall*1.1)) { return(TRUE); }    // if path almost straight, good
    return(FALSE);                      // no direct path available
}

float pathMin(float a, float b) { if (a<b) { return(a); } else { return(b); }} // usual min and max, avoiding name clashes
float pathMax(float a, float b) { if (a>b) { return(a); } else { return(b); }}
//
//  pathGetBoundingBoxWorld -- get object bounding box in world coordinates
//
//  LSL gives us the bounding box in object coordinates. We have to rotate it
//  and get the limits.  One vertex at a time.
//
list pathGetBoundingBoxWorld(key id)
{
    list info = llGetObjectDetails(id, [OBJECT_POS, OBJECT_ROT]) + llGetBoundingBox(id);
    vector pos = llList2Vector(info, 0);            // position in world coords
    rotation rot = llList2Rot(info, 1);             // rotation in world coords
    integer ix; 
    integer iy;
    integer iz;
    //  Convert each corner of the original bounding box to world coordinates.
    vector mincorner;                               // bounding box in world coords
    vector maxcorner;
    integer first = TRUE;                           // first time through
    for (ix=0; ix<2; ix++)                          // do all vertices of bounding box
    {   vector vx = llList2Vector(info, ix+2);
        for (iy=0; iy<2; iy++)
        {   vector vy = llList2Vector(info, iy+2);
            for (iz=0; iz<2; iz++)
            {   vector vz = llList2Vector(info, iz+2);
                vector pt = <vx.x, vy.y, vz.z>;     // one corner of the bounding box in obj coords
                vector ptworld = pt * rot + pos;    // in world coords
                if (first)
                {   mincorner = ptworld;
                    maxcorner = ptworld;
                    first = FALSE;
                }  else {
                    mincorner = <pathMin(mincorner.x,ptworld.x), pathMin(mincorner.y, ptworld.y), pathMin(mincorner.z, ptworld.z)>;
                    maxcorner = <pathMax(maxcorner.x,ptworld.x), pathMax(maxcorner.y, ptworld.y), pathMax(maxcorner.z, ptworld.z)>; 
                }
            }
        }
    }
    return([mincorner, maxcorner]);                 // min and max corners, in world coordinates
}
//
//  pathBoundingBoxOverlap -- true if bounding boxes overlap
//
integer pathBoundingBoxOverlap(key id0, key id1,  vector allowance)
{   list bb0 = pathGetBoundingBoxWorld(id0);
    list bb1 = pathGetBoundingBoxWorld(id1);
    if ((llGetListLength(bb0) != 2) || (llGetListLength(bb1) != 2))
    {   pathMsg(PATH_MSG_WARN, "Could not get bounding boxes.");
        return(TRUE);                                   // treat as overlap
    }
    llOwnerSay("Bounding box check: " + (string)bb0 + "   " + (string)bb1);
    vector bb0min = llList2Vector(bb0,0) + allowance;   // lower bound, plus allowance
    vector bb0max = llList2Vector(bb0,1);               // upper bound
    vector bb1min = llList2Vector(bb1,0) + allowance;   // lower bound, plus allowance
    vector bb1max = llList2Vector(bb1,1);               // upper bound
    if ((bb0min.x > bb1max.x) || (bb1min.x > bb0max.x)) { return(FALSE); } // no overlap
    if ((bb0min.y > bb1max.y) || (bb1min.y > bb0max.y)) { return(FALSE); } // no overlap
    if ((bb0min.z > bb1max.z) || (bb1min.z > bb0max.z)) { return(FALSE); } // no overlap
    return(TRUE);                           // overlap
}


//
//   pathLinearInterpolate  -- simple linear interpolation
//
float pathLinearInterpolate(float n1 , float n2 , float fract )
{   return n1 + ( (n2-n1) * fract );    } 

//
//  easeineaseout  --  interpolate from 0 to 1 with ease in and ease out using cubic Bezier.
//
//  ease = 0: no smoothing
//  ease = 0.5: reasonable smoothing
//
float easeineaseout(float ease, float fract)
{   float ym = pathLinearInterpolate( 0 , fract, fract );
    float yn = pathLinearInterpolate(fract , 1 , fract );
    float y = pathLinearInterpolate( ym , yn , fract);
    return(y);
}

//
//  pathFaceInDirection  --  face in desired direction
//
//  Uses llSetRot. OK to use on characters when no pathfinding operation is in progress.
//
pathFaceInDirection(vector lookdir)
{   float TURNRATE = 90*DEG_TO_RAD;                             // (deg/sec) turn rate
    lookdir.z = 0.0;                                            // rotate about XY axis only
    rotation endrot = llRotBetween(<1,0,0>,llVecNorm(lookdir)); // finish here
    rotation startrot = llGetRot();                             // starting from here
    float turntime = llFabs(llAngleBetween(startrot, endrot)) / TURNRATE;  // how much time to spend turning
    integer steps = llCeil(turntime/0.200 + 0.001);             // number of steps 
    integer i;
    for (i=0; i<= steps; i++)                                   // turn in 200ms steps, which is llSetRot delay
    {   float fract = ((float)i) / steps;                       // fraction of turn (0..1)
        float easefract = easeineaseout(PATH_ROTATION_EASE, fract); // smooth acceleration
        llSetRot(slerp(startrot, endrot, easefract));           // interpolate rotation
    }
}
#endif // OBSOLETE
