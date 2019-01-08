//
//  Pathfinding library
//
//  IN PROGRESS - DO NOT USE YET
//
//  Makes Second Life pathfinding work reliably.
//
//  Animats
//  January, 2019
//  License: GPL
//
//  Adds checking and retry, which is essential to getting anything done reliably.
//
//  Works around Second Life bugs BUG-5021, BUG-226061.
//
//  To use this, change your code as follows:
//
//      llNavigateTo -> pathNavigateTo
//      llPursue -> pathPursue
//      llWander -> pathWander
//      llEvade -> pathEvade
//      llFlee -> pathFlee
//
//      path_update -> pathUpdate
//
//      (llPatrol is not implemented. Suggest using llNavigateTo for each goal point.)
//
//  and call pathTick from a timer or sensor at least once every 2 seconds when pathfinding.
//
//  Retrying will happen when necessary.  
//
//
//  Configuration
//
integer PATH_STALL_TIME = 120;                          // really stalled if greater than this
integer PATH_START_TIME = 2;                            // allow this long for path operation to start
//
//  Constants
//
//  Pathmode - what we're doing now.
integer PATHMODE_OFF = 0;                                   // what we're doing now
integer PATHMODE_NAVIGATE_TO = 1;
integer PATHMODE_PURSUE = 2;
integer PATHMODE_WANDER = 3;
integer PATHMODE_EVADE = 4;
integer PATHMODE_FLEE_FROM = 5;
//
//  Path statuses.  Also, the LSL PU_* pathfinding statuses are used
integer PATHSTALL_NONE = -1;                                // not stalled, keep going
integer PATHSTALL_RETRY = -2;                               // unstick and retry
integer PATHSTALL_STALLED = -3;                             // failed, despite retries
//
//  Error levels
//
integer PATH_MSG_ERROR = 0;
integer PATH_MSG_WARN = 1;
integer PATH_MSG_INFO = 2;
//
//  Globals internal to this module
//
integer gPathPathmode = PATHMODE_OFF;                       // what are we doing right now.  
key gPathTarget = NULL_KEY;                                 // target avatar, when pursuing
vector gPathGoal;                                           // goal when navigating
vector gPathDist;                                           // distance when wandering
list gPathOptions;                                          // options for all operations
floag gPathTolerance;                                       // how close to end is success 
integer gPathRequestStartTime;                              // UNIX time path operation started
integer gPathActionStartTime;                               // UNIX time path operation started
integer gPathMsgLevel = PATH_MSG_WARN;                      // debug message level                                    

//
//  Call these functions instead of the corresponding LSL functions
//
pathNavigateTo(vector goal, list options)
{   if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathGoal = goal;                                       // head here
    gPathOptions = options;                                 // using these options
    pathActionStart(PATHMODE_NAVIGATE_TO);                  // use common function
}

pathPursue(key target, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathTarget = target;
    gPathOptions = options;
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

pathFleeFrom(vector goal, vector dist, list options)
{
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    gPathGoal = goal;
    gPathDist = dist;
    gPathOptions = options;
    pathActionStart(PATHMODE_FLEE_FROM);                        // use common fn
}
//
//  pathInit -- call this at startup/rez to initialize the variables
//
pathInit()
{
    gPathPathmode = PATHMODE_OFF;
    gPathTarget = NULL_KEY;
}
//
//  pathTick --  call this at least once every 2 seconds.
//
pathTick()
{
    if (gPathPathmode == PATHMODE_OFF) { return; }              // not pathfinding, nothing to do.
    integer status = pathStallCheck();                          // are we stuck?
    if (status == PATHSTALL_NONE) { return; }                   // no problem, keep going
    if (status == PU_GOAL_REACHED)                              // success
    {   pathStatus(status);                                     // call user-defined completion fn
        return;
    }
    if (status == PATHSTALL_STALLED)                            // total fail
    {   pathStop();                                             // stop operation in progress
        pathStatus(status);                                     // call user defined completion function
        return;
    }
    //  Possible recoverable problem
    //  ***MORE***
    pathUnstick();                                              // try to unstick situation
    pathActionRestart();                                        // and restart
}
//
//  Internal functions
//
pathMsg(integer level, string msg)
{   if (level > gPathMsgLevel) { return; }                      // ignore if suppressed
    llOwnerSay("Pathfinding: " + msg);                          // message
}
pathActionStart(integer pathmode)
{
    gPathPathmode = pathmode;
    //  ***MORE*** get tolerance from options, set timer, start operation
    gPathRequestStartTime = llGetUnixTime();                    // start time of path operation
}

//
//  pathActionRestart  -- restart whatever was running
//
pathActionRestart()
{
    gPathActionStartTime = llGetUnixTime();                     // start timing
    if (gPathPathmode == PATHMODE_NAVIGATE_TO)                  // depending on operation in progress
    {   llNavigateTo(gPathGoal, gPathOptions);                  // restart using stored values
    } else if (gPathPathmode == PATHMODE_PURSUE)
    {   llPursue(gPathTarget, gPathOptions);
    } else if (gPathPathmode == PATHMODE_WANDER)
    {   llWanderWithin(gPathGoal, gPathDist, gPathOptions);
    } else if {gPathPathmode == PATHMODE_EVADE)
    {   llEvade(gPathTarget, gPathOptions); }
    } else if (gPathPathmode == PATHMODE_FLEE_FROM)
    {   llFleeFrom(gPathGoal, gPathDist, gPathOptions);
    } else {
        pathMsg(PATH_MSG_ERROR, "pathActionRestart called incorrectly with pathmode = " + (string)pathmode);
    }
}

//
//  pathStallCheck  --  is pathfinding stalled?
//
//  Checks:
//      1. Stall timeout - entire operation took too long. Usual limit is about 2 minutes.
//      2. We're at the goal, so stop.
//      3. Zero velocity and zero angular rate - pathfinding quit on us.
//      4. No significant progress in last few seconds - pathfinding is stalled dynamically.
//
integer pathStallCheck()
{
    if (gPathPathmode == PATHMODE_NONE) { return(FALSE); } // idle, cannot stall
    //  Stall timeout
    integer now = llGetUnixTime();                  // UNIX time since epoch, sects
    if (now - gPathRequestStarTime > PATH_STALL_TIME)// if totally stalled
    {   pathStop();                                 // stop whatever is going on
        pathMsg(PATH_MSG_ERROR, "Request stalled after " + (string)PATH_STALL_TIME + " seconds."); 
        return(PATHSTALL_STALLED);
    }
    //  At goal check. Usually happens when start and goal are too close
    if (gPathPathmode == PATHMODE_NAVIGATE_TO)          // only if Navigate To
    {   if (llVecMag(llGetPos() - gPathGoal) < gGoalTolerance) // if arrived.
        {   pathMsg(PATH_MSG_WARNING, "At goal.");
            return(PU_GOAL_REACHED);                    // Happy ending
        }
    }

    //  Zero velocity check
    if (now - gPathActionStartTime) > PATH_START_TIME) // if enough time for pathfinding to engage
    {   if llVecMag(llGetVel() < 0.01 && llVecMag(llGetOmega()) < 0.01)
        {   // Not moving
            pathMsg(PATH_MSG_WARNING, "Not moving");
            return(PATHSTALL_RETRY);                // retry needed
        }
    }
    //  ***MORE*** need check for dynamically stalled, moving but not making progress
    return(PATHSTALL_NONE);                         // we are OK
}
