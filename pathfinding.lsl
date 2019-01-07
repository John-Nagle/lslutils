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
//  Globals internal to this module
//
integer gPathPathmode = PATHMODE_OFF;                       // what are we doing right now.  
key gPathTarget = NULL_KEY;                                 // target avatar, when pursuing
vector gPathGoal;                                           // goal when navigating
vector gPathDist;                                           // distance when wandering
list gPathOptions;                                          // options for all operations                                   

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
    gPathGoal = goal;                                       // wander around here
    gPathDist = dist;                                       // within this distance
    gPathOptions = options;                                 // using these options
    pathActionStart(PATHMODE_WANDER);                       // use common function
}

pathEvade(key target, list options)
{   gPathTarget = target;
    gPathOptions = options;
    pathActionStart(PATHMODE_EVADE);
}

pathFleeFrom(vector goal, vector dist, list options)
{   gPathGoal = goal;
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
pathActionStart(integer pathmode)
{
    gPathPathmode = pathmode;
    //  ***MORE*** get tolerance from options, set timer, start operation
}

//
//  pathActionRestart  -- restart whatever was running
//
pathActionRestart()
{
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
        pathError("pathActionRestart called incorrectly with pathmode = " + (string)pathmode);
    }
}

//
//  pathStallCheck  --  is pathfinding stalled?
//
//  Checks:
//      1. Zero velocity and zero angular rate - pathfinding quit on us.
//      2. No significant progress in last few seconds - pathfinding is stalled dynamically.
//      3. Stall timeout - entire operation took too long. Usual limit is about 2 minutes.
//
integer pathStallCheck()
{
    //  ***MORE***
}
