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
//
//  Configuration
//
integer PATH_STALL_TIME = 120;                          // (secs) really stalled if greater than this
integer PATH_START_TIME = 2;                            // (secs) allow this long for path operation to start
float   PATH_GOAL_TOL = 1.0;                            // (m) how close to dest is success?
float   PATH_GOOD_MOVE_DIST = 2.0;                      // (m) must move at least this far
integer PATH_GOOD_MOVE_TIME = 5;                        // (secs) this often
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
integer PATHSTALL_CANNOT_MOVE = -4;                         // can't move at all at current position
integer PATHSTALL_NOPROGRESS = -5;                          // not making progress, fail
integer PATHSTALL_UNSTICK = -6;                             // stuck, need to try an unstick
integer PATHSTALL_NOPURSUE = -7;                            // pursue did not start, unreachable				                
//
//  Error levels
//
integer PATH_MSG_ERROR = 0;
integer PATH_MSG_WARN = 1;
integer PATH_MSG_INFO = 2;
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
integer gPathMsgLevel = PATH_MSG_ERROR;                     // debug message level (set to change level)
vector gPathRequestStartPos;                                // position at start
vector gPathLastGoodPos;                                    // position of last good move
integer gPathLastGoodTime;                                  // time of last good move
integer gPathRetries;                                       // retries after zero speed
//
//  Call these functions instead of the corresponding LSL functions
//
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
    {   pathUpdateCallback(status,[]);                          // call user-defined completion fn
        return;
    }
    if (status == PATHSTALL_STALLED || status == PATHSTALL_NOPROGRESS || status == PATHSTALL_NOPURSUE) // total fail
    {   pathStop();                                             // stop operation in progress
        pathUpdateCallback(status,[]);                          // call user defined completion function
        return;
    }
    //  Possible recoverable problem
    if (status == PATHSTALL_UNSTICK)                            // if unstick needed
    {
        pathMsg(PATH_MSG_WARN, "Retrying path operation");
        integer success = pathUnstick();                        // try to unstick situation
        if (!success)
        {   pathStop();                                         // fails
            pathUpdateCallback(PATHSTALL_CANNOT_MOVE,[]);       // we are stuck
            return;
        }
    }
    //  Recoverable error, try again.
    pathActionRestart();         
}
//
//  Internal functions
//
pathMsg(integer level, string msg)                              // print debug message
{   if (level > gPathMsgLevel) { return; }                      // ignore if suppressed
    llOwnerSay("Pathfinding: " + msg);                          // message
}

pathStop()
{
    llExecCharacterCmd(CHARACTER_CMD_STOP,[]);                  // stop whatever is going oin
    gPathPathmode = PATHMODE_OFF;                               // not pathfinding
}

pathActionStart(integer pathmode)
{
    gPathPathmode = pathmode;
    pathMsg(PATH_MSG_INFO, "Starting pathfinding.");
    gPathRequestStartTime = llGetUnixTime();                    // start time of path operation
    gPathRetries = 0;                                           // no retries yet
    gPathRequestStartPos = llGetPos();                          // starting position
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
    {   pathMsg(PATH_MSG_INFO, "Starting llNavigateTo to " + (string)gPathGoal);
        llNavigateTo(gPathGoal, gPathOptions);                  // restart using stored values
    } else if (gPathPathmode == PATHMODE_PURSUE)
    {   pathMsg(PATH_MSG_INFO, "Starting llPursue of " + llKey2Name(gPathTarget));
        llPursue(gPathTarget, gPathOptions);
    } else if (gPathPathmode == PATHMODE_WANDER)
    {   pathMsg(PATH_MSG_INFO, "Starting llWander around " + (string)gPathGoal);
        llWanderWithin(gPathGoal, gPathDist, gPathOptions);
    } else if (gPathPathmode == PATHMODE_EVADE)
    {   pathMsg(PATH_MSG_INFO, "Starting llEvade of " + llKey2Name(gPathTarget));
        llEvade(gPathTarget, gPathOptions); 
    } else if (gPathPathmode == PATHMODE_FLEE_FROM)
    {   pathMsg(PATH_MSG_INFO, "Starting llFleeFrom " + (string)gPathGoal);
        llFleeFrom(gPathGoal, llVecMag(gPathDist), gPathOptions);
    } else {
        pathMsg(PATH_MSG_ERROR, "pathActionRestart called incorrectly with pathmode = " 
            + (string)gPathPathmode);
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
    if (gPathPathmode == PATHMODE_OFF) { return(PATHSTALL_NONE); } // idle, cannot stall
    if (gPathPathmode == PATHMODE_WANDER 
    || gPathPathmode == PATHMODE_EVADE 
    || gPathPathmode == PATHMODE_FLEE_FROM) { return(PATHSTALL_NONE); }  // no meaningful completion criterion 
    //  Stall timeout
    integer now = llGetUnixTime();                  // UNIX time since epoch, secs
    vector pos = llGetPos();
    if (now - gPathRequestStartTime > PATH_STALL_TIME)// if totally stalled
    {   pathStop();                                 // stop whatever is going on
        pathMsg(PATH_MSG_ERROR, "Request stalled after " + (string)PATH_STALL_TIME + " seconds."); 
        return(PATHSTALL_STALLED);
    }
    //  At-goal check. Usually happens when start and goal are too close
    if (pathAtGoal(gPathTolerance))                 // must be close to force an abort
    {   pathStop();                                 // request complete
        return(PU_GOAL_REACHED);                    // Happy ending
    }

    //  Zero velocity check
    if (now - gPathActionStartTime > PATH_START_TIME) // if enough time for pathfinding to engage
    {   if (llVecMag(llGetVel()) < 0.01 && llVecMag(llGetOmega()) < 0.01)
        {   pathMsg(PATH_MSG_WARN, "Not moving after " + (string) (now-gPathActionStartTime) 
                + " secs. Vel: " + (string)(llVecMag(llGetVel())));
            //  Check for pursue fails to start. This usually means the destination is unreachable.
            if (gPathPathmode == PATHMODE_PURSUE) 
            {   vector pos = llGetPos();                // we are here
                pathMsg(PATH_MSG_WARN, "Pursuit not moving, started at " + (string)gPathRequestStartPos + " and now at " + (string)pos);
                if (llVecMag(gPathRequestStartPos - llGetPos()) < 0.01)
                {   return(PATHSTALL_NOPURSUE); }           // pursuit will not start, don't try
            }
            if (gPathRetries == 0)                      // if have not retried
            {   gPathRetries++;              
                return(PATHSTALL_RETRY);                // just restart operation
            } else {                                    // tried that already
                gPathRetries = 0;                           
                return(PATHSTALL_UNSTICK);              // force unstick
            }
        }
        gPathRetries = 0;                           // we are moving
    }
    //  Dynamically stalled, moving but not making progress. Usually means blocked by a new obstacle
    if ((llVecMag(pos - gPathLastGoodPos)) > PATH_GOOD_MOVE_DIST)   // if making some progress
    {   gPathLastGoodPos = pos;                     // we are good
        gPathLastGoodTime = now;
        gPathRetries = 0;                           // no need to retry
    } else {
        if ((now - gPathLastGoodTime) > PATH_GOOD_MOVE_TIME) // not getting anywhere
        {   pathMsg(PATH_MSG_WARN, "Moving but not making progress.");       
            if (gPathRetries == 0)                      // if have not retried
            {   gPathRetries++;              
                return(PATHSTALL_RETRY);                // just restart operation
            } else {                                    // tried that already
                gPathRetries = 0;                           
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
//  pathUnstick  -- get stuck object moving again
//
//  Unstick steps:
//      1. Wander a small amount.
//      2. Do a small move with llPos.
//
integer pathUnstick()
{
    pathMsg(PATH_MSG_WARN,"Attempting recovery - wandering briefly.");
    vector pos = llGetPos();
    //  Have to disable avatar avoidance for this or it won't start when too close to an avi.
    llUpdateCharacter(pathReplaceOption(gPathCreateOptions, [CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS])); 
    llWanderWithin(pos, <2.0, 2.0, 1.0>,[]);    // move randomly to get unstuck
    llSleep(2.0);
    llExecCharacterCmd(CHARACTER_CMD_STOP, []);
    llSleep(1.0);
    vector newpos = llGetPos();
    llOwnerSay("Unstick: pos was " + (string) pos + " and now is " + (string) newpos); // ***TEMP***
    if (llVecMag(newpos - pos) < 0.001)             // if character did not move at all
    {   pathMsg(PATH_MSG_WARN,"Wandering recovery did not work.  Trying forced move.");
        pos.x = pos.x + 0.25;                       // ***NEEDS TO BE SAFER ABOUT OBSTACLES***
        pos.y = pos.y + 0.25;
        llSetPos(pos);                              // force a move.
        llSleep(2.0);                               // give physics time to settle if we collided
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
    }
    llUpdateCharacter(gPathCreateOptions);      // restore old state
    return(TRUE);                               // unable to move at all
}

//  
//  pathUpdate  -- Pathfinding system reports completion
//
pathUpdate(integer status, list reserved)
{
    if (gPathPathmode == PATHMODE_OFF)                  // idle, should not happen
    {   pathMsg(PATH_MSG_ERROR, "Unexpected path_update call, status " + (string)status);
        return;
    }
    if (status == PU_GOAL_REACHED)                      // may be at goal, or not.
    {   //  Pathfinding system thinks goal reached. 
        if (pathAtGoal(gPathTolerance)) 
        {   pathStop();
            pathUpdateCallback(PU_GOAL_REACHED,[]);     // tell user
            return;                                     // done
        } else {                                        // need to retry, not there yet.
            pathMsg(PATH_MSG_WARN, "Pathfinding reported success, but goal not reached. Retry.");
            pathActionRestart();                        // try again
        }
        //  Not at goal,  
    } else if (status == PU_EVADE_HIDDEN 
            || status == PU_EVADE_SPOTTED 
            || status == PU_SLOWDOWN_DISTANCE_REACHED)
    {   pathMsg(PATH_MSG_INFO, "Misc. path update status report: " + (string)status);
        pathUpdateCallback(status,[]);                  // tell user
        return;
    } 
    //  Not done, and not a misc. status report. Fail.
    pathMsg(PATH_MSG_WARN, "Path operation failed, status " + (string) status);
    pathStop();
    pathUpdateCallback(status,[]);                                         // tell user
}
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
    ////llOwnerSay("PathReplaceOptions: " +  pathList2String(options) + " -> " + pathList2String(outlist));    // ***TEMP***
    return(outlist);                                // insert new value
}
//
//  Useful utility functions. Optional.
//
//  pathErrMsg -- path error number to string.  Mostly for debug.
//
string pathErrMsg(integer patherr)
{   list patherrspos =[
    "Character is near current goal.",
    "Character has reached the goal.",
    "Character cannot navigate from the current location - e.g., the character is off the navmesh or too high above it.",
    "Goal is not on the navmesh and cannot be reached.",
    "Goal is no longer reachable for some reason - e.g., an obstacle blocks the path.",
    "Target (for llPursue or llEvade) can no longer be tracked - e.g., it left the region or is an avatar that is now more than about 30m outside the region.",
    "There's no good place for the character to go - e.g., it is patrolling and all the patrol points are now unreachable.",
    "An llEvade character thinks it has hidden from its pursuer.",
    "An llEvade character switches from hiding to running",
    "A fatal error reported to a character when there is no navmesh for the region. This usually indicates a server failure and users should file a bug report and include the time and region in which they received this message.",
    "Character entered a region with dynamic pathfinding disabled.",
    "A character failed to enter a parcel because it is not allowed to enter, e.g. because the parcel is already full or because object entry was disabled after the navmesh was baked."];

    list patherrsneg = [                // our additional errors
        "Error 0",                      // should not make it to user
        "No error",                     // should not make it to user
        "Retrying pathfinding", 
        "Stall timeout",
        "Cannot move from current position",
        "Not making progress",
        "Stuck, need unstick",
        "Unable to start Pursue operation"];
        
    if (patherr >= 0 && patherr < llGetListLength(patherrspos)) // positive numbers, defined by LL
    {   return(llList2String(patherrspos,patherr));
    } else if (-patherr < llGetListLength(patherrsneg))          // negative numbers, defined by us
    {    return(llList2String(patherrsneg, -patherr)); 
    } else {
        return("Unknown path error " + (string) patherr);       // some bogus number
    }
}
//
//  pathNoObstacleBetween  -- is there an obstacle in a straight line between two points?
//
//  Uses llGetStaticPath to test
//
//  Assumes we are near our own character for z-height purposes
//
integer pathNoObstacleBetween(vector targetpos, vector lookatpos)
{   vector charsize = llGetScale();             //
    float charaboveground = charsize.z * 0.5;   // must query at ground level
    vector pos = llGetPos();
    targetpos.z = pos.z - charaboveground;      // use base of character aa Z for test.
    lookatpos.z = targetpos.z;
    list path = llGetStaticPath(targetpos, lookatpos, OUR_CHARACTER_RADIUS, []);
    integer listlength = llGetListLength(path);  // last item is error code
    if (listlength < 1)
    {   pathMsg(PATH_MSG_ERROR, "LSL INTERNAL ERROR: llGetStaticPath returned zero length list."); // broken
        return(FALSE);
    }
    integer status = llList2Integer(path,0);    // get status
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
    if (llGetListLength(bb0) != 2 || llGetListLength(bb1) != 2)
    {   pathMsg(PATH_MSG_WARN, "Could not get bounding boxes.");
        return(TRUE);                                   // treat as overlap
    }
    ////llOwnerSay("Bounding box check: " + (string)bb0 + "   " + (string)bb1);
    vector bb0min = llList2Vector(bb0,0) + allowance;   // lower bound, plus allowance
    vector bb0max = llList2Vector(bb0,1);               // upper bound
    vector bb1min = llList2Vector(bb1,0) + allowance;   // lower bound, plus allowance
    vector bb1max = llList2Vector(bb1,1);               // upper bound
    if (bb0min.x > bb1max.x || bb1min.x > bb0max.x) { return(FALSE); } // no overlap
    if (bb0min.y > bb1max.y || bb1min.y > bb0max.y) { return(FALSE); } // no overlap
    if (bb0min.z > bb1max.z || bb1min.z > bb0max.z) { return(FALSE); } // no overlap
    return(TRUE);                           // overlap
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
    {   llSetRot(slerp(startrot, endrot, ((float)i)/steps)); }  // interpolate rotation
}
