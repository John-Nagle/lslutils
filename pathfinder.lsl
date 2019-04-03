//
//  Pathfinding sub-script - does the actual pathfinding requests
//
//  Makes Second Life pathfinding work reliably.
//
//  Used as a standalone script in the pathfinding object.
//  The controlling script includes "pathcalls.lsl" and
//  calls the functions there.
//
//  This is in a separate script because the size of the recovery code has become so large.
//
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
//  Retrying will happen when necessary.
//
//
//  TODO:
//  1. Make sure link messages never get lost. If they do, we need a "tick" event to keep alive. 
//
#include "pathfinding.lsl"
//
//  Constants
//
float WORKINGTIMER = 2.0;                           // update ever 2 seconds when active
float OUR_CHARACTER_RADIUS = 0.125;                 // ***TEMP*** need to get from options
//
//
//  actionRequestRecv -- call from link_msg
//
integer actionRequestRecv(integer sender_num, integer num, string jsonstr, key id)
{   if (num != PATH_DIR_REQUEST) { return(FALSE); } // not ours to handle
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    list opts = llJson2List((string)id);            // the options
    list jsonlist = llJson2List(jsonstr);           // key/value pairs
    integer pathaction = PATHMODE_OFF;              // no action yet
    gPathTarget = NULL_KEY;                         // reset current state of pathfinding engine
    gPathDist = ZERO_VECTOR;
    gPathGoal = ZERO_VECTOR;
    gPathTolerance = PATH_GOAL_TOL;                 // default tolerance ***TEMP***
    integer i;    
    for (i=0; i<llGetListLength(jsonlist); i = i + 2)      // go through strided list
    {   string itemid = llList2String(jsonlist,i);           // which item in JSON
        if (itemid == "action") 
        { pathaction = llList2Integer(jsonlist,i+1);      // set action
        } else if (itemid == "target")
        { gPathTarget = llList2Key(jsonlist,i+1);
        } else if (itemid == "goal")
        { gPathGoal = (vector)llList2String(jsonlist,i+1); 
        } else if (itemid == "dist")
        { gPathDist = (vector)llList2String(jsonlist,i+1);  // set dist
        } else if (itemid == "msglev")
        { gPathMsgLevel = llList2Integer(jsonlist,i+1); // set verbosity level
        } else {
            llSay(DEBUG_CHANNEL, "Invalid pathfinding request msg key: '" + itemid + "' in " + jsonstr);   // BUG
            return(FALSE);                          // failed 
        }
    }
    pathActionStart(pathaction);                    // common function for all commands
    return(TRUE);                                   // handled
}

//  
//  setTimer -- set our timer based on state of pathfinding system
//
setTimer()
{
    if (gPathPathmode == PATHMODE_OFF)              // if no pathfinding in progress
    {   llSetTimerEvent(0.0); }                     // no timer
    else
    {   llSetTimerEvent(WORKINGTIMER); }            // timer event every few seconds
}

//
//  pathUpdateCallback -- pathfinding is done, tell requesting script
//
pathUpdateCallback(integer status, list unused)
{   ////llOwnerSay("pathfinder -> client status: " + (string)status);   // ***TEMP***
    llMessageLinked(LINK_THIS, PATH_DIR_REPLY, (string)status, NULL_KEY);  // status to client
    setTimer();
} 

//  The "main program" of this task.
//  Waits for incoming requests and handles them.
default {
    
    on_rez(integer rezinfo)
    {   llResetScript(); }                                  // just reset on init
    
    state_entry()
    {
        pathInit();                                         // initialize the pathfinding library
    }
    
    timer()
    {
        pathTick();                                         // pathfinding library timer
        if (gPathPathmode == PATHMODE_OFF)                  // if no pathfinding in progress
        {   llSetTimerEvent(0.0);                          // no need for a timer
            return; 
        }
        llMessageLinked(LINK_THIS, PATH_DIR_REPLY_TICK, "", NULL_KEY);  // keep alive        
    }
    
    link_message(integer sender_num, integer num, string str, key id)
    {   actionRequestRecv(sender_num, num, str, id); 
        setTimer();
    }
    
    path_update(integer type, list reserved)
    {   pathUpdate(type, reserved); }                       // send event to library
    
    collision_start(integer num_detected)
    {   
        pathCollide(num_detected);
    }
}  

