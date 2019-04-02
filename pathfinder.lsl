//
//  Pathfinding sub-script - does the actual pathfinding requests
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
//  Communicates with other scripts to do pathfinding tasks.
//
//  This is in a separate script because the size of the recovery code has become so large.
//
//  Works around Second Life bugs BUG-5021, BUG-226061.
//
//  Retrying will happen when necessary.
//
//  llNavigateTo and llPursue will always produce a callback to pathUpdateCallback when they are done or
//  have failed to reach their goal; this package provides timeouts, retries, and checks for them
// 
//  llWander, llEvade, and llFleeFrom do not have timeouts, retries, or checks, because they
//  have no well-defined goal. 
//
#include "pathfinding.lsl"
//
//  Internal functions
//
//
    //  ***NEED CREATE CHARACTER, UPDATE CHARACTER, PATH STOP***

//
//  pathActionRequestRecv -- call from link_msg
//
integer pathActionRequestRecv(integer sender_num, integer num, string jsonstr, key id)
{   if (num != PATH_DIR_REQUEST) { return(FALSE); } // not ours to handle
    if (gPathPathmode != PATHMODE_OFF) { pathStop(); }      // stop whatever we are doing
    string opts = llJson2List((string)id);          // the options
    list jsonlist = llJson2List(jsonstr);           // key/value pairs
    integer pathaction = PATHMODE_OFF;              // no action yet
    gPathTarget = NULL_KEY;                         // reset current state
    gPathDist = ZERO_VECTOR;
    gpathGoal = ZERO_VECTOR;
    integer i;    
    for (i=0; i<llListLength(json); i = i + 2)      // go through strided list
    {   string itemid = llList2String(i);           // which item in JSON
        if (itemid == "action") 
        { gPathPathmode = llList2Integer(i+1);      // set action
        } else if (itemid == "target")
        { gPathTarget = llList2Key(jsonlist,i+1);
        } else if (itemid == "goal")
        { gPathGoal = llList2Vector(jsonlist,i+1);  
        } else if (itemid == "dist")
        { gPathDist = llList2Vector(jsonlist,i+1);  // set dist
        } else {
            llSay(DEBUG_CHAN, "Invalid pathfinding request msg: " + jsonstr);   // BUG
            return(FALSE);                          // failed 
        }
    }
    pathActionStart(pathaction);                    // common function for all commands
    return(TRUE);                                   // handled
}


