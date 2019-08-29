//
//  pathcalls.lsl --  Pathfinding library external interface
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
//      llExecCharacterCmd(CHARACTER_CMD_STOP,[]) -> pathStop()
//
//  Also:
//      path_update -> pathUpdateCallback - define this for your own callback
//      In your path_update, call pathUpdate.
//
//      similarly, for
//      link_message -> pathLinkMsg
//      In your link_message, call pathLinkMsg
//
//  (llPatrol is not implemented. Suggest using llNavigateTo for each goal point.)
//
//  Call these functions instead of the corresponding LSL functions.
//
//  The work is all done in a separate script because the workarounds are big and
//  stack/heap collisions occur if everything is done in one script.
//
//  TODO: 
//      1. Add way to turn debugging messages on and off
//      2. Add detection for lost link messages. [DONE]
//
#include "pathdefs.lsl"
//
//  Compatibility with usage as a library.
//
pathInit() {}                               // initialization, nothing to do
pathUpdate(integer status, list reserved) {}// unneeded, backwards compat
pathCollide(integer num_detected) {}        // unneeded, backwards compat
//
//  Globals
//
integer PATHTICKTIMEOUT = 10;               // no reply in 10 secs will abort
integer gPathLastTick = 0;                  // last reply from pathfinder
integer gPathPathmode = PATHMODE_OFF;       // doing anything?

pathTick() 
{   if (gPathPathmode == PATHMODE_OFF) { return; }  // idle
    if (llGetUnixTime() - gPathLastTick > PATHTICKTIMEOUT)
    {   pathUpdateCallback(PATHSTALL_LINKMSGFAIL,[]);   // tell caller so they can do something
        pathMsg(PATH_MSG_ERROR,"Pathfinding task not responding"); // should not be happening
        gPathPathmode = PATHMODE_OFF;       // failed
    }
}

pathCreateCharacter(list options)
{   pathActionRequest(PATHMODE_CREATE_CHARACTER, NULL_KEY, ZERO_VECTOR, ZERO_VECTOR, options); }

pathUpdateCharacter(list options)
{   pathActionRequest(PATHMODE_UPDATE_CHARACTER, NULL_KEY, ZERO_VECTOR, ZERO_VECTOR, options); }

pathNavigateTo(vector goal, list options)
{   pathActionRequest(PATHMODE_NAVIGATE_TO, NULL_KEY, goal, ZERO_VECTOR, options); }

pathPursue(key target, list options)
{   pathActionRequest(PATHMODE_PURSUE, target, ZERO_VECTOR, ZERO_VECTOR, options); }
                
pathWander(vector goal, vector dist, list options)
{   pathActionRequest(PATHMODE_WANDER, NULL_KEY, goal, dist, options); }

pathEvade(key target, list options)
{   pathActionRequest(PATHMODE_EVADE, target, ZERO_VECTOR, ZERO_VECTOR, options); }

pathFleeFrom(vector goal, float distmag, list options)
{   pathActionRequest(PATHMODE_FLEE_FROM, NULL_KEY, goal, <distmag,0,0>, options); }

pathStop()
{   pathActionRequest(PATHMODE_OFF, NULL_KEY, ZERO_VECTOR, ZERO_VECTOR, []); }


//
//  Internal functions
//
//
//  pathActionRequest -- common function for all path actions.  
//
//  Sends message to script to do the work.
//  
pathActionRequest(integer action, key target, vector goal, vector dist, list options)
{
    string jsonopts = llList2Json(JSON_ARRAY, options);          // convert options to JSON
    string json = llList2Json(JSON_OBJECT, ["action", action, "target", target, "goal", goal, "dist", dist, "msglev", gPathMsgLevel]);
    llMessageLinked(LINK_THIS, PATH_DIR_REQUEST, json, jsonopts );  // send to worker script
    gPathLastTick = llGetUnixTime();                // reset the timeout
    gPathPathmode = action;                         // what we are doing
}

//
//  pathLinkMsg -- call from link_msg
//
integer pathLinkMsg(integer sender_num, integer num, string str, key id)
{   if (num == PATH_DIR_REPLY_TICK)                 // keep alive
    {   gPathLastTick = llGetUnixTime();            // time of last tick
        return(TRUE);
    }
    if (num == PATH_DIR_REPLY)                      // return status
    {   integer status = (integer)str;              // all that comes back is an integer status
        if (status != 0) { gPathPathmode = PATHMODE_OFF; }// we are now idle
        pathUpdateCallback(status,[]);              // all that comes back is an integer status
        return(TRUE);
    }
    return(FALSE);                                  // not handled
}
//
//  Utility functions. May change.
//
//
//  pathFaceInDirection  --  face in desired direction
//
//  Uses llSetRot. OK to use on characters when no pathfinding operation is in progress.
//
pathFaceInDirection(vector lookdir)
{   float TURNRATE = 90*DEG_TO_RAD;                             // (deg/sec) turn rate
    float   PATH_ROTATION_EASE = 0.5;                           // (0..1) Rotation ease-in/ease out strength
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
pathMsg(integer level, string msg)                              // print debug message
{   if (level > gPathMsgLevel) { return; }                      // ignore if suppressed
    llOwnerSay("Pathfinding: " + msg);                          // message
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


