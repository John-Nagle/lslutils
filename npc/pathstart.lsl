//
//
//  pathstart.lsl -- startup section of new pathfinding system
//
//  Called by user task which imports pathcall. Handles
//  startup and retry.
//
//  Animats
//  November, 2019
//
//  License: GPLv3.
//
#include "npc/patherrors.lsl"
#include "npc/pathbuildutils.lsl"
//

//  Constants
float REGION_SIZE = 256.0;                                      // size of a region  
float TESTSPACING = 0.33;                                       // (m) vertical spacing between ray casts

////#define PATHCALLSTALLTIME 300 ////120                                   // if stalled for 120 seconds, reset everything 
//
//  Globals
//  Character parameters
float gPathcallSpeed = 1.0;                                     // speed defaults are very slow
float gPathcallTurnspeed = 0.1;
//  Retry state
float gPathcallLastDistance = 0.0;                              // distance to goal on last try
list  gPathcallLastParams = [];                                 // params at last try

integer gPathcallReset = FALSE;                                 // have we reset the system since startup?
integer gPathcallStarttime = 0;                                 // time last command started

integer gLocalPathId = 0;                                       // current path serial number
integer gLocalRequestId = 0;                                    // current request from user serial number

//
//  pathLinkMsg -- reply coming back from path execution
//
pathLinkMsg(string jsn, key hitobj)
{   
    integer status = (integer)llJsonGetValue(jsn, ["status"]);  // status and pathid via JSON.
    integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);  // hitobj as key param in link message
    if (pathid != gLocalPathId)                         // result from a cancelled operation
    {   pathMsg(PATH_MSG_WARN, "Stale path completed msg discarded."); return; }
    pathMsg(PATH_MSG_WARN,"Path complete, status " + (string)status + " Time: " + (string)llGetTime());
    if (pathretry(status, hitobj)) { return; }          // attempt a retry
    gPathcallStarttime = 0;                             // really done, stop clock
    pathUpdateCallback(status, hitobj);
}

//
//  pathprepstart -- start the path planner task
//
pathprepstart(key target, vector goal, float width, float height, float stopshort, integer chartype, float testspacing, integer pathid)
{   pathMsg(PATH_MSG_INFO,"Path plan start req, pathid: " + (string)pathid);
    string params = llList2Json(JSON_OBJECT, 
        ["target",target, "goal", goal, "stopshort", stopshort, "width", width, "height", height, "chartype", chartype, "testspacing", testspacing,
        "speed", gPathcallSpeed, "turnspeed", gPathcallTurnspeed,
        "pathid", pathid, "msglev", gPathMsgLevel]);
    llMessageLinked(LINK_THIS, PATHPLANREQUEST, params,"");   // send to planner  
}
//
//  pathbegin -- go to indicated point or target. Internal fn.
//
//  Go to the indicated location, in the current region, avoiding obstacles.
//
//  Stop short of the target by the distance stopshort. This can be zero. 
//
pathbegin(key target, vector endpos, float stopshort, integer dogged, integer requestid)
{
    gPathcallLastDistance = INFINITY;                               // must get shorter on each retry
    gLocalRequestId = requestid;                                    // save request ID for callbacks
    pathstart(target, endpos, stopshort, dogged);                   // do it
}
//
//  pathstart -- go to indicated point or target. Internal fn. Used by begin or restart
//
//  Go to the indicated location, in the current region, avoiding obstacles.
//
//  Stop short of the target by the distance stopshort. This can be zero. 
//
pathstart(key target, vector endpos, float stopshort, integer dogged)
{   assert(gPathWidth > 0);                                         // we must be initialized
    gPathcallLastParams = [];                                       // no params for retry stored yet.
    gLocalPathId = (gLocalPathId+1)%(PATHMAXUNSIGNED-1);// our serial number, nonnegative
    if (target != NULL_KEY)                                         // if chasing a target
    {   list details = llGetObjectDetails(target, [OBJECT_POS]);    // get object position
        if (details == [])                                          // target has disappeared
        {   pathdonereply(PATHEXETARGETGONE,target,gLocalPathId);  // send message to self and quit
            return;
        }
        endpos = llList2Vector(details,0);                          // use this endpos
        llOwnerSay("Position of target " + llKey2Name(target) + " is " + (string)endpos); // ***TEMP***
    }
    gPathcallLastParams = [target, endpos, stopshort, dogged];      // save params for restart
    //  Find walkable under avatar. Look straight down. Startpos must be on ground.
    if (!pathvaliddest(endpos))
    {   pathMsg(PATH_MSG_WARN,"Destination " + (string)endpos + " not allowed."); 
        pathdonereply(PATHEXEBADDEST,NULL_KEY,gLocalPathId);         // send message to self to report error
        return; 
    }

    float newz = pathfindwalkable(endpos, 0.0, gPathHeight*3);             // find walkable below char
    if (newz < 0)
    {   pathMsg(PATH_MSG_WARN,"Error looking for walkable under goal."); 
        pathdonereply(PATHEXEBADDEST,NULL_KEY,gLocalPathId);         // send message to self to report error
        return; 
    }
    endpos.z = newz;                                                // use ground level found by ray cast
    //  Generate path
    pathprepstart(target, endpos, gPathWidth, gPathHeight, stopshort, gPathChartype, TESTSPACING, gLocalPathId);
    gPathcallStarttime = llGetUnixTime();                               // timestamp we started
    //  Output from pathcheckobstacles is via callbacks
}
//
//  pathretry -- check if path can be retried
//
integer pathretry(integer status, key hitobj)
{
    if (llListFindList(PATHRETRYABLES, [status]) < 0) { return(FALSE); }// not retryable
    if (gPathcallLastParams == []) { return(FALSE); }                   // no retry params
    key target = llList2Key(gPathcallLastParams,0);                     // get retry params back
    vector endpos = llList2Vector(gPathcallLastParams,1);               // this language needs structures
    float shortstop = llList2Float(gPathcallLastParams, 2);
    integer dogged = llList2Integer(gPathcallLastParams,3);
    gPathcallLastParams = [];                                           // consume retry params to prevent loop
    float dist = pathdistance(llGetPos(), endpos, gPathWidth, CHARACTER_TYPE_A);  // measure distance to goal at gnd level
    if (dist < 0 || dist >= gPathcallLastDistance)                      // check for closer. negative means path distance failed. Avoids loop.
    {   if (!dogged || status != PATHEXETARGETMOVED)                    // dogged pursue mode, keep trying even if not getting closer  
        {   pathMsg(PATH_MSG_WARN, "No retry, did not get closer. Distance " + (string)dist + "m."); return(FALSE); }   // cannot retry  
    }
    if (gPathcallLastDistance > dist) { gPathcallLastDistance = dist; } // use new value if smaller, so the initial value of infinity gets reduced
    //  Must do a retry
    pathMsg(PATH_MSG_WARN, "Distance " + (string)dist + "m. Retrying...");                              // retry
    pathstart(target, endpos, shortstop, dogged);                       // trying again
    return(TRUE);                                                       // doing retry, do not tell user we are done.
}

//
//  pathUpdateCallback -- pass callback info back to calling user program
//
pathUpdateCallback(integer status, key hitobj)
{   
    llMessageLinked(LINK_THIS,PATHSTARTREPLY,
        llList2Json(JSON_OBJECT,["reply","pathbegin","requestid", gLocalRequestId, "status",status,"hitobj",hitobj]),""); 
}
//
//  Main program
//
default
{
    state_entry()
    {   pathinitutils();                        // library init
    }

    //
    //  Incoming link message - will be a plan job request, stop, or speed set
    //    
    link_message(integer status, integer num, string jsn, key id)
    {   if (num == PATHSTARTREQUEST)                                     // if request for a planning job
        {   pathMsg(PATH_MSG_WARN,"Path request (" + (string)num +"): " + jsn);
            integer requestid = (integer)llJsonGetValue(jsn,["requestid"]); // caller controls the path serial number
            string request = llJsonGetValue(jsn,["request"]);           // get request type
            if (request == "pathbegin")                                 // common start function
            {   vector goal = (vector)llJsonGetValue(jsn,["goal"]);     // get goal point
                key target = (key)llJsonGetValue(jsn,["target"]);  // get target if pursue
                float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
                integer dogged = (integer)llJsonGetValue(jsn,["dogged"]);
                pathbegin(target, goal, stopshort,  dogged, requestid);    // start movement
            } else if (request == "pathstop")
            {   pathbegin(NULL_KEY,llGetPos(),100.0, FALSE, requestid);    // a stop is a dummy move to self
            } else if (request == "pathspeed")
            {   gPathcallSpeed = (float)llJsonGetValue(jsn,["speed"]);
                gPathcallTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]);
            } else {
                panic("Unknown path request: " + jsn);                  // unlikely
            }
        } else if (num == PATHPLANREPLY)                                // reply from planner
        {   pathLinkMsg(jsn,id);                                        // will retry, or will reply to user
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                                      // initialize params
    }
}
             





