//
//
//  bhvcall.lsl -- interface to new NPC system for behaviors.
//
//  Animats
//  November, 2019
//
//  License: GPLv3.
//
//  A behavior is a script which, for a time, has control of an NPC.
//  Behaviors are scheduled by a scheduler. They are told when to
//  go and stop, and can be preempted by higher priority behaviors.
//
//  Behaviors use regular LSL calls to find out information about
//  the world, but must send messages to the scheduler to move 
//  or animate the NPC. This is to eliminate race conditions.
//
//  Behaviors must include this file.
//
//  ***MORE*** mostly unimplemented
//
//
#ifndef BVHCALLLSL
#define BVHCALLLSL
#include "npc/patherrors.lsl"
#include "debugmsg.lsl"
//
//
//  Constants
//
#define BHVMSGFROMSCH       1001                // from BVH scheduler, behaviors listen
#define BHVMSGTOSCH         1002                // to BVH scheduler, scheduler listens

//
//  Globals
//
integer gBhvMnum = -99999;                      // our "mnum", the link message number that identifies us to the scheduler
integer gBhvRegistered;                         // we are not registered yet
string gBhvThisScriptname;                      // name of this script
integer gBhvLinkNumber = -99999;                // link number of this prim, starts bogus
integer gBhvSchedLinkNumber = -99999;           // the scheduler's link number


//
//  Behavior API functions
//
//
//  bhvNavigateTo  -- go to point
//
//  Region corner (ZERO_VECTOR for current region) allows inter-region moves.
//  Stop short of "goal" by distance "stopshort"
//  Travel at "speed", up to 5m/sec, which is a fast run. 
//
bhvNavigateTo(vector regioncorner, vector goal, float stopshort, float speed)
{
    bvhpathreq(regioncorner, goal, NULL_KEY, stopshort, speed, ZERO_VECTOR);
}

//
//  bhvPursue -- pursue target avatar
//
bhvPursue(key target, float stopshort, float speed)
{
    bvhpathreq(ZERO_VECTOR, ZERO_VECTOR, target, stopshort, speed, ZERO_VECTOR);
}

//
//  bhvTurn -- turn self to face heading. Heading is a unit vector in XY plane.
//
bhvTurn(vector heading)
{
    bvhpathreq(ZERO_VECTOR, llGetRootPosition(), NULL_KEY, 0, 0.1, heading);
}

//
//  bhvAnimate -- run list of animations.
//
//  These only run when the avatar is stationary.
//  A new list replaces the old list, keeping any
//  anims already running still running without a restart
//
bhvAnimate(list anims)
{
}

//
//  bhvStop -- stop current movement
//
//  This is not usually necessary.
//  Sending a command while one is already running will stop 
//  the current movement, although not instantly.
//
bhvStop()
{   
}

//
//  bhvTick -- call every at least once a minute when running the path system.
//
//  This is used only for a stall timer.
//  Failure to call this will cause a script restart.
//
//
bhvTick()
{
    bhvreqreg();                                        // request registration if needed
}

//
//  bhvSetPriority -- set priority for this behavior
//
//  Larger numbers have higher priority.
//  Zero means this behavior does not want control now.
//
bhvSetPriority(integer priority)
{
    if (!gBhvRegistered)
    {   debugMsg(DEBUG_MSG_ERROR,"Set priority before registering"); return; }  // behavior error
    llMessageLinked(gBhvSchedLinkNumber,BHVMSGTOSCH, llList2Json(JSON_OBJECT,["request","priority","mnum",gBhvMnum,"pri",priority]),"");  // tell scheduler
}

//
//  bhvInit -- behavor is being initialized
//  
//  The behavior has restarted.
//
bhvInit()
{
    gBhvMnum = 0;                               // no mnum yet
    gBhvRegistered = FALSE;                     // not registered yet
    gBhvLinkNumber = llGetLinkNumber();         // my prim number, for msgs
    gBhvThisScriptname = llGetScriptName();     // name of this script
    gBhvSchedLinkNumber = -99999;               // we don't know this yet
    bhvreqreg();                                // try to register
}
//
//  Internal functions part of every behavior
//
//
//  bvhpathreq -- path request, via scheduler to path system
//
//  Handles NavigateTo, Pursue, and Turn.
//
bvhpathreq(vector regioncorner, vector goal, key target, float stopshort, float speed, vector finaldir)
{
    string jsn = llList2Json(JSON_OBJECT,["request","pathbegin","mnum",gBhvMnum,
        "target",target, "goal",goal, "stopshort",stopshort, "speed",speed, "finaldir", finaldir]);
    llMessageLinked(gBhvSchedLinkNumber, BHVMSGTOSCH,jsn,"");
}



bhvreqreg()
{   if (gBhvRegistered) { return; }             // already done
    //  Request register - this hooks us to the scheduler. Broadcast, but only at startup.
    llOwnerSay(gBhvThisScriptname + " requesting register");    // ***TEMP***
    llMessageLinked(LINK_SET, BHVMSGTOSCH, llList2Json(JSON_OBJECT,["request","register", "scriptname", gBhvThisScriptname, "linknum",gBhvLinkNumber]),"");
}

//
//  bhvregisterreply -- reply from register request
//
bhvregisterreply(string scriptname, integer mnum, integer schedlink)
{
    if (gBhvRegistered) { return; }                                         // already registered
    if (scriptname != gBhvThisScriptname) { return; }                       // not for us
    gBhvSchedLinkNumber = schedlink;                                        // link number of the scheduler
    gBhvMnum = mnum;                                                        // our assigned mnum
    gBhvRegistered = TRUE;                                                  // we are now registered
    llOwnerSay("Registered behavior #" + (string)mnum + ": " + gBhvThisScriptname); // ***TEMP***
}

//
//  bhvreqreply  -- some reply from scheduler on our channel
//
bhvreqreply(string jsn)
{
    llOwnerSay("Reply from scheduler: " + jsn);                             // ***TEMP***
    string reply = llJsonGetValue(jsn,["reply"]);
    string request = llJsonGetValue(jsn,["request"]);
    if (reply == "pathbegin")                               // movement completed
    {   integer status = (integer)llJsonGetValue(jsn,["status"]);
        key hitobj = (key)llJsonGetValue(jsn,["hitobj"]);
        bhvDoRequestDone(status, hitobj);
        return;
    }
    if (request == "start")
    {   bhvDoStart(); 
        return;
    }
    if (request == "stop")
    {   bhvDoStop();
        return;
    }
    debugMsg(DEBUG_MSG_ERROR,"Unexpected reply to behavior: " + jsn);   // unexpected
}

//
//  Callbacks - these must be implemented by the behavior task
//
//



//
//  bhvDoStart -- behavior has control
//
//  bhvDoStart()

//
//  bhvDoStop -- behavior no longer has control
//
//  bhvDoStop()

//
//  bhvRequestDone  -- request to scheduler completed
//
//  bhvDoRequestDone(string jsn)
//

//
//  Incoming events
//
//  Pass all incoming link messages intended for us to this.
//
bhvSchedMessage(integer num, string jsn)
{
    if (num == gBhvMnum)                            // intended for us
    {   bhvreqreply(jsn); 
        return;
    }
    if (num == BHVMSGFROMSCH)                       // if scheduler broadcast at startup
    {   string reptype = llJsonGetValue(jsn,["reply"]);
        string reqtype = llJsonGetValue(jsn,["request"]);
        if (reptype == "register")
        {   bhvregisterreply(llJsonGetValue(jsn,["scriptname"]), (integer)llJsonGetValue(jsn,["mnum"]), (integer)llJsonGetValue(jsn,["schedlink"])); 
            return;
        }
        if (reqtype == "reset")                     // scheduler reset, must reset behavior
        {   llResetScript(); }
    }
}
#endif // BVHCALLLSL

