//
//  pathavatartrack.lsl  
//  
//  Service task for pathfinding system users.
//  Task side.
//
//  Reports nearby avatars and keeps track of which ones 
//  have been served.
//
//  Split from patrollerdemo for space reasons.
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/pathbuildutils.lsl"
#include "npc/pathcall.lsl"
#include "npc/pathavatartrackcall.lsl"



//  Configuration
float IDLE_POLL = 10.0;
float MIN_MOVE_FOR_RETRY = 5.0;             // (m) minimum avatar move to trigger a retry
integer MIN_MEMORY = 3000;                  // trouble if memory this low
#ifndef VERBOSITY                           // define VERBOSITY to override
#define VERBOSITY PATH_MSG_ERROR            // verbose
#endif // VERBOSITY

//  Globals
key gOwner = NULL_KEY;                          // my owner
key gGroup = NULL_KEY;                          // my group
//
//  Targets in the sim.
//
list gDoneTargets = [];                         // we have said hello
list gDeferredTargets = [];                     // trouble with these, wait and retry
list gDeferredPositions = [];                   // don't retry until target moves

//
//  startup - initialization
//
startup()
{   gOwner = llGetOwner();                      // my owner
    list groupdetails = llGetObjectDetails(llGetKey(), [OBJECT_GROUP]); // my group
    gGroup = llList2Key(groupdetails,0);        // group of animesh

    llSetTimerEvent(IDLE_POLL);                                 // check for work
    llOwnerSay("Restart.");
}

//
//  startavatar -- start working on indicated avatar
//
//  The recipient doesn't have to do this.
//  We resend this every 10 seconds until they get around to it.
//
startavatar(key id)
{
    llMessageLinked(LINK_THIS, PATHAVATARTRACKREQUEST, llList2Json(JSON_OBJECT,["request","trackavi", "id", id]),"");
}
//
//  avatarcheck -- check local avatar situation
//
avatarcheck()
{
    float closestdist = 99999;              
    key target = NULL_KEY;                     
    integer targetix = -1;
    list newDoneTargets;
    //  Get all agents on same owner parcels in region.
    //  Don't do agent search if tight on memory.
    //  This will clear lists and start all avatar actions over, so
    //  it is an emergency measure.
    list agents;
    if (!pathneedmem(MIN_MEMORY))                      // if not tight on memory
    {   agents = llGetAgentList(AGENT_LIST_PARCEL_OWNER,[]);  
    } else {
        pathMsg(PATH_MSG_WARN,"Out of memory, clearing avatar list.");  // unlikely, but good backup
    }
    integer num_detected = llGetListLength(agents);
    if (num_detected == 0)                          // nobody around
    {
        gDoneTargets = [];                          // clear everything
        gDeferredTargets = [];
        gDeferredPositions = [];                      
        if (target != NULL_KEY)
        {                   
            pathMsg(PATH_MSG_INFO,"Forget old target.");
        } 
        target = NULL_KEY;      
        return;
    }
    //  Avatars are in the sim.
    integer i;
    for (i=0; i < num_detected; i++)        
    {   key id = llList2Key(agents,i);              // agent to examine  
        if (llListFindList(gDoneTargets,[id]) >= 0) // if on the greeted targets list
        {   newDoneTargets += id;                   // greeted target still in range
            id = NULL_KEY;                          // do not re-greet this target
        }
        integer j = -1;
        if (id != NULL_KEY)
        {   j = llListFindList(gDeferredTargets,[id]);} // check for avatar on deferred problem list
        if (j >= 0)
        {   //  Avatar on deferred problem list. Skip if deferred and hasn't moved.
            if (llVecMag(llList2Vector(gDeferredPositions, j) - target_pos(id)) < MIN_MOVE_FOR_RETRY)
            {   id = NULL_KEY; }                    // do not retry this one yet
        }
        if(id != NULL_KEY)
        {   if (!valid_dest(target_pos(id)))        // if in different parcel/out of region
            {   id = NULL_KEY;   }                  // ignore
        }
        //  We have a possible target. Find closest.   
        if (id != NULL_KEY)
        {   float dist = llVecMag(vec_to_target(id));
            if (dist > 0 && dist < closestdist) 
            {   target = id; 
                targetix = j;                       // which index                  
                closestdist = dist;
            }
        }
    }
    gDoneTargets = newDoneTargets;              // only keep greeted targets still in range
    if (target != NULL_KEY)                     // if there is an avatar to be dealt with
    {
        //  New target found. Go to them.
        startavatar(target);
        // Remove pursue target from to-do list.
        gDeferredTargets = llDeleteSubList(gDeferredTargets, targetix, targetix);
        gDeferredPositions = llDeleteSubList(gDeferredPositions, targetix, targetix);                   
    }
}

//
//  avatardone -- done with this target
//
//  Add to the "done" list.
//
avatardone(key id)
{   integer ix = llListFindList(gDoneTargets,[id]);     // look up in "done" list
    if (ix >= 0) { return; }                            // done
    gDoneTargets += [id];                               // add to "done" list.
    integer targetix = llListFindList(gDeferredTargets,[id]);         // if was on deferred list
    if (targetix < 0) { return; }                       // not on deferred lists
    //  Remove from deferred list. Shouldn't be on there anyway.
    gDeferredTargets = llDeleteSubList(gDeferredTargets, targetix, targetix);
    gDeferredPositions = llDeleteSubList(gDeferredPositions, targetix, targetix);                     
}
//
//  avatardefer -- defer action on this target
//
//  Add to the "defer" list.
//
avatardefer(key id)
{   integer ix = llListFindList(gDoneTargets,[id]);     // look up in "done" list
    if (ix >= 0) { return; }                            // if done, don't add to defer list
    vector avipos = target_pos(id);                     // get pos of target. Will retry when it moves.
    gDoneTargets += [id];
    integer targetix = llListFindList(gDeferredTargets,[id]);         // if was on deferred list
    if (targetix >= 0) { return; }                      // already on on deferred lists
    //  Add to deferred list
    gDeferredTargets += [id];
    gDeferredPositions += [avipos];                     // pos of avatar, not us
}
//
//  avatarbusy -- busy, don't bother me for a while
//
avatarbusy()
{
    //  Could extend the timer.
}


default
{
    on_rez(integer start_param)
    {
        llResetScript();
    }
 
    state_entry()
    {
        startup();
    }

    timer()                                             // timer tick
    {   
        avatarcheck();
    }
    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   //  Reply from behavior task is only message here.
        if (num != PATHAVATARTRACKREPLY) { return; }    // not ours
        string reply = llJsonGetValue(jsn,["reply"]);
        if (reply != "trackavi") { return; }            // not ours
        key id = (key) llJsonGetValue(jsn,["id"]);
        string action = llJsonGetValue(jsn,["action"]);
        gDebugMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]); // message level for debug msgs
        if (action == "done")                           // if done with avi
        {   avatardone(id); }                           // done with it
        else if (action == "defer")                     // defer until it moves
        {   avatardefer(id); }
        else if (action == "busy")                      // busy, don't bother me for a while
        {   avatarbusy(); }
        else
        {   pathMsg(PATH_MSG_ERROR,"Invalid msg: " + jsn); }
    }
    
    
}

