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
#define IDLE_POLL (10.0)                        // (s) seconds between tests for avatar movement
#define MIN_MOVE_FOR_RETRY (5.0)                // (m) minimum avatar move to trigger a retry
#define MIN_MEMORY (3000)                       // (bytes) trouble if memory this low
#define MAX_VERT_DIST_TO_AVATAR (256.0)         // (b) Further than this and we assume an unreachable skybox

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
{   pathinitutils();                                            // library init
    llSetTimerEvent(IDLE_POLL);                                 // check for work
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
    list newDoneTargets;
    list newDeferredTargets;
    list newDeferredPositions;
    //  Get all agents on same owner parcels in region.
    //  Don't do agent search if tight on memory.
    //  This will clear lists and start all avatar actions over, so
    //  it is an emergency measure.
    list agents;
    if (!pathneedmem(MIN_MEMORY))                      // if not tight on memory
    {   agents = llGetAgentList(AGENT_LIST_PARCEL_OWNER,[]);    // get all agents in sim  
    } else {
        pathMsg(PATH_MSG_WARN,"Out of memory, clearing avatar list.");  // unlikely, but good backup
    }
    integer num_detected = llGetListLength(agents);
    if (num_detected == 0)                          // nobody around
    {
        gDoneTargets = [];                          // clear everything
        gDeferredTargets = [];
        gDeferredPositions = [];                      
        return;
    }
    //  Avatars are in the sim.
    vector pos = llGetPos();                                // our location
    integer i;
    for (i=0; i < num_detected; i++)        
    {   key id = llList2Key(agents,i);                      // agent to examine
        vector tpos = target_pos(id);                       // position of avatar
        if (tpos != ZERO_VECTOR && (llFabs(pos.z - tpos.z) < MAX_VERT_DIST_TO_AVATAR)) // if sane position
        {   if (pathvaliddest(tpos))                        // if in same owner/group parcel, etc.
            {                                               // avatar of interest. Do we need to greet it?
                integer doneix = llListFindList(gDoneTargets,[id]);     // on "done" list?
                if (doneix >= 0)
                {   newDoneTargets += id; }                 // keep on "done" list            
                integer deferix = llListFindList(gDeferredTargets,[id]);// check for avatar on deferred target list
                if (deferix >= 0)                           // if on deferred list
                {   //  Avatar on deferred problem list. Skip if deferred and hasn't moved.
                    vector tpos = target_pos(id);           // position of target
                    vector oldtpos = llList2Vector(gDeferredPositions, deferix); // position when deferred
                    if (llVecMag(oldtpos - tpos) < MIN_MOVE_FOR_RETRY) // if avi has not moved since deferral
                    {   pathMsg(PATH_MSG_INFO,"Do not retry " + llKey2Name(id) + " at " + (string)tpos + " was at " + (string)oldtpos); // ***TEMP***       
                        newDeferredTargets += id;           // keep on deferred list
                        newDeferredPositions += oldtpos;    // along with its position at defer
                    } else {
                        deferix = -1;                       // do not defer any more
                    }
                }           
                if(doneix < 0 && deferix < 0)               // live target and not deferred
                {   float dist = llVecMag(vec_to_target(id));   // pick closest target
                    if (dist > 0 && dist < closestdist) 
                    {   target = id; 
                        closestdist = dist;
                    }
                }
            }
        }
    }
    //  Update all lists to new values. 
    gDoneTargets = newDoneTargets;              // only keep greeted targets still in range
    gDeferredTargets = newDeferredTargets;
    gDeferredPositions = newDeferredPositions;  
    if (target != NULL_KEY)                     // if there is an avatar to be dealt with
    {
        //  New target found. Go to it.
        startavatar(target);                       
    }
}

//
//  avatardone -- done with this target
//
//  Add to the "done" list, and start the next request, if any.
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
    avatarcheck();                                      // and start up the next request                   
}
//
//  avatardefer -- defer action on this target
//
//  Add to the "defer" list, and start the next request, if any.
//
avatardefer(key id)
{   integer ix = llListFindList(gDoneTargets,[id]);     // look up in "done" list
    if (ix >= 0) { return; }                            // if done, don't add to defer list
    vector avipos = target_pos(id);                     // get pos of target. Will retry when it moves.
    integer targetix = llListFindList(gDeferredTargets,[id]);         // if was on deferred list
    if (targetix >= 0) { return; }                      // already on on deferred lists
    //  Add to deferred list
    gDeferredTargets += [id];
    gDeferredPositions += [avipos];                     // pos of avatar, not us
    avatarcheck();                                      // and start up the next request                   
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
    {   //  Reply from behavior task is main message here.
        if (num == PATHAVATARTRACKREPLY)                    // if ours
        {   pathMsg(PATH_MSG_INFO,"Rcvd: " + jsn);          // show incoming JSN
            string reply = llJsonGetValue(jsn,["reply"]);
            if (reply != "trackavi") { return; }            // not ours
            key id = (key) llJsonGetValue(jsn,["id"]);
            string action = llJsonGetValue(jsn,["action"]);
            if (action == "done")                           // if done with avi
            {   avatardone(id); }                           // done with it
            else if (action == "defer")                     // defer until it moves
            {   avatardefer(id); }
            else if (action == "busy")                      // busy, don't bother me for a while
            {   avatarbusy(); }
            else
            {   pathMsg(PATH_MSG_ERROR,"Invalid msg: " + jsn); }
        
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                              // initialize params
    }
    
    
}

