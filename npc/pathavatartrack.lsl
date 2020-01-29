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
#include "npc/mathutils.lsl"
#include "npc/pathavatartrackcall.lsl"


//  Configuration
#define IDLE_POLL (10.0)                        // (s) seconds between tests for avatar movement
#define MIN_MOVE_FOR_RETRY (5.0)                // (m) minimum avatar move to trigger a retry
#define MIN_MEMORY (3000)                       // (bytes) trouble if memory this low
#define MAX_VERT_DIST_TO_AVATAR (256.0)         // (b) Further than this and we assume an unreachable skybox
#define MIN_TIME_FOR_RETRY (15.0)               // (s) Min time for exponential backofff
#define EXPONENTIAL_BACKOFF (2.0)               // (no units) scale for exponential backoff

//
//  Targets in the sim.
//
list gDoneTargets = [];                         // we have said hello

//  These all have the same subscript
list gDeferredTargets = [];                     // trouble with these, wait and retry
////list gDeferredPositions = [];                   // don't retry until target moves
list gDeferredTimes = [];                       // time of next attempt
list gDeferredIntervals = [];                   // interval for time backoff

//
//  startup - initialization
//
startup()
{   pathinitutils();                                            // library init
    ////llSetTimerEvent(IDLE_POLL);                                 // check for work
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
    assert(gPathWidth > 0);                             // checks that initialization happened first
    float closestdist = 99999;              
    key target = NULL_KEY;                     
    list newDoneTargets;
    list newDeferredTargets;
    ////list newDeferredPositions;
    list newDeferredTimes;
    list newDeferredIntervals;
    //  Get all agents on same owner parcels in region.
    //  Don't do agent search if tight on memory.
    //  This will clear lists and start all avatar actions over, so
    //  it is an emergency measure.
    list agents;
    if (!pathneedmem(MIN_MEMORY))                      // if not tight on memory
    {   agents = llGetAgentList(AGENT_LIST_PARCEL_OWNER,[]);    // get all agents in sim  
    } else {
        debugMsg(DEBUG_MSG_WARN,"Out of memory, clearing avatar list.");  // unlikely, but good backup
    }
    integer num_detected = llGetListLength(agents);
    if (num_detected == 0)                          // nobody around
    {
        gDoneTargets = [];                          // clear everything
        gDeferredTargets = [];
        ////gDeferredPositions = [];
        gDeferredTimes = [];
        gDeferredIntervals = [];                     
        return;
    }
    //  Avatars are in the sim.
    vector pos = llGetRootPosition();                       // our location
    integer now = llGetUnixTime();                          // time now
    integer i;
    for (i=0; i < num_detected; i++)        
    {   key id = llList2Key(agents,i);                      // agent to examine
        vector tpos = target_pos(id);                       // position of avatar
        if (tpos != ZERO_VECTOR && (llFabs(pos.z - tpos.z) < MAX_VERT_DIST_TO_AVATAR)) // if sane position
        {   if (pathvaliddest(tpos))                        // if in same owner/group parcel, etc. and not in moving vehicle
            {                                               // avatar of interest. Do we need to greet it?
                integer doneix = llListFindList(gDoneTargets,[id]);     // on "done" list?
                if (doneix >= 0)
                {   newDoneTargets += id; }                 // keep on "done" list 
                else                                        // not done, see if time to do it.
                {           
                    integer deferix = llListFindList(gDeferredTargets,[id]);// check for avatar on deferred target list
                    if (deferix >= 0)                           // if on deferred list
                    {   //  Avatar on deferred problem list. Skip if deferred and hasn't moved.
                        vector tpos = target_pos(id);           // position of target
                        ////vector oldtpos = llList2Vector(gDeferredPositions, deferix); // position when deferred
                        integer oldtime = llList2Integer(gDeferredTimes, deferix);   // time of next retry even if no move
                        integer oldinterval = llList2Integer(gDeferredIntervals, deferix); // time of next retry
                        if (oldtime > now)                      // if timed out, can look again
                        {   newDeferredTargets += id;           // keep on deferred list
                            newDeferredTimes += oldtime;        // next retry time
                            newDeferredIntervals += oldinterval;  // same interval
                        } else {                                // delay time expired, can try again
                            debugMsg(DEBUG_MSG_WARN, "Can now retry " + llKey2Name(id) + " after " + (string)oldinterval + "s.");    // ***TEMP***
                            deferix = -1;                       // do not defer any more
                        }
                    }        
                    if(deferix < 0)                             // live target and not deferred
                    {   ////float dist = llVecMag(tpos-pos);        // pick closest target
                        float dist = pathdistance(pos, tpos, gPathWidth, gPathChartype); // pick closest reachable target
                        if (dist < 0.0)                         // not statically reachable
                        {   float interval = MIN_TIME_FOR_RETRY;// put on deferred list
                            newDeferredTargets += id;           // keep on deferred list
                            ////newDeferredPositions += oldtpos;    // along with its position at defer
                            newDeferredTimes += (now + interval);// next retry time
                            newDeferredIntervals += (interval);  // increase interval
                        
                        }
                        if (dist > 0 && dist < closestdist)      // if meaningful distance, keep closest
                        {   target = id; 
                            closestdist = dist;
                        }
                    }
                }
            }
        }
    }
    //  Update all lists to new values. 
    gDoneTargets = newDoneTargets;              // only keep greeted targets still in range
    gDeferredTargets = newDeferredTargets;
    ////gDeferredPositions = newDeferredPositions;
    gDeferredTimes = newDeferredTimes;
    gDeferredIntervals = newDeferredIntervals;  
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
    ////gDeferredPositions = llDeleteSubList(gDeferredPositions, targetix, targetix);
    gDeferredTimes = llDeleteSubList(gDeferredTimes, targetix, targetix);
    gDeferredIntervals = llDeleteSubList(gDeferredIntervals, targetix, targetix);
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
    integer now = llGetUnixTime();                      // time now
    integer interval = (integer)MIN_TIME_FOR_RETRY;     // default deferral period
    integer targetix = llListFindList(gDeferredTargets,[id]);         // if was on deferred list
    if (targetix >= 0)                                  // if already on list, boost time
    {   integer interval = llList2Integer(gDeferredIntervals,targetix);
        interval = (integer)(interval*EXPONENTIAL_BACKOFF); // increase interval for next time
        gDeferredTimes = llListReplaceList(gDeferredTimes,[now+interval],ix,ix);
        gDeferredIntervals = llListReplaceList(gDeferredIntervals,[interval],ix,ix);
    } else {                                            // if not on list, add it
        gDeferredTargets += [id];                       // add to deferred list
        gDeferredTimes += (now + interval);             // set next retry time
        gDeferredIntervals += interval;                 // set initial retry interval
    }
    debugMsg(DEBUG_MSG_WARN,"Defer " + llKey2Name(id) + " for " + (string)(interval)   + "s."); // ***TEMP***         
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
        {   debugMsg(DEBUG_MSG_INFO,"Rcvd: " + jsn);          // show incoming JSN
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
            {   debugMsg(DEBUG_MSG_ERROR,"Invalid msg: " + jsn); }
        
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn);                            // initialize params
            llSetTimerEvent(IDLE_POLL);                     // start checking for work
            llOwnerSay("Avatar track init");                // ***TEMP***
        } else if (num == DEBUG_MSGLEV_BROADCAST)           // set debug message level for this task
        {   debugMsgLevelSet(jsn);
        }
    }
    
    
}

