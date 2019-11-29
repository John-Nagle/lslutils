//
//  bhvscheduler.lsl --- Behavior scheduler for NPCs.
//
//  Animats
//  November, 2019
//  License: GPLv3.
//
//
//  This is the scheduler for the behavior tasks of Animats NPCs.
//  One behavior runs at a time, the highest priority one.
//  This task controls those behavior tasks.
//
//  Tasks do all movement and animation by sending messages to this
//  task, to avoid race conditions.
//
//  Startup
//
//  At startup, this task sends a broadcast message to all
//  scripts in all prims, asking behaviors to register. 
//  Behaviors send a message back, with their script name,
//  and prim number. The scheduler registers them, 
//  assigns them a "num" value for messages, and 
//  replies to the register message to the correct prim
//  on a common channel. The behavior is then live,
//  but stopped and at zero priority. It can then
//  request a run priority from the scheduler.
//
//  Run
//
//  When the scheduler selects a behavior to run, it sends
//  a "run" message to the desired prim and num value to enter
//  run mode. This includes a serial number of the run cycle,
//  to prevent race conditions. The behavior than has control
//  and can make path planning and animation requests by sending
//  messages to the scheduler. These include the "num" value and
//  the run cycle number. Those have to match the scheduler's
//  currently active behavior, or the request will be rejected.
//  This prevents race conditions during behavior switching.
//  Rejected requests produce a "stop" message from scheduler to
//  behavior.
//
//  Stop
//
//  A stop message to a behavior may indicate that it has been
//  preempted by a higher priority behavior. The behavior can
//  no longer have movement requests executed. But it can still
//  compute and sense the world. When a behavior is done
//  and no longer needs to run, it can request priority 0,
//  which will yield a "stop" message, and cause the scheduler
//  to find something else to do.
//
//  
//  ***MORE***

//
//  Constants
//
#define BHVBEHAVIORSSTRIDE 4    // stride of behavior list
#define BHVOFFPRIORITY  0       // do not run if at this priority
#define BHVMSGNUMSTART -1999    // link messages for scripts start here
//
//  Globals
//
list gBehaviors;                // [scriptname,primnum,msgnum,priority]
integer gBehaviorMsgnum = BHVMSGNUMSTART; // msg number for comm with behavior

integer gActiveBehavior = -1;   // index into active behavior table 
integer gActivePriority = 0;    // priority of active behavior
integer gActiveToken = 0;       // sequence number for activity

//
//  registerbehavior -- register a new behavior script
//
registerbehavior(string scriptname, integer primnum)
{
    if (llListFindList(gBehaviors, [scriptname]) >= 0)  // if dup, ignore
    {   return; }
    //  Add new behavior to list
    gBehaviors += [scriptname, primnum, gBehaviorMsgnum, BHVOFFPRIORITY];   // strided entry
    debugMsg(DEBUG_MSG_WARN,"New behavior #" + (string) gBehaviorMsgnum + ": " + scriptname);
    gBehaviorMsgnum++;
}

//
//  findbehavior -- find behavior by msgnum
//
integer findbehavior(integer num)
{
    integer i;
    for (i=0; i<llGetListLength(gBehaviors); i += BHVBEHAVIORSSTRIDE)
    {   if (llList2Integer(gBehaviors,i+2) == num)
        {   return(i); }                            // index into strided list
    }
    return(-1);                                     // no find  
}
//
//  setpriority -- set priority of a behavior
//
setpriority(integer num, integer pri)
{   assert(pri >= 0);
    integer ix = findbehavior(num);                 // index of entry
    gBehaviors = llListReplaceList(gBehaviors,[pri],ix+3,ix+3); // replace priority
    if (priority > gActivePriority)                 // this will start a new task
    {
        schedbvh();                                 // run the scheduler
    }
}

//
//  getbhvtorun -- get highest priority behavior. 
//
//  Inefficient, but probably not worth optimizing; this is called maybe several times
//  a minute.
//
//  Returns strided index into behaviors or -1.
//
integer getbvhtorun()
{
    integer ix;                                 // index into strided list
    integer pri = BHVOFFPRIORITY + 1;           // min priority of interest
    //  Get list of behaviors at winning priority
    list bhvindexes = [];                       // ones at highest priority
    for (i=0; i<llGetListLength(gBehaviors); i += BHVBEHAVIORSSTRIDE)
    {   integer ixpri = llList2Integer(gBehaviors,ix+3); // priority at this index
        if (ixpri > pri)                        // if new highest priority
        {   bvhindexes = [ix]; pri = ixpri; }   // restart list
        else if (ixpri == pri)                  // if same as higest priority
        {   bvhindexes += ix; }                 // add to list
    }
    //  Pass 3 - pick one at random from list
    if(llGetListLength(bvhindexes) <= 0) { return(-1); } // nothing ready to run
    return(llList2Integer(bvhindexes,(integer)llFrand(lGetListLength(bvhindexes))));  // return some random bhv at this pri
}
//
//  schedbvh - schedule and start next behavior
//
schedbvh()
{
    if (gActiveBehavior >= 0)                   // stop whatever was running
    {   stopbvh(gActiveBehavior);   
        gActiveBehavior = -1;                   // nobody running
        gActivePriority = 0;                
        gActiveToken = (gActiveToken+1)%(PATHMAXUNSIGNED-1);// advance run serial number, nonnegative
    }
    integer ix = getbvhtorun();                 // get next behavior to run
    if (ix < 0) { return; }                     // nothing to run
    //  Starting new task
    gActiveBehavior = ix;
    gActivePriority = llList2Integer(gBehaviors,3); // priority of this behavior
    startbvh(ix);
}
//
//  startbvh  -- start behavior
//
startbvh(integer ix)
{
    // ***MORE***
}

//
//  stopbvh -- stop behavior
//
stopbvh(integer ix)
{
    // ***MORE***
}




