//
//  bhvanimator.lsl -- "animation overrider"
//
//  Uses new-format config file, lines of:
//
//      animator, walk, NEWANIM ...
//      animator, always, NEWANIM, NEWANIM
//
//  The general idea is that these anims are used when in motion,
//  automatically selected based on movement. 
//  but the active behavior changes the "stand" anim depending on what it
//  is doing.
//  
//  Part of Animats pathfinding system.
//
//
//  License: GPLv3.
//
//  Animats
//  2020
//
#include "npc/assert.lsl"
#include "npc/bhv/bhvcall.lsl"

#ifdef OBSOLETE
#include "npc/mathutils.lsl"

integer ACTION_IDLE = 0;
integer ACTION_PATROL = 2;                  // on patrol, heading for new point
integer ACTION_PATROLWAIT = 3;              // waiting for avatar type obstacle to move
integer ACTION_PATROLFACE = 4;              // completed patrol, turning to final position


//  Configuration

#define PRIORITYPATROL  PRIORITY_BACKGROUND // lower than greet, lower than evade vehicle

float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.5;            // (fract) Retry if random < this.
float TESTSPACING = 0.33;                   // (fract) Multiply height and width by this to get ray cast spacing

#define CHARACTER_SPEED  2.5                // (m/sec) speed
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
////string IDLE_ANIM = "stand 2";            // idle or chatting         
////string STAND_ANIM = "stand 2";           // just when stopped
///string WAITING_ANIM = "stand arms folded";  // during planning delays
string WAITING_ANIM = "SEmotion-bento13";   // arms folded during planning delays
string IDLE_ANIM = "SEmotion-bento18";      // arms folded during planning delays
string STAND_ANIM = "SEmotion-bento18";     // just when stopped
float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                  // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;            // must move at least this far before we recheck on approach
integer PATH_STALL_TIME = 300;              // path stall time


//  Global variables
integer gAction = ACTION_IDLE;

//  Patrol points
integer gPatrolEnabled;
integer gPatrolNotecardLine;
key gPatrolNotecardQuery;
list gPatrolPointCorners = [];  // region corners for patrol points
list gPatrolPoints = [];        // list of points to patrol 
list gPatrolPointDwell = [];    // dwell time at patrol point
list gPatrolPointDir = [];      // direction to face on arriving

//  Globals
integer gNextPatrolPoint;       // next patrol point we are going to
vector gPatrolDestination;      // where are we going next?
vector gPatrolRegionCorner;     // region corner for next patrol point
float gDwell;                   // dwell time at next patrol point
float gFaceDir;                 // direction to face next
float gSpeed = CHARACTER_SPEED;               // how fast

//
//  bhvDoRequestDone -- pathfinding is done. Analyze the result and start the next action.
//
bhvDoRequestDone(integer status, key hitobj)
{   debugMsg(DEBUG_MSG_WARN, "Path update: " + (string) status + " obstacle: " + llKey2Name(hitobj)); // ***TEMP*** temporarily WARN
    if (status == PATHERRMAZEOK)                    // success
    {   
        if (gAction == ACTION_PATROLFACE)           // final face at end of patrol move
        {   gAction = ACTION_IDLE;                  // and we go idle
            return;
        }
        if (gAction == ACTION_PATROL)               // at goal
        {   bhvTurn(gFaceDir);                      // face in programmed direction
            start_anim(IDLE_ANIM);
            gAction = ACTION_PATROLFACE;            // turning to final position
            llResetTime();                          // reset timeout timer but keep dwell time
            debugMsg(DEBUG_MSG_INFO,"Patrol point reached:" + (string)llGetRootPosition());
            return;
        }
        if (gAction == ACTION_IDLE)                 // nothing to do, but timer runs
        {   
            return;
        }
        //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        gAction = ACTION_IDLE;            
        start_anim(IDLE_ANIM);
        return;
    }
    //  If blocked by something, deal with it.
    if (gAction == ACTION_PATROL)                                       // if going somewhere
    {   if (is_active_obstacle(hitobj))                                 // and it might move
        {
            if (llFrand(1.0) < OBSTACLE_RETRY_PROB)                     // if random test says retry
            {   debugMsg(DEBUG_MSG_WARN,"Obstacle " + llKey2Name(hitobj) + ", will try again.");
                gAction = ACTION_PATROLWAIT;                        // dwell briefly
                gDwell = 3.0;                                       // wait 3 secs
                return;
            } else {                                                    // no retry, give up now
                gDwell = 0.0;                                           // no dwell time, do something else now
                gAction = ACTION_IDLE;                                  // which will try a different patrol point
                debugMsg(DEBUG_MSG_WARN,"Obstacle " + llKey2Name(hitobj) + ", will give way.");
            }
        } else {
           debugMsg(DEBUG_MSG_WARN,"Stopped by fixed obstacle: " + llKey2Name(hitobj)); // report stopped by fixed obstacle
        }
    }
    //  Default - errors we don't special case.
    {                      
        //  Failed, back to idle.
        gAction = ACTION_IDLE;            
        debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        start_anim(IDLE_ANIM);
        return;
    }
}
#endif // OBSOLETE
//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    bhvSetPriority(PRIORITYOFF);                                    // we never take control of the NPC
}
//
//  bhvDoStop -- this behavior no longer has control
//
bhvDoStop()
{}

//
//  bhvDoRequestDone -- pathfinding is done. Should never be called.
//
bhvDoRequestDone(integer status, key hitobj) {}
//
//  bhvDoCollisionStart -- a collision has occured. Should never be called.
//
bhvDoCollisionStart(key hitobj)
{}      


//
//  Config reading
//
//  
//  bhvConfigDone -- entire config read, done
//
bhvConfigDone(integer valid)
{
    gBhvConfigNotecardLine = -1;                        // no more reading
    if (valid)
    {   llOwnerSay("Configuration done.");
        bhvInit();                                      // set up scheduler system
    }
    else
    {   llSay(DEBUG_CHANNEL,"=== CONFIGURATION FAILED ==="); // stuck
    }
}

//
//  Animation info
//
//  The animations triggered automatically by movement.
//  These can and shoudl be overridden by the config notecard.
list gAnimStand = ["stand"];
list gAnimSlowWalk = ["crouchwalk"];
list gAnimWalk = ["walk"];
list gAnimRun = ["run"];
lisr gAnimLturn = ["turnleft"];                     // defaults are the the built-in anims
list gAnimRturn = ["turnright"];

list gAnimAlways = [];                              // permanent anims, always run these

//
//  loadanims -- load requested anims from notecard info
//
//  Param format is
//
//  animator, walk, NEWANIM ...
//
//  Returns TRUE if all anims valid.
//
integer loadanims(list params)
{
    string cmd = llList2String(params,1);               // request type
    params = llListDeleteList(params,0,1);              // get rid of "animator, cmd"
    //  Validate anims list
    integer i;
    for (i=0; i<llGetListLength(params), i++)
    {   string anim = llList2String(params,i);          // get this anim's name
        if (llGetInventoryType(anim) != INVENTORY_ANIMATION) // if no such animation
        {   llSay(DEBUG_CHANNEL,"Configured animation \"" + anim + "\" not found.");
            return(FALSE);                              // fails
        }
    
    }
    //  Store animations per cmd.
    if (cmd == "stand") { gAnimStand = params; }
    else if (cmd == "crouchwalk" || cmd == "slowwalk") { gAnimSlowWalk = params; }
    else if (cmd == "walk") { gAnimWalk = params; }
    else if (cmd == "run") { gAnimRun = params; }
    else if (cmd == "turnleft") { gAnimLturn = params; }
    else if (cmd == "turnright") { gAnimRturn = params; }
    else if (cmd == "always") { gAnimAlways = params; }
    else { llSay(DEBUG_CHANNEL,"No animator command \"" + cmd + "\"."); return(FALSE); }
    return(TRUE);                                       // successful config    
}

#ifdef OBSOLETE
string IDLE_ANIM = "stand 2";    // idle or chatting         
string STAND_ANIM = "stand 2"; // just when stopped
string WALK_ANIM = "Female Walk 1";
string RUN_ANIM = "avatar_run";            // default run animation 
string SLOW_WALK_ANIM = "animesh_slow_short_stride_walk";   // for slow movement
////string TURN_ANIM = "RD Walk In Place";      // when turning
string RTURN_ANIM = "TurnR";
string LTURN_ANIM = "TurnL";
string BLINK_ANIM = "Benato Blink";         // eye blink cycling animation
list PERM_ANIMS = [BLINK_ANIM];               // anims we always run
#endif // OBSOLETE

float POLL_TIME = 0.25;
float IDLE_TICKS = 30;                          // 7.5 secs to shutdown
integer DEBUG = FALSE; /// TRUE;                // messages on/off

float SPEED_WALK = 0.7;                         // faster than this, walking
float SPEED_RUN = 3.0;                          // faster than this, running
float ROTRATE_TURN = 0.3;                       // medium speed turn

//  ***MORE*** run, etc.

//  Globals
string gStandAnim = STAND_ANIM;                        // base stand animation
string gCurrentAnim = "";                              // no current animation
integer gMoving;
integer gIdleTicks;                                     // not moving, turn off
rotation gPrevRot = ZERO_ROTATION;                      // previous orientation
vector gPrevPos = ZERO_VECTOR;                          // previous position

stop_anims(list except)                                    
{   list anims =  llGetObjectAnimationNames();
    integer i;
    for (i=0; i < llGetListLength(anims); i++)
    {   string s = llList2String(anims,i);
        if (llListFindList(except,[s]) < 0)             // if not in "except" list, stop
        {   if (DEBUG) { llOwnerSay("Stopping " + s); }
            llStopObjectAnimation(s);
        }
    }
}

start_anim(string anim)
{   if (gCurrentAnim == anim) { return; }       // no change
    llStartObjectAnimation(anim);               // start new anim
    if (DEBUG) { llOwnerSay("Start animation " + anim); } // change animation
    llSleep(0.1);                               // allow more time overlap with both running
    stop_anims(PERM_ANIMS + anim);              // stop old anims
    gCurrentAnim = anim;                        // current anim
}

//
//  update_anim -- pick which animation to use, based on current movement.
//
//  Velocity from llGetVel is not reliable if being moved with llSetPos,
//  which the behavior controller can do if it wants.
//
update_anim()                                   // called periodically and when movement starts
{   if (!gBhvRegistered) { return; }            // not ready to run yet
    float elapsed = llGetAndResetTime();        // time since last tick
    rotation prevrot = gPrevRot;                // rot on last cycle
    gPrevRot = llGetRot();                      // update rot
    vector prevpos = gPrevPos;
    gPrevPos = llGetPos();
    //  Need rotation around Z axis betwen those two rotations
    vector posdiff = gPrevPos - prevpos;        // distance moved
    rotation rotdiff = gPrevRot / prevrot;      // rotation on this cycle
    vector rotang = llRot2Euler(rotdiff);       // as angle
    float rotspeed = 0;
    if (elapsed > 0)                            // avoid divide by zero
    {   rotspeed = rotang.z / elapsed; }
    float speed = llVecMag(llGetVel());         // moving how fast?
    float posspeed = 0;
    if (elapsed > 0)
    {   posspeed = llVecMag(posdiff) / elapsed; }
    if (speed > posspeed) { posspeed = speed; }
    ////llSetText((string)speed + " m/sec  " +  (string)posspeed + " m/sec", <0,1,1>, 1.0);  // ***TEMP***
    if (posspeed < SPEED_WALK)                 // just stand or walk for now
    {   if (rotspeed > ROTRATE_TURN)
        {   start_anim(gAnimLturn); 
            gIdleTicks = 0;                     // not idle
        }
        else if (rotspeed < -ROTRATE_TURN)
        {   start_anim(gAnimRturn);
            gIdleTicks = 0;
        }
        else if (posspeed > 0.01)
        {   start_anim(gAnimSlowwalk);          // slow walk
            gIdleTicks = 0;
        }
        else                                    // not moving
        {   start_anim(gAnimStand);
            if (!gMoving)                       // if not moving, still might be turning
            {   gIdleTicks++;
                if (gIdleTicks > IDLE_TICKS)    // idle, ingnore until awakened
                {   llSetTimerEvent(0.0); }     // put AO to sleep to reduce load
            }
        }
    } else if (posspeed > SPEED_RUN)            // running speed
    {   gIdleTicks = 0;
        start_anim(gAnimRun);
    } else {
        gIdleTicks = 0;
        start_anim(gAnimWalk);
    }
}


default
{
    state_entry()
    {
        gMoving = FALSE;
        gIdleTicks = 0;
        integer i;
        for (i=0; i<llGetListLength(PERM_ANIMS); i++)
        {   llStartObjectAnimation(llList2String(PERM_ANIMS,i)); }  // start all "always" anims
    }
    
    on_rez(integer param)
    {   llResetScript(); }
       
    timer()
    {   update_anim();    }
    
    moving_start()                              // run timer only when moving
    {   llSetTimerEvent(POLL_TIME); 
        gMoving = TRUE;
        update_anim();                          // update immediately
        gIdleTicks = 0;
    }
    
    moving_end()
    {   
        gMoving = FALSE;
        update_anim();
        gIdleTicks = 0;                         // begin shutdown timing
    } 
    
    link_message(integer source, integer num, string str, key id)
    {
        llSetTimerEvent(POLL_TIME);             // wake up on any message
        gIdleTicks = 0;                         // stay awake long enough to do whatever asked to do
        if (num == 1)                           // change stand animation
        {   gStandAnim = str; }                 // use new stand animation
    }    
}


















#ifdef OBSOLETE
//
//  Region name lookup. This takes two data server calls for each region name.
//  So we have a one-item cache.
//
key gLookupRegionStatusQuery = NULL_KEY;                // query ID for region
key gLookupRegionCornerQuery = NULL_KEY;                // query ID for region corner
string gLookupRegionName = "";                          // name being looked up
list gLookupRegionParams;                               // info associated with current query for callback
string gCachedRegionName;                               // saved name
vector gCachedRegionCorner;                             // saved corner
//
//  lookupregion -- start process of looking up a region
//
lookupregion(string name, list params)
{
    //  Stage 1: see if region is known.
    //  A quary for the position of an unknown region generates no reply.
    //  But a status query gets a reply of "unknown"
    if (name == gCachedRegionName)
    {   lookupregioncallback(gCachedRegionCorner, params);  // in cache, use cached result
        return;
    }
    gLookupRegionParams = params;                       // save params for callback
    gLookupRegionName = name;                           // save name for callback
    assert(gLookupRegionStatusQuery == NULL_KEY);
    gLookupRegionStatusQuery = llRequestSimulatorData(name,DATA_SIM_STATUS);
}
//
//  lookupregionstatusreply -- dataserver event called after region status lookup complete
//
lookupregionstatusreply(string data)
{   //  Stage 2: get location of known region
    gLookupRegionStatusQuery = NULL_KEY;                // note no dataserver request in progress
    if (data == "unknown")                                      // status no find
    {   lookupregioncallback(ZERO_VECTOR, gLookupRegionParams);// report no find
        return;
    }
    gLookupRegionCornerQuery = llRequestSimulatorData(gLookupRegionName,DATA_SIM_POS);
}
//
//  lookupregioncornerreply -- dataserver event called after region corner lookup complete
//
lookupregioncornerreply(string data)
{   gCachedRegionName = gLookupRegionName;                  // cache this result
    gCachedRegionCorner = (vector)data;                     // cache this result
    lookupregioncallback(gCachedRegionCorner,gLookupRegionParams);     // got data 
}
//
//  lookupregioncallback -- actual callback to do the work
//
lookupregioncallback(vector corner, list params)
{
    llOwnerSay("Patrol point: " + llDumpList2String(params,",") + " region " + (string)corner);
    //  Add this patrol point
    vector point = (vector)llList2String(params,3);                 // get point to patrol
    if (point == ZERO_VECTOR)                                       // if invalid point
    {   llSay(DEBUG_CHANNEL,"Invalid patrol point in config: " +  llDumpList2String(params,",")); 
        bhvConfigDone(FALSE);                                       // fails
    }
    gPatrolPointCorners += corner;                                  // region corners for patrol points
    gPatrolPoints += point;                                         // point to patrol 
    gPatrolPointDwell += (float)llList2String(params,4);            // dwell time at patrol point
    gPatrolPointDir += (float)llList2String(params,5);              // direction to face on arriving   
    bhvGetNextConfigLine();                                         // all done, go on 
}
#endif // OBSOLETE
//
//  bhvDoConfigLine -- handle a config line, test version
//
bhvDoConfigLine(list params)
{   if (llList2String(params,0) == "animator")
    {   integer valid = loadanims(list params);                     // load and diagnose param line
        if (!valid) { return; }                                     // fails, we will not start
    }
    bhvGetNextConfigLine();                                         // on to next notecard line
}
//
//  startup - initialization
//
startup()
{
    bhvReadConfig();                        // start reading the config
}

//
// bhvRegistered -- initialization is done.
//
bhvRegistered()                                                     // tell controlling script to go
{   
    bhvSetPriority(PRIORITYOFF);                                    // we never take control of the NPC
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

    timer()                                         // timer tick
    {   bhvTick();                                  // timers in path library get updated
    }
    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   ////llOwnerSay("patrol: " + (string)num + ": " + jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            bhvSchedMessage(num,jsn);                       // message from scheduler
            return;
        } else if (num == DEBUG_MSGLEV_BROADCAST)       // set message level from broadcast
        {   debugMsgLevelSet(jsn); }
    }
    
    dataserver(key query_id, string data)               
    {   ////llOwnerSay("Dataserver callback: " + data);     // ***TEMP***
        //  Handle all the various data server events
        if (query_id == gBhvConfigNotecardQuery)       {   bhvParseConfigLine(data, gBhvConfigNotecardLine);}
    }
}

