//
//  bhvpatrol.lsl -- visit patrol points behavior.
//
//  Uses new-format patrol points file, lines of:
//
//      patrol,point,REGIONNAME,POS,DWELL,HEADING
//      patrol,speed,WALKSPEED
//  
//  Part of Animats pathfinding system.
//
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/assert.lsl"
#include "npc/bhv/bhvcall.lsl"
#include "npc/mathutils.lsl"

integer ACTION_IDLE = 0;
integer ACTION_PATROL = 2;                  // on patrol, heading for new point
integer ACTION_PATROLWAIT = 3;              // waiting for avatar type obstacle to move
integer ACTION_PATROLFACE = 4;              // completed patrol, turning to final position


//  Configuration

#define PRIORITYPATROL  PRIORITY_BACKGROUND // lower than greet, lower than evade vehicle

float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.5;            // (fract) Retry if random < this.
float TESTSPACING = 0.33;                   // (fract) Multiply height and width by this to get ray cast spacing

#define CHARACTER_SPEED  2.5                // (m/sec) default speed
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
#define IDLE_POLL 10.0                      // (secs) timer, used only for bhv library
#define MAX_PATROL_RETRIES 20               // ask owner for help after this many fails

//  Global variables
integer gAction = ACTION_IDLE;              // state of patrol

//  Animations
list gAnimWait = ["stand"];                 // When waiting - crossed arms or impatient
list gAnimIdle = ["stand"];                 // doing nothing

//  Patrol points
integer gPatrolEnabled;
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
float gSpeed = CHARACTER_SPEED; // how fast
integer gPatrolRetries;         // number of fail retries

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
            bhvAnimate(gAnimIdle);
            gAction = ACTION_PATROLFACE;            // turning to final position
            llResetTime();                          // reset timeout timer but keep dwell time
            debugMsg(DEBUG_MSG_INFO,"Patrol point reached:" + (string)llGetRootPosition());
            gPatrolRetries = 0;                     // reset failure counter
            return;
        }
        if (gAction == ACTION_IDLE)                 // nothing to do, but timer runs
        {   
            return;
        }
        //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        gAction = ACTION_IDLE;            
        bhvAnimate(gAnimIdle);
        return;
    }
    //  If blocked by something, deal with it.
    if (gAction == ACTION_PATROL)                                       // if going somewhere
    {   if (is_active_obstacle(hitobj))                                 // and it might move
        {
            if (llFrand(1.0) < OBSTACLE_RETRY_PROB)                     // if random test says retry
            {   debugMsg(DEBUG_MSG_WARN,"Obstacle " + llKey2Name(hitobj) + ", will try again.");
                gAction = ACTION_PATROLWAIT;                            // dwell briefly
                gDwell = 3.0;                                           // wait 3 secs
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
    //  Failed, back to idle.
    gAction = ACTION_IDLE;            
    debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
    bhvAnimate(gAnimIdle);
    gPatrolRetries++;                                               // tally failure
    if ((gPatrolRetries % (MAX_PATROL_RETRIES)) == 0)               // we need help, keep resending
    {   debugMsg(DEBUG_MSG_ERROR,"Need help! Patrol stuck at " + (string)llGetRootPosition() + ", retry " + (string)gPatrolRetries); }
}

//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    gAction = ACTION_IDLE;                                          // whatever we were doing is cancelled
    bhvAnimate(gAnimIdle);                                          // use idle animation
    start_patrol();                                                 // go patrol to someplace
    llSetTimerEvent(IDLE_POLL);                                     // check for dwell time
}
//
//  bhvDoStop -- this behavior no longer has control
//
bhvDoStop()
{
    gAction = ACTION_IDLE;                                          // we've been preempted. No patrol now.
    llSetTimerEvent(0);                                             // don't need a timer
}
//
//  bhvDoCollisionStart -- a collision has occured.
//
//  We don't actually have to do anything about the collision except apologise for it.
//  The path system will stop for this, and pathstart will retry.
//  Only avatars and physical objects generate collisions, because we're keyframe motion.
//
bhvDoCollisionStart(key hitobj)
{       
    list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
    integer pathfindingtype = llList2Integer(details,0);        // get pathfinding type
    debugMsg(DEBUG_MSG_WARN, "Collided with " + llList2String(details,1));
    if (pathfindingtype == OPT_AVATAR)                          // apologize if hit an avatar
    {   bhvSay("Excuse me."); }         
}      

//
//  start_patrol -- start patrolling if allowed to do so.
//
start_patrol()
{   //  Start patrolling if nothing else to do
    if (gAction == ACTION_IDLE && gPatrolEnabled && 
        llGetTime() > gDwell)      
    {   llResetTime(); 
        //  Pick a random patrol point different from the last one.
        integer newpnt;
        integer bound = llGetListLength(gPatrolPoints); // want 0..bound-1
        if (bound < 1)
        {   llSay(DEBUG_CHANNEL,"No patrol points, cannot patrol."); return; }
        if (bound > 1)                          // if more than one patrol point
        {   do { newpnt = rand_int(bound);  }   // pick next one at random, 
            while (newpnt == gNextPatrolPoint); // but not same as last time
            gNextPatrolPoint = newpnt;
            gPatrolRegionCorner = llList2Vector(gPatrolPointCorners, gNextPatrolPoint);
            gPatrolDestination = llList2Vector(gPatrolPoints, gNextPatrolPoint);
            gDwell = llList2Float(gPatrolPointDwell, gNextPatrolPoint);
            gFaceDir = llList2Float(gPatrolPointDir, gNextPatrolPoint);
        }
        restart_patrol();
    }  
}
//
//  restart_patrol -- start patrol to previously selected point.
//
restart_patrol()
{
    debugMsg(DEBUG_MSG_WARN,"Patrol to " + (string)gPatrolDestination);
    bhvAnimate(gAnimWait);                          // applies only when stalled during movement
    bhvNavigateTo(gPatrolRegionCorner,gPatrolDestination,0,gSpeed);  // head for next pos
    gAction = ACTION_PATROL;                        // patrolling
}
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

//
//  bhvDoConfigLine -- handle a config line, test version
//
bhvDoConfigLine(list params)
{   if (llList2String(params,0) == "patrol")
    {
        ////llOwnerSay("Config: " + llDumpList2String(params,","));
        string cmd = llList2String(params,1);                       // what command type
        if (cmd == "point")                                         // point to patrol to
        {   lookupregion(llList2String(params,2),params);           // look up name
            return;                                                 // end of lookup gets next notecard line
        }
        else if (cmd == "speed")
        {
            gSpeed = (float)llList2String(params,2);                // how fast
            if (gSpeed < 0.25) { gSpeed = 0.25; }                   // bound speed
            if (gSpeed > 8.0) { gSpeed = 8.0; }
        }
        else
        {   integer valid = doanimconfig(params);
            if (!valid)
            {   llSay(DEBUG_CHANNEL,"Unexpected config line: " +  llDumpList2String(params,","));
                bhvConfigDone(FALSE);                               // fails
            }
        }
    }
    bhvGetNextConfigLine();                                         // on to next notecard line
}
//
//  doanimconfig -- configure animations for animations
//
integer doanimconfig(list params)
{
    //  Store animations per cmd.
    string cmd = llList2String(params,1);           // Format: patrol, CMD, PARAM, PARAM ...
    params = llDeleteSubList(params,0,1);               // get rid of "greet, cmd"
    if (cmd == "waiting") { gAnimWait = params; }
    else if (cmd == "idling") { gAnimIdle = params; }
    else { llSay(DEBUG_CHANNEL,"No patrol command \"" + cmd + "\"."); return(FALSE); }    
    //  Must be an anim if we get here.
    //  Validate anims list
    integer i;
    for (i=0; i<llGetListLength(params); i++)
    {   string anim = llList2String(params,i);          // get this anim's name
        if (llGetInventoryType(anim) != INVENTORY_ANIMATION) // if no such animation
        {   llSay(DEBUG_CHANNEL,"Configured animation \"" + anim + "\" not found.");
            return(FALSE);                              // fails
        }    
    }
    
    
    return(TRUE);                                       // successful config    
}
//
//  startup - initialization
//
startup()
{
    gAction = ACTION_IDLE;
    gDwell = 0.0;                           // dwell time after patrol point reached
    gPatrolEnabled = FALSE;                 // turns on when all points loaded
    bhvReadConfig();                        // start reading the config
}

//
// bhvRegistered -- scheduler is ready to run us.
//
bhvRegistered()                                                     // tell controlling script to go
{   
    gPatrolEnabled = TRUE;                                          // start patrolling
    bhvSetPriority(PRIORITYPATROL);                                 // now we can ask to run
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
        if (llGetTime() < gDwell)                   // if clock has not run out
        {   return; }                               // ignore        
        if (gAction == ACTION_IDLE)                 // if talking to an avi
        {   //  Done waiting, go to a new patrol point
            start_patrol();                         // on to next patrol point
        } else if (gAction == ACTION_PATROLWAIT)    // we keep trying
        {   restart_patrol();                       // try again, same destination
        }
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
        else if (query_id == gLookupRegionStatusQuery) { lookupregionstatusreply(data); }
        else if (query_id == gLookupRegionCornerQuery) { lookupregioncornerreply(data); }
    }
}

