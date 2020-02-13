//
//  bhvanimator.lsl -- "animation overrider"
//
//  Uses new-format config file, lines of:
//
//      animator, walk, NEWANIM, ...
//      animator, slowwalk, NEWANIM, ...
//      animator, walk, NEWANIM, ...
//      animator, run, NEWANIM, ...
//      animator, turnleft, NEWANIM, ...
//      animator, turnright, NEWANIM, ...
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

//
//  Configuration
//
//  Timer
#define POLL_TIME 0.25
#define IDLE_TICKS 30                           // 7.5 secs to shutdown

//  Thresholds at which different animations turn on
#define SPEED_WALK      1.5                     // faster than this, fast walking
#define SPEED_RUN       3.0                     // faster than this, running
#define ROTRATE_TURN    0.3                     // medium speed turn
//
//  Globals
//
//  Animation info
//
//  The animations triggered automatically by movement.
//  These can and shoudl be overridden by the config notecard.
//
list gAnimStand = ["stand"];                        // defaults are the built in anims
list gAnimSlowWalk = ["crouchwalk"];
list gAnimWalk = ["walk"];
list gAnimRun = ["run"];
list gAnimLturn = ["turnleft"];                     // defaults are the the built-in anims
list gAnimRturn = ["turnright"];
list gAnimAlways = [];                              // permanent anims, always run these

list gCurrentAnims = [];                            // what's running now
integer gCurrentAnimState = -1;                     // optimization to prevent too much list work

integer gMoving;                                // true if moving
integer gIdleTicks;                             // not moving, turn off
rotation gPrevRot = ZERO_ROTATION;              // previous orientation
vector gPrevPos = ZERO_VECTOR;                  // previous position

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
    params = llDeleteSubList(params,0,1);               // get rid of "animator, cmd"
    //  Validate anims list
    integer i;
    for (i=0; i<llGetListLength(params); i++)
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

//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    bhvSetPriority(PRIORITY_OFF);                                   // we never take control of the NPC
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
//  stop_anims -- stop all anims not in the except list
//
stop_anims(list except)                                    
{   list anims =  llGetObjectAnimationNames();
    integer i;
    for (i=0; i < llGetListLength(anims); i++)
    {   string s = llList2String(anims,i);
        if (llListFindList(except,[s]) < 0)             // if not in "except" list, stop
        {   if (DEBUG) { llOwnerSay("Stopping " + s); } // ***TEMP***
            llStopObjectAnimation(s);
        }
    }
}
//
//  start_anims -- start all anims in list, stopping anything not in list
//
start_anims(list anims)
{
    if (anims == gCurrentAnims)                 // compares list length only
    {   if (llListFindList(gCurrentAnims, anims) == 0)  // if identical
        {   return; }                           // no changes needed
    }
    //  Anims are changing, must do the hard case
    integer i;
    for (i=0; i<llGetListLength(anims); i++)
    {   string anim = llList2String(anims,i);   // nth anim
        if (llListFindList(gCurrentAnims,[anim]) < 0) // if new anim
        {   llStartObjectAnimation(anim);       // start new anim
            if (DEBUG) llOwnerSay("Starting " + anim);     // ***TEMP***
        }
    }
    llSleep(0.1);                               // allow more time overlap with both running
    stop_anims(anims);                          // stop everything else
    gCurrentAnims = anims;                      // set anims
}
//
//  setanims -- set a new animation state.
//  Optimization to avoid too much useless list work on a fast timer.
//
#define setanims(statenum, animlist) { if ((statenum) != gCurrentAnimState) { start_anims((animlist)); gCurrentAnimState = statenum; }}
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
    gPrevRot = llGetRootRotation();             // update rot
    vector prevpos = gPrevPos;
    gPrevPos = llGetRootPosition();
    //  Need rotation around Z axis betwen those two rotations
    vector posdiff = gPrevPos - prevpos;        // distance moved
    rotation rotdiff = gPrevRot / prevrot;      // rotation on this cycle
    vector rotang = llRot2Euler(rotdiff);       // as angle
    float rotspeed = 0;
    if (elapsed > 0)                            // avoid divide by zero
    {   rotspeed = rotang.z / elapsed; }
    float speed = llVecMag(llGetVel());         // moving how fast?
    float posspeed = 0;                         // speed computed from position change
    if (elapsed > 0)
    {   posspeed = llVecMag(posdiff) / elapsed; }
    if (speed > posspeed) { posspeed = speed; }
    ////llSetText((string)speed + " m/sec  " +  (string)posspeed + " m/sec", <0,1,1>, 1.0);  // ***TEMP***
    //  When movement starts, there's a short delay before we see a nonzero velocity and a
    //  change in position. This causes a brief "skid", where the body moves before the legs do.
    //  This fixes that.
    if (gMoving && (posspeed <= 0.01))
    {   posspeed = 0.02;                        // if moving, we must be moving a little
        ////llOwnerSay("Moving but zero vel."); // ***TEMP*** debug trap
    }
    if (posspeed < SPEED_WALK)                 // just stand or walk for now
    {   if (rotspeed > ROTRATE_TURN)
        {   setanims(1,gAnimLturn+gAnimAlways); 
            gIdleTicks = 0;                     // not idle
        }
        else if (rotspeed < -ROTRATE_TURN)
        {   setanims(2,gAnimRturn+gAnimAlways);
            gIdleTicks = 0;
        }
        else if (posspeed > 0.01)
        {   setanims(3,gAnimSlowWalk+gAnimAlways);          // slow walk
            gIdleTicks = 0;
        }
        else                                    // not moving
        {   setanims(4,gAnimStand+gAnimAlways);
            if (!gMoving)                       // if not moving, still might be turning
            {   gIdleTicks++;
                if (gIdleTicks > IDLE_TICKS)    // idle, ignore until awakened
                {   llSetTimerEvent(0.0); }     // put AO to sleep to reduce load
            }
        }
    } else if (posspeed > SPEED_RUN)            // running speed
    {   gIdleTicks = 0;
        setanims(5,gAnimRun+gAnimAlways);
    } else {
        gIdleTicks = 0;
        setanims(6,gAnimWalk+gAnimAlways);
    }
}

//
//  bhvDoConfigLine -- handle a config line, test version
//
bhvDoConfigLine(list params)
{   if (llList2String(params,0) == "animator")
    {   integer valid = loadanims(params);                          // load and diagnose param line
        if (!valid) { return; }                                     // fails, we will not start
    }
    bhvGetNextConfigLine();                                         // on to next notecard line
}
//
//  startup - initialization
//
startup()
{                                                        
    
}

//
// bhvRegistered -- initialization is done.
//
bhvRegistered()                                                     // tell controlling script to go
{   
    bhvSetPriority(PRIORITY_OFF);                                   // we never take control of the NPC
    setanims(999,gAnimAlways);                                      // start the "always" animations
}

default
{
 
    state_entry()
    {   gBhvThisService = "anim";                   // we are the "anim" service.
        bhvReadConfig();                            // start reading the config
    }

    timer()                                         // timer tick
    {   bhvTick();                                  // timers in path library get updated
        update_anim();
    }
    
    on_rez(integer param)
    {   llResetScript(); }
       
    
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

    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   ////llOwnerSay("animator: " + (string)num + ": " + jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            if (llJsonGetValue(jsn,["request"]) == "anim")    // if this is an anim request from the scheduler
            {   gAnimStand = llJson2List(llJsonGetValue(jsn, ["stand"])); // get the new stand anims being set. Can only set stand for now.
                gCurrentAnimState = -1;                     // invalidate optimization
            } 
            else 
            {   bhvSchedMessage(num,jsn);                       // message from scheduler
            }
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

