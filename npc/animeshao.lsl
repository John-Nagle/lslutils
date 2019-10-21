//
//   Animesh AO
//
//   Manages movement animations for an avatar
//
//   TEST ONLY
//   
//   Animats
//   2019
//
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
{
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
    float speed = llVecMag(llGetVel());     // moving how fast?
    float posspeed = 0;
    if (elapsed > 0)
    {   posspeed = llVecMag(posdiff) / elapsed; }
    if (speed > posspeed) { posspeed = speed; }
    ////llSetText((string)speed + " m/sec  " +  (string)posspeed + " m/sec", <0,1,1>, 1.0);  // ***TEMP***
    if (posspeed < SPEED_WALK)                 // just stand or walk for now
    {   if (rotspeed > ROTRATE_TURN)
        {   start_anim(LTURN_ANIM); 
            gIdleTicks = 0;                 // not idle
        }
        else if (rotspeed < -ROTRATE_TURN)
        {   start_anim(RTURN_ANIM);
            gIdleTicks = 0;
        }
        else if (posspeed > 0.01)
        {   start_anim(SLOW_WALK_ANIM);         // slow walk
            gIdleTicks = 0;
        }
        else                                    // not moving
        {   start_anim(gStandAnim);
            if (!gMoving)                       // if not moving, still might be turning
            {   gIdleTicks++;
                if (gIdleTicks > IDLE_TICKS)    // idle, ingnore until awakened
                {   llSetTimerEvent(0.0); }     // put AO to sleep to reduce load
            }
        }
    } else if (posspeed > SPEED_RUN)            // running speed
    {   gIdleTicks = 0;
        start_anim(RUN_ANIM);
    } else {
        gIdleTicks = 0;
        start_anim(WALK_ANIM);
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

