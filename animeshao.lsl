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
string IDLE_ANIM = "CCBirdGirlStand01.2";    // idle or chatting         
string STAND_ANIM = "bp hands behind back"; // just when stopped
string WALK_ANIM = "Female Walk 1"; 
string TURN_ANIM = "RD Walk In Place";      // when turning
float POLL_TIME = 0.5;
float IDLE_TICKS = 10;                          // 5 secs to shutdown

float SPEED_WALK = 0.1;                         // faster than this, walking
float ROTRATE_TURN = 0.3;                       // medium speed turn

//  ***MORE*** run, etc.

//  Globals
string gStandAnim = STAND_ANIM;                        // base stand animation
string gCurrentAnim = "";                              // no current animation
integer gMoving;
integer gIdleTicks;                                     // not moving, turn off

stop_anims()                                    
{   list anims =  llGetObjectAnimationNames();
    integer i;
    for (i=0; i < llGetListLength(anims); i++)
    {   string s = llList2String(anims,i);
        llOwnerSay("Stopping " + s);
        llStopObjectAnimation(s);
    }
}

start_anim(string anim)
{   if (gCurrentAnim == anim) { return; }       // no change
    stop_anims();                               // stop old anims
    llStartObjectAnimation(anim);               // start new anim
    llOwnerSay("Start animation " + anim);      // change animation
    gCurrentAnim = anim;                        // current anim
}

update_anim()                                   // called periodically and when movement starts
{
    float speed = llVecMag(llGetVel());     // moving how fast?
    float rotspeed = llVecMag(llGetOmega());// rotating how fast
    if (speed < SPEED_WALK)                 // just stand or walk for now
    {   if (rotspeed > ROTRATE_TURN)
        {   start_anim(TURN_ANIM); 
            gIdleTicks = 0;                 // not idel
        }
        else 
        {   start_anim(gStandAnim);
            if (!gMoving)                       // if not moving, still might be turning
            {   gIdleTicks++;
                if (gIdleTicks > IDLE_TICKS)    // idle, ingnore until awakened
                {   llSetTimerEvent(0.0); }     // put AO to sleep to reduce load
            }
        }
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
        gIdleTicks = 0;                         // begin shutdown timing
    } 
    
    link_message(integer source, integer num, string str, key id)
    {
        llSetTimerEvent(POLL_TIME);               // wake up
        gIdleTicks = 0;                         // stay awake long enough to do whatever asked to do
        if (num == 1)                           // change stand animation
        {   gStandAnim = str; }                 // use new stand animation
    }    
}
