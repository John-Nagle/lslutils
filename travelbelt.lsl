//
//   Travel belt
//
//   Animats
//   November, 2018
//
//  People mover belt.  Works like a real one.  Walk on it and it moves you.
//
//  The belt is always running. But, to eliminate all load on the sim when not in use,
//  when no one is on the belt, it's run as a texture animation.
//

integer gOn;                                    // belt is running
vector gStartPos;                               // start position
rotation gStartRot;                             // start rotation
float CYCLETIME = 0.5;                          // move cycle time
float TEXSCALEX = 0.5;                          // maintain texture scale regardless of size           
float TEXSCALEY = 0.5;                          // per half meter, as documented
float STOPTIME = 10.0;                          // stop belt when no traffic

start_belt()                                    // start belt moving 
{
    if (gOn) { return; }                        // already running
    gStartPos = llGetPos();
    gStartRot = llGetRot();
    vector motionDir = <1,0,0>;                 // direction of motion, local
    motionDir = motionDir * gStartRot;          // direction of motion, world
    llScaleTexture(TEXSCALEX, TEXSCALEY, ALL_SIDES);
    llSetTextureAnim(FALSE, ALL_SIDES, 0, 0, 0.0, 0.0, 1.0);    // stop texture animation
    //  Keyframed motion with snapback.
    llSetKeyframedMotion([motionDir, CYCLETIME],[KFM_DATA,KFM_TRANSLATION, KFM_MODE,KFM_LOOP]);
    gOn = TRUE;                                 // belt is running
    llSetTimerEvent(STOPTIME);                  // stop after this much idle time
}

//
//   stop_belt. Belt stops really moving, but texture animation takes over to maintain the illusion.
//  
stop_belt()
{
    llSetTimerEvent(0.0);                       // stop timer
    llSetKeyframedMotion([],[]);                // stop keyframed motion
    llSetTextureAnim(ANIM_ON|LOOP|SMOOTH|REVERSE, ALL_SIDES, 0, 1, 1.0, 1.0, 1.0/CYCLETIME);
    gOn = FALSE;
    llSleep(CYCLETIME*2.0);                     // wait for stop
    llSetPos(gStartPos);                        // restore position to stop creep
    llSetRot(gStartRot);
}
        
default
{
    state_entry()
    {   llSetKeyframedMotion([],[]);                // stop keyframed motion
        gOn = FALSE;                                // not running
        llSleep(CYCLETIME*2.0);                     // wait for stop
        gStartPos = llGetPos();                     // capture initial position
        gStartRot = llGetRot(); 
        llSetTextureAnim(ANIM_ON|LOOP|SMOOTH|REVERSE, ALL_SIDES, 0, 1, 1.0, 1.0, 1.0/CYCLETIME);
    }
    
    collision_start(integer num_detected)
    {
        start_belt();
        llSetTimerEvent(STOPTIME);                  // stop after this much idle time
    }   
    
    collision(integer num_detected)
    {
        start_belt();
        llSetTimerEvent(STOPTIME);                  // stop after this much idle time
    }   
    
    on_rez(integer start_param)
    {   llResetScript();
    }
    
    timer()
    {   stop_belt(); }                                  // idle timeout
}

