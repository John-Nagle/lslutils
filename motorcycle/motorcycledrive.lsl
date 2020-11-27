//  Motorcycle driving script
//  Automatic region crossing trouble prevention
//  EXPERIMENTAL.
//
//  Animats
//  March, 2018
//  Derived from a motorcycle script of unknown origin.
//
//  Bike control is basic. Arrow keys to steer, shift-rightarrow for gear up, shift leftarrow for gear down.
//  Rider and passenger.
//
#include "regionrx.lsl"            // region crossing system
//  Constants
float TINY = 0.0001;                // small value to prevent divide by zero
float HUGE = 100000000000.0;        // huge number bigger than any distance
float REGION_SIZE = 256.0;          // SL region size
integer LIGHT_CONTROL_CH = 1102;    // light on/off

//    Basic settings
                                    // region cross speed control
integer SIT_TROUBLE_MAX = 50;         // in trouble if out of position this many
float SPEED_STATIONARY = 0.05;      // 5cm/sec is "stopped"
vector SIT_TARGET_POS = <-0.25,0.0,1.00>; // Sit target pos
vector SIT_TARGET_ROT = <0.0,0,0.0>;     // Sit target rotation (degrees)

float MIN_SAFE_DOUBLE_REGION_CROSS_TIME = 2.0;      // min time between double region crosses at speed
float MIN_BRAKE_SPEED = 0.2;                        // minimum speed we will ever brake to
float COAST_POWER = 2.0;                            // power to use when coasting across a region crossing

float DOWNFORCE = -10.0;                            // was 2.0
float LOUD_VOLUME = 1.0;                            // full annoying hog volume
float SOFT_VOLUME = 0.15;                           // much quieter

//  Inter-link messages within bike
integer     DIR_STOP = 100;
integer     DIR_START = 101;
integer     DIR_NORM = 102;
integer     DIR_LEFT = 103;
integer     DIR_RIGHT = 104;

integer     MSG_DEMO = 200;                         // will die when rider gets off
integer     MSG_LOCKED = 201;                       // will only accept owner
integer     MSG_UNLOCKED = 202;                     // will accept anyone
integer     MSG_LOUD = 301;                         // loud sounds
integer     MSG_SOFT = 302;                         // softer sounds                  


float       BankPower=6000;
float       ForwardPower; //Forward power
list        ForwardPowerGears = [5,10,15,25,40,60,80];
float       ReversePower = -5; //Reverse power, same as first gear, but in reverse
float       TurnPower = 1500;  //  was 1000; //Turning power
float       TurnSpeedAdjust = 0.7; //how much effect speed has on turning, higher numbers effect more. 0.0 to disable
integer     Permission_Set = 0; // must initialize at startup
integer     Need_Camera_Reset = FALSE;  // camera reset needed?

string      SitText = "Drive"; //Text to show on pie menu

string      IdleSound = "idle";                 // Sound to play when idling
string      RunSound = "run";                   // Sound to play when the gas in pressed
string      StartupSound = "starter";           // Sound to play when owner sits
string      NextSound = "";                     // sound to run next
//      Animation control
string      DrivingAnim = "Bike_Ride01";        // Driver when riding
string      IdleAnim = "Biker_Idle01";          // Driver when idle or off
string      CurrentAnim = "";                   // animation currently running, if any
string      NextAnim = "";                      // animation we want running
//Other variables
integer     NumGears;
integer     Gear = 0;
integer     NewSound;
string      Sound;
integer     CurDir;                             // left, right, or norm
integer     Forward;                            // +1 or -1
vector      SpeedVec;
vector      Linear;
vector      Angular;
integer     Active = FALSE;
float       lowestslowmsg = HUGE;               // for minimizing SLOW log messages
key         sitting = NULL_KEY;                 // the driver's UUID
integer     LockStatus = MSG_DEMO;              // demo version unless overridden
float       Loudness = LOUD_VOLUME;             // start out loud


integer TurnSpeedNum=1;
list turn_speeds=[.15,.12,.10,.01];
float TurnSpeed;

#include "vehicleutils.lsl"

set_vehicle_params()                // set up SL vehicle system
{
    /*
    llSetVehicleType(VEHICLE_TYPE_CAR);
    llSetVehicleFlags(VEHICLE_FLAG_NO_DEFLECTION_UP | VEHICLE_FLAG_LIMIT_ROLL_ONLY | VEHICLE_FLAG_LIMIT_MOTOR_UP);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_EFFICIENCY, 0.4);
    llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_EFFICIENCY, 0.80);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_TIMESCALE, 0.10);
    llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_TIMESCALE, 0.10);
    llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_TIMESCALE, 5);
    llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_DECAY_TIMESCALE, 0.2);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_DECAY_TIMESCALE, 0.2);
    llSetVehicleVectorParam(VEHICLE_LINEAR_FRICTION_TIMESCALE, <6, 0.5, 1000.0> );
    llSetVehicleVectorParam(VEHICLE_ANGULAR_FRICTION_TIMESCALE, <10.0, 10.0, 0.5> );
    llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_EFFICIENCY, 0.50);
    llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_TIMESCALE, 0.40);
    llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, 1.0);
    llSetVehicleFloatParam(VEHICLE_BANKING_TIMESCALE, 0.01);
    llSetVehicleFloatParam(VEHICLE_BANKING_MIX, 1.0);
    llCollisionSound("", 0.0);
        
    llRemoveVehicleFlags(VEHICLE_FLAG_HOVER_WATER_ONLY | VEHICLE_FLAG_HOVER_TERRAIN_ONLY | VEHICLE_FLAG_HOVER_GLOBAL_HEIGHT);
        
    llSetVehicleFlags(VEHICLE_FLAG_NO_DEFLECTION_UP | VEHICLE_FLAG_LIMIT_ROLL_ONLY | VEHICLE_FLAG_HOVER_UP_ONLY | VEHICLE_FLAG_LIMIT_MOTOR_UP);
*/
//  Values from rachel1206
    llSetVehicleType(VEHICLE_TYPE_CAR);

    llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_EFFICIENCY, 0.2);
    llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_EFFICIENCY, 1.0);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_TIMESCALE, 0.01);
    llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_TIMESCALE, 0.10);
    llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_TIMESCALE, 2);
    llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_DECAY_TIMESCALE, 0.2);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, 0.28);
    llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_DECAY_TIMESCALE, 0.2);
    llSetVehicleVectorParam(VEHICLE_LINEAR_FRICTION_TIMESCALE, <1.5, 0.5, 2.0> );
    llSetVehicleVectorParam(VEHICLE_ANGULAR_FRICTION_TIMESCALE, <0.01,  2.1, 2.5> );
    llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_EFFICIENCY, 0.7);
    llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_TIMESCALE, 0.4);
    llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, 0.99);
    llSetVehicleFloatParam(VEHICLE_BANKING_TIMESCALE, 0.01);
    llSetVehicleFloatParam(VEHICLE_BANKING_MIX, 0.5);


    llSetVehicleFloatParam(VEHICLE_BUOYANCY, -0.1);
    ////llSetVehicleFloatParam(VEHICLE_HOVER_HEIGHT, 0);
    ////llSetVehicleFloatParam(VEHICLE_HOVER_EFFICIENCY, 0);
    ////llSetVehicleFloatParam(VEHICLE_HOVER_TIMESCALE, 180);

    llRemoveVehicleFlags(VEHICLE_FLAG_HOVER_WATER_ONLY | VEHICLE_FLAG_HOVER_UP_ONLY | VEHICLE_FLAG_LIMIT_ROLL_ONLY
       | VEHICLE_FLAG_HOVER_TERRAIN_ONLY);
    llSetVehicleFlags(VEHICLE_FLAG_NO_DEFLECTION_UP | VEHICLE_FLAG_LIMIT_ROLL_ONLY| VEHICLE_FLAG_LIMIT_MOTOR_UP);
    
    llSetForce(<0,0,-5>,FALSE);   // forced downforce
    llCollisionSound("", 0.0);

}
//  set_camera_params
set_camera_params()
{
    llSetCameraParams([
        CAMERA_ACTIVE, 1, // 1 is active, 0 is inactive
        CAMERA_BEHINDNESS_ANGLE, 2.0, // (0 to 180) degrees
        CAMERA_BEHINDNESS_LAG, 0.1, // (0 to 3) seconds
        CAMERA_DISTANCE, 8.0, // ( 0.5 to 10) meters
        // CAMERA_FOCUS, <0,0,0>, // region-relative position
        CAMERA_FOCUS_LAG, 0.1 , // (0 to 3) seconds
        CAMERA_FOCUS_LOCKED, FALSE, // (TRUE or FALSE)
        CAMERA_FOCUS_THRESHOLD, 0.5, // (0 to 4) meters
        CAMERA_PITCH, 20.0, // (-45 to 80) degrees
        // CAMERA_POSITION, <0,0,0>, // region-relative position
        CAMERA_POSITION_LAG, 0.1, // (0 to 3) seconds
        CAMERA_POSITION_LOCKED, FALSE, // (TRUE or FALSE)
        CAMERA_POSITION_THRESHOLD, 0.5, // (0 to 4) meters
        CAMERA_FOCUS_OFFSET, <0,0,0> // <-10,-10,-10> to <10,10,10> meters
    ]);
    Need_Camera_Reset = FALSE;          // camera reset done
}
//  startup -- driver has just sat on bike. Call only in Ground state.
startup()
{   initregionrx(LOG_WARN);                     // init region crossing system
    llSay(9890,"s " +(string)TurnSpeedNum);
    llWhisper(11,"started");
    sitting = llAvatarOnSitTarget();                // this is the driver, not any passengers
    llRequestPermissions(sitting, Permission_Set);
    llStopSound();
    llTriggerSound(StartupSound, Loudness);
    llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
    llSetPos(llGetPos() + <0,0,0.15>);
    llSetStatus(STATUS_PHYSICS, TRUE);
    Sound = IdleSound;
    llLoopSound(Sound, Loudness);
    llSetTimerEvent(TIMER_INTERVAL);
    CurDir = DIR_NORM;
    Gear = 0;                                   // start in low gear
    ForwardPower = llList2Float(ForwardPowerGears, Gear);
    Linear.x = 0;
    Linear.y = 0;

}
//  shutdown -- shut down bike
shutdown(integer release)
{
    llSetTimerEvent(0.0);
    if (llGetPermissions() & PERMISSION_TRIGGER_ANIMATION)
    {   llStopAnimation(DrivingAnim);   }   // don't stop anim if not seated
    llStopSound();
    Sound = "";                             // no current sound
    llSetStatus(STATUS_PHYSICS, FALSE);
    Active = FALSE;

    llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
    if (release)
    {   llReleaseControls(); }
    logrx(LOG_NOTE,"SHUTDOWN", "Distance traveled (km)",gDistanceTraveled/1000.0); // last log message
    sitting = NULL_KEY;
}


default
{
    state_entry()
    {
        Permission_Set = PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS | PERMISSION_CONTROL_CAMERA;
        TurnSpeedAdjust *= 0.01;
        ForwardPower = llList2Integer(ForwardPowerGears, 0);
        NumGears = llGetListLength(ForwardPowerGears);
        Active = FALSE;
        sitting = NULL_KEY;                                                     // nobody is sitting
        llSetSitText(SitText);
        llCollisionSound("", 0.0);
        llSitTarget(SIT_TARGET_POS, llEuler2Rot(SIT_TARGET_ROT*DEG_TO_RAD));    // driver, not passenger
        TurnSpeed=llList2Float(turn_speeds, TurnSpeedNum-1);
        state Ground;
    }
}

state Ground
{
    state_entry()
    {   llStopSound();
        Sound = "";
        if(!Active)
        {
            llSetStatus(STATUS_PHYSICS, FALSE);
            llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
            llUnSit(llAvatarOnSitTarget());
        } else {
            llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
            NewSound = 1;
            Sound = IdleSound;
            Linear = <0,0,DOWNFORCE>;  
        }
        set_vehicle_params();
    }
    
    on_rez(integer param)
    {
        llResetScript();
    }
    
    //  Message from lock/unlock dialog.
    //  If we don't get a message from the lock/unlock dialog at startup, the bike is in demo
    //  mode and will die when the rider gets off.  So removing the owner menu script makes a bike a demo.
    link_message(integer sender_num, integer num, string msg, key id)
    {   if (num == MSG_LOCKED || num == MSG_UNLOCKED)       // if from lock script
        {   LockStatus = num;                               // set lock status
            if (num == MSG_UNLOCKED) 
            { llWhisper(0,"Bike unlocked - anyone can ride."); }
            else 
            { llWhisper(0,"Bike locked - owner only."); }
        }
        if (num == MSG_LOUD || num == MSG_SOFT)             // if menu command to change volume
        {   if (num == MSG_LOUD) { Loudness = LOUD_VOLUME; } else { Loudness = SOFT_VOLUME; }
            if (Sound != "")
            {   llStopSound();                              // stop previous sound
                llLoopSound(Sound,Loudness);                // change volume of sound
            }
            ////llOwnerSay("Sound change: " + Sound + " " + (string)Loudness);  // ***TEMP***
        }
    }
    
    changed(integer change)
    {   
        if((change & CHANGED_LINK) == CHANGED_LINK)         // rider got on or off
        {   key newsitting = llAvatarOnSitTarget();         // this is the driver, not any passengers
            if((newsitting != NULL_KEY && sitting == NULL_KEY) && !Active)            // driver got on
            {   if ((LockStatus == MSG_LOCKED) && (newsitting != llGetOwner())) // if owner user only
                {   llShout(0,"Get your own bike! Buy one at 2RAW EXTREME");
                    llUnSit(newsitting);                    // eject non-owner
                    return;
                }
                startup();                                  // start up bike               
            } else if ((newsitting == NULL_KEY) && !Active) {  // no driver, not active
                integer linknum;                            // count sitters just for message
                integer primcount = llGetNumberOfPrims();   // do once before loop
                for (linknum = 1; linknum <= primcount; linknum++)    // check all links for sitters
                {   key avatar = llAvatarOnLinkSitTarget(linknum);
                    if (avatar != NULL_KEY)                     // found a seated avatar
                    {   llWhisper(0,"You're on the passenger seat!");
                        return;
                    }
                }
                return;                                     // wait until driver gets on                                    
            } else if((newsitting == NULL_KEY) && Active)
            {
                handlechanged(change);
                shutdown(TRUE);                             // shut down bike and release
                if (LockStatus == MSG_DEMO)                 // if this was a demo
                {   llSay(0,"Thanks for trying this demo bike. Buy one at 2RAW EXTREME!");
                    llDie();                                // bike disappears
                }
                return;                                     // we want Shutdown to be the last logged event
            }
        }
        if (change & CHANGED_REGION) { Need_Camera_Reset = TRUE; } // reset camera after region change settles
        handlechanged(change);                                          // handle
    }
    
    run_time_permissions(integer perms)
    {
        if(perms == (Permission_Set))
        {
            Active = TRUE;
            Linear = <0,0,DOWNFORCE>;
            Angular = <0,0,0>;
            llStopAnimation("sit");
            NextAnim = IdleAnim;
            CurrentAnim = IdleAnim;
            llStartAnimation(IdleAnim);
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_DOWN | CONTROL_UP | CONTROL_RIGHT | CONTROL_LEFT | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT, TRUE, FALSE);
            set_camera_params();    // set up camera
            logrx(LOG_DEBUG, "PERMS", "Got permissions", 0.0);
        }
    }
    
    control(key id, integer levels, integer edges)
    {   ////llOwnerSay("Levels: " + (string) levels + "  Edges: " + (string) edges); // ***TEMP***
        Angular.x=0;
        Angular.z=0;
        SpeedVec = llGetVel() / llGetRot();
        //  The way this is coded, engine control happens on edge events only.
        //  Adjusting Linear power has to be symmetrical.
        if ((edges & levels) & CONTROL_RIGHT)                   // shift-right => upshift
        {   if (!Active)                                        // if not running, start
            {   llWhisper(0, "Engine on.");
                startup();
                return;
            }
            if((Gear + 1) < NumGears)
            {
                ++Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
            }
        } else if ((edges & levels) & CONTROL_LEFT) // shift-left => downshift
        {
            if (Gear > 0)
            {
                --Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
            } else {                                            // downshift from gear 1 is a shutdown
                float speed = llVecMag(llGetVel());             // speed, absolute
                if (speed < TINY)                               // if stopped
                {   llWhisper(0, "Engine off.");                // turning engine off
                    shutdown(FALSE);                            // shut down bike
                    return;
                }
                
            }
        }
        if (!Active & (edges & levels != 0))                    // any arrow key press while stopped
        {   llWhisper(0,"Shift right arrow to start engine.");
            return;
        }
        if ((edges & levels) & CONTROL_FWD)
        {
            Linear.x += ForwardPower;
            NewSound = 1;
        }else if ((edges & ~levels) & CONTROL_FWD)
        {   
            Linear.x -= ForwardPower;
            NewSound = 1;
        }
        if ((edges & levels) & CONTROL_BACK)
        {
            Linear.x += ReversePower;
            NewSound = 1;
        } else if ((edges & ~levels) & CONTROL_BACK)
        {
            Linear.x -= ReversePower;
            NewSound = 1;
        }
        if(NewSound)
        {
            if(Linear.x) Sound = RunSound;
            else Sound = IdleSound;
        }
        if (llFabs(SpeedVec.x) < 0.5)
        {
            if (levels & CONTROL_ROT_LEFT) CurDir = DIR_LEFT;
            else if(levels & CONTROL_ROT_RIGHT) CurDir = DIR_RIGHT;
            else CurDir = DIR_NORM;
            Angular.z = 0.0;
        }else{
            if(SpeedVec.x < 0.0)
            {
                Forward = -1;
                SpeedVec.x *= -TurnSpeedAdjust;
            }else{
                Forward = 1;
                SpeedVec.x *= TurnSpeedAdjust;
            }
            if (levels & CONTROL_ROT_LEFT)
            {
                CurDir = DIR_LEFT;
                Angular.z = TurnPower ;
                Angular.x=-BankPower;
            }else if ((edges & ~levels) & CONTROL_ROT_LEFT)
            {
                CurDir = DIR_NORM;
                Angular.z = 0;
                Angular.x=0;
            }
            if(levels & CONTROL_ROT_RIGHT)
            {
                CurDir = DIR_RIGHT;
                Angular.z = -TurnPower;
                Angular.x =BankPower;
            }else if ((edges & ~levels) & CONTROL_ROT_RIGHT)
            {
                CurDir = DIR_NORM;
                Angular.z = 0;
                Angular.x=0;
            }
        }
         
        if(levels & (CONTROL_ROT_RIGHT))
        {
            Angular.z -= TurnPower;
            Angular.x=BankPower;
        }
        if(levels & (CONTROL_ROT_LEFT))
        {
            Angular.z +=TurnPower;
            Angular.x=-BankPower;
        }
        // Wheelie Functionality added by Relee Baysklef
        if(levels & CONTROL_UP)
        {            
            Angular.y -= 75;
        }
        if(edges & ~levels & CONTROL_UP)
        {            
            Angular.y = 0;
        }
        llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION,Angular);
    }
        
    timer()                                         // every 100ms when running
    {   integer stat = handletimer();               // handle timer event
        //  Check for avatar out of position and shut down if total fail.
        //  Prevents runaway bike
        if (stat == TICK_FAULT)                     // region crossing failure
        {   llSetVelocity(<0,0,0>,TRUE);            // force stop
            shutdown(TRUE);                         // force shutdown
            return;
        }
        if (stat == TICK_CROSSSTOPPED)              // if stopped at region crossing
        {   return;  }                              // still crossing, just wait
        integer resetanim = FALSE;
        if (!ifnotperms(Permission_Set))            // if safe to set animations
        {   if (Need_Camera_Reset)      // reset on first good tick after sim crossing
            {   set_camera_params();    // reset camera to avoid jerks at sim crossings
                //  We restart the driving animation after every sim crossing, in case
                //  it was lost, which happens. This does not result
                //  in the animation running more than once.
                resetanim = TRUE;                       // force animation reset
            }
            if (CurrentAnim != NextAnim || resetanim)                // set next animation if needed
            {   if (CurrentAnim != "")
                {   llStopAnimation(CurrentAnim);
                    CurrentAnim = "";
                }      
                if (NextAnim != "")
                {   llStartAnimation(NextAnim); }
                CurrentAnim = NextAnim;
            }
        }    
        //  Speed control - slows down for double region crossings, but only if really close
        vector pos = llGetPos();
        pos.z = 0.0;                                // only care about XY
        vector vel = llGetVel();
        vel.z = 0.0;
        float speed = llVecMag(vel);                        // speed
        float maxspeed = maxspeedforregioncross(pos, vel);
        if (speed > maxspeed)
        {   if (maxspeed < lowestslowmsg * 0.8)             // prevent excessive msg rate to server
            {   logrx(LOG_NOTE, "SLOW", "Slowing for double region cross to ", maxspeed);
                lowestslowmsg = maxspeed;                   // slowest limit speed we loggged
            }
            vector dir = llVecNorm(vel);                    // direction of movement
            llSetVelocity(dir * maxspeed, FALSE);           // forcibly slow. Non-realistic             
        } else {                                            // not slowing
            if (maxspeed >= HUGE)                           // if no longer slowing
            {   lowestslowmsg = HUGE;  }                    // reset message spam prevention
            if (Linear != <0.0,  0.0, DOWNFORCE>)
            {   vector wlinear = Linear;        // working linear power
                //  Don't apply full acceleration if we're braking for a region crossing, but 
                //  use normal steering control. We force very low power in that situation.
                if (maxspeed < HUGE && wlinear.x > 0.0) { wlinear.x = COAST_POWER; } 
                llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <wlinear.x,wlinear.y,0.0>);
                llApplyImpulse(<wlinear.x,wlinear.y,DOWNFORCE>, TRUE);
            }
        }
        if(Angular != <0.0, 0.0, 0.0>) { llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, Angular); }
        //  Set next animation
        if (speed > TINY) 
        {   NextAnim = DrivingAnim; } 
        else 
        {   NextAnim = IdleAnim; }

        if(NewSound)
        {
            llStopSound();
            NewSound = 0;
            llLoopSound(Sound, Loudness);
        }
    }
}

