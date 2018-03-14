//  Motorcycle driving script
//  Automatic region crossing trouble prevention
//  EXPERIMENTAL.
//
//  Animats
//  March, 2018
//  Derived from a motorcycle script of unknown origin.
#include "regionrx.lsl"            // region crossing system
//  Constants
float TINY = 0.0001;                // small value to prevent divide by zero
float HUGE = 100000000000.0;        // huge number bigger than any distance
float REGION_SIZE = 256.0;          // SL region size
integer LIGHT_CONTROL_CH = 1102;    // light on/off
integer LISTEN_CH = 4;              // inter-script communication

//    Basic settings
                                    // region cross speed control
float BRAKING_TIME = 0;           // allow this braking time at boundary
float AVATAR_MAX_DIST_TO_BIKE = 1.0;// avatar in trouble if sit pos wrong
integer SIT_TROUBLE_MAX = 50;         // in trouble if out of position this many
float SPEED_STATIONARY = 0.05;      // 5cm/sec is "stopped"
vector SIT_TARGET_POS = <.30,0.0,0.32>; // Sit target pos
vector SIT_TARGET_ROT = <0,-.15,0>;     // Sit target rotation (radians)
integer TIMER_SIT_CHECK_EVERY = 5;  // check sit situation every N ticks
float SIT_FAIL_SECS = 10.0;         // in trouble if out of position this many

float MIN_SAFE_DOUBLE_REGION_CROSS_TIME = 0.5;      // min time between double region crosses at speed
float MIN_BRAKE_SPEED = 0.2;                        // minimum speed we will ever brake to
float COAST_POWER = 5.0;                            // power to use when coasting across a region crossing

float DOWNFORCE = -10.0;                             // was 2.0

float       BankPower=6000;
float       ForwardPower; //Forward power
list        ForwardPowerGears = [10,20,26,30,38,150];
float       ReversePower = -14; //Reverse power
float       TurnPower =1000; //Turning power
float       TurnSpeedAdjust = 0.7; //how much effect speed has on turning, higher numbers effect more. 0.0 to disable
integer     Permission_Set = 0; // must initialize at startup
integer     Need_Camera_Reset = FALSE;  // camera reset needed?
integer     Back=FALSE;                 // going backwards
float       DistanceTraveled = 0.0;     // total distance traveled
vector      PrevPos = <0,0,0>;          // previous pos for distance calc

string      SitText = "Drive"; //Text to show on pie menu

string      HornCommand = "h";
string      RevCommand = "r";
string      IdleCommand = "i";
string      StopCommand = "stop";

string      HornSound = "dukes";
string      IdleSound = "idle"; //Sound to play when idling
string      RunSound = "run"; //Sound to play when the gas in pressed
string      RevSound = "rev";
string      StartupSound = "starter"; //Sound to play when owner sits
string      DrivingAnim = "driving generic"; //Animation to play when owner sits

//Other variables
key         Owner;
integer     NumGears;
integer     Gear = 0;
integer     NewSound;
string      Sound;
integer     CurDir;
integer     LastDir;
integer     Forward;
vector      SpeedVec;
vector      Linear;
vector      Angular;
integer     Active;
key         sitting;
integer     Moving;
string      SimName;

integer     DIR_STOP = 100;
integer     DIR_START = 101;
integer     DIR_NORM = 102;
integer     DIR_LEFT = 103;
integer     DIR_RIGHT = 104;

integer locked=TRUE;
integer burnout=FALSE;
integer TurnSpeedNum=1;
list turn_speeds=[.15,.12,.10,.01];
float TurnSpeed;

#include "vehicleutils.lsl"

set_vehicle_params()                // set up SL vehicle system
{
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
{   initregionrx(LOG_NOTE);                     // init region crossing system
    llSay(9890,"s " +(string)TurnSpeedNum);
    llWhisper(11,"started");
    llRequestPermissions(sitting, Permission_Set);
    llTriggerSound(StartupSound, 1.0);
    llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
    llSetPos(llGetPos() + <0,0,0.15>);
    llSetStatus(STATUS_PHYSICS, TRUE);
    SimName = llGetRegionName();
    llLoopSound(IdleSound,1);
    llSetTimerEvent(TIMER_INTERVAL);
    CurDir = DIR_NORM;
    LastDir = DIR_NORM;
    DistanceTraveled = 0.0;
    PrevPos = llGetPos() + llGetRegionCorner(); // global pos
    PrevPos.z = 0.0;                            // no height, only for distance comp.
    logrx(LOG_NOTE,"STARTUP", "",0.0);
}
//  shutdown -- shut down bike
shutdown()
{
    llSetTimerEvent(0.0);
    llStopAnimation(DrivingAnim);
    llStopSound();
    llSetStatus(STATUS_PHYSICS, FALSE);
    llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
    llReleaseControls();
    logrx(LOG_NOTE,"SHUTDOWN", "Distance traveled (km)",DistanceTraveled/1000.0);
}


default
{
    state_entry()
    {
        Permission_Set = PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS | PERMISSION_CONTROL_CAMERA;
        Owner = llGetOwner();
        TurnSpeedAdjust *= 0.01;
        ForwardPower = llList2Integer(ForwardPowerGears, 0);
        NumGears = llGetListLength(ForwardPowerGears);
        Active = 0;
        llSetSitText(SitText);
        llCollisionSound("", 0.0);
        llSitTarget(SIT_TARGET_POS, llEuler2Rot(SIT_TARGET_ROT));
        TurnSpeed=llList2Float(turn_speeds, TurnSpeedNum-1);
        state Ground;
    }
}

state Ground
{
    state_entry()
    {   llListen(LISTEN_CH, "", NULL_KEY, "");      // listen for other bike scripts

        llStopSound();
        llSay (11,"landed");
        if(!Active)
        {
            llSetStatus(STATUS_PHYSICS, FALSE);
            llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
            llUnSit(llAvatarOnSitTarget());
        }else{
            SimName = llGetRegionName();
            llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
            llMessageLinked(LINK_ALL_CHILDREN, DIR_NORM, "", NULL_KEY);
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
    
    listen(integer channel, string name, key id, string message)
    {
        message = llToLower(message);                           // work in lower case
        if (channel != LISTEN_CH) return;                      // channel is bogus
        if(llGetOwnerKey(id) != Owner && locked) return;
        if(message == StopCommand) llStopSound();
        else if (message=="start") llLoopSound("idle",1);
        else if (message=="gear up") 
        {          
                if (Gear>=NumGears-1) return;      
                 ++Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;}
        else if (message=="gear down"){
                if (Gear<=0) return;
                --Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
        }
        if (message=="turn speed 1")      { TurnSpeedNum=1;TurnSpeed=llList2Float(turn_speeds,0);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed);}
        else if (message=="turn speed 2") { TurnSpeedNum=2;TurnSpeed=llList2Float(turn_speeds,1);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed);}
        else if (message=="turn speed 3") { TurnSpeedNum=3;TurnSpeed=llList2Float(turn_speeds,2);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed);}
        else if (message=="turn speed 4") {TurnSpeedNum=4; TurnSpeed=llList2Float(turn_speeds,3);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed);}
    }
    
    changed(integer change)
    {   
        if((change & CHANGED_LINK) == CHANGED_LINK)         // rider got on or off
        {   sitting = llAvatarOnSitTarget();                // this is the driver, not any passengers
            if((sitting != NULL_KEY) && !Active)
            {
                startup();              // start up bike                // new avatar sitting
            } else if((sitting == NULL_KEY) && Active)
            {
                shutdown();         // shut down bike and release
                Active = 0;
            }
        }
        handlechanged(change);                                          // handle
    }
    
    run_time_permissions(integer perms)
    {
        if(perms == (Permission_Set))
        {
            Active = 1;
            Linear = <0,0,DOWNFORCE>;
            Angular = <0,0,0>;
            llStopAnimation("sit");
            llStartAnimation(DrivingAnim);
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_DOWN | CONTROL_UP | CONTROL_RIGHT | CONTROL_LEFT | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT, TRUE, FALSE);
            set_camera_params();    // set up camera
            logrx(LOG_DEBUG, "PERMS", "Got permissions", 0.0);
        }
    }
    
    control(key id, integer levels, integer edges)
    {   
        Back=FALSE;
        if (levels & CONTROL_BACK)Back=TRUE;
        Angular.x=0;
        Angular.z=0;
        if(!Moving)
        {
            Moving = 1;
            llSetStatus(STATUS_PHYSICS, TRUE);
            llStartAnimation(DrivingAnim);
        }
        SpeedVec = llGetVel() / llGetRot();
        if((edges & levels & CONTROL_RIGHT)) // shift-right => upshift
        {
            if((Gear + 1) != NumGears)
            {
                ++Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
            }
        }else if((edges & levels & CONTROL_LEFT)) // shift-left => downshift
        {
            if((Gear - 1) != -1)
            {
                --Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
            }
        }
        if((edges & levels & CONTROL_FWD))
        {
            Linear.x += ForwardPower;
            NewSound = 1;
        }else if((edges & ~levels & CONTROL_FWD))
        {
            Linear.x -= ForwardPower;
            NewSound = 1;
        }
        if((edges & levels & CONTROL_BACK))
        {
            Linear.x += ReversePower;
            NewSound = 1;
        }else if((edges & ~levels & CONTROL_BACK))
        {
            Linear.x -= ReversePower;
            NewSound = 1;
        }
        if(NewSound)
        {
            if(Linear.x) Sound = RunSound;
            else Sound = IdleSound;
        }
        if(llFabs(SpeedVec.x) < 0.5)
        {
            if(levels & CONTROL_ROT_LEFT) CurDir = DIR_LEFT;
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
            if(levels & CONTROL_ROT_LEFT)
            {
                CurDir = DIR_LEFT;
                Angular.z = TurnPower ;
                Angular.x=-BankPower;
            }else if((edges & ~levels & CONTROL_ROT_LEFT))
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
            }else if((edges & ~levels & CONTROL_ROT_RIGHT))
            {
                CurDir = DIR_NORM;
                Angular.z = 0;
                Angular.x=0;
            }
        }
         
                 //if(levels & (CONTROL_RIGHT|CONTROL_ROT_RIGHT))
                 if(levels & (CONTROL_ROT_RIGHT))
        {
            Angular.z -= TurnPower;
            Angular.x=BankPower;
        }
        //if(levels & (CONTROL_LEFT|CONTROL_ROT_LEFT))
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
        if (Back) llMessageLinked(LINK_ALL_OTHERS,10,"back",NULL_KEY);
        else llMessageLinked(LINK_ALL_OTHERS,10,"regular",NULL_KEY);
    }
    
    moving_end()                                    // legacy code. Necessary?
    {
        if(llGetRegionName() == SimName)
        {
            Moving = 0;
            llSetStatus(STATUS_PHYSICS, FALSE);
            if (Active){llStopAnimation(DrivingAnim);}
        }else{
            SimName = llGetRegionName();
        }
    }
    
    timer()                                         // every 100ms when running
    {   integer stat = handletimer();               // handle timer event
        vector pos = llGetPos();
        pos.z = 0.0;                                // only care about XY
        vector gpos = pos + llGetRegionCorner();    // global pos
        DistanceTraveled += llVecMag(PrevPos - gpos);// distance traveled add
        PrevPos = gpos;                              // save position
        //  Check for avatar out of position and shut down if total fail.
        //  Prevents runaway bike
        if (stat == TICK_FAULT)                     // region crossing failure
        {   llSetVelocity(<0,0,0>,TRUE);            // force stop
            shutdown();                             // force shutdown
            return;
        }
        if (stat == TICK_CROSSSTOPPED)              // if stopped at region crossing
        {   return;  }                              // still crossing, just wait
        if (Need_Camera_Reset && (!ifnotperms(Permission_Set)))      // reset on first good tick after sim crossing
        {   set_camera_params();    // reset camera to avoid jerks at sim crossings
            //  We restart the driving animation after every sim crossing, in case
            //  it was lost, which happens. This does not result
            //  in the animation running more than once.
            llStartAnimation(DrivingAnim); // reset driving anim
        }
        //  Speed control - slows down for double region crossings, but only if really close
        vector vel = llGetVel();
        vel.z = 0.0;
        float speed = llVecMag(vel);                        // speed
        float maxspeed = maxspeedforregioncross(pos, vel);
        if (speed > maxspeed)
        {   logrx(LOG_NOTE, "SLOW", "Slowing for double region cross to ", maxspeed);
            vector dir = llVecNorm(vel);                    // direction of movement
            llSetVelocity(dir * maxspeed, FALSE);           // forcibly slow. Non-realistic             
        } else {
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
        if(CurDir != LastDir)
        {
            llMessageLinked(LINK_ALL_CHILDREN, CurDir, "", NULL_KEY);
            LastDir = CurDir;
        }
        if(NewSound)
        {
            llStopSound();
            NewSound = 0;
            llLoopSound(Sound, 1.0);
        }
    }
}

