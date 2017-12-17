//  Motorcycle driving script
//  Automatic region crossing handling
//  EXPERIMENTAL
//
//  Animats
//  December, 2017
//  Derived from a motorcycle script of unknown origin.

//  Constants
float TINY = 0.0001;                // small value to prevent divide by zero
float HUGE = 100000000000.0;        // huge number bigger than any distance
float REGION_SIZE = 256.0;          // SL region size


//    Basic settings
                                    // region cross speed control
float BRAKING_TIME = 1.5;           // allow this braking time at boundary
float REGION_BRAKE = 2.0;           // how hard to brake
float REGION_CROSS_SPEED = 5.0;     // max speed at region cross

float BankPower=6000;
float       ForwardPower; //Forward power
list        ForwardPowerGears = [10,20,26,30,38,150];
float       ReversePower = -14; //Reverse power
float       TurnPower =1000; //Turning power
float       TurnSpeedAdjust = 0.7; //how much effect speed has on turning, higher numbers effect more. 0.0 to disable

float       FlightForwardPower = 24;
float       FlightReversePower = 16;
float       FlightUpPower = 14;
float       FlightDownPower = 14;
float       FlightStrafePower = 12;
float       FlightTurnPower = 4;

integer Back=FALSE;


string      SitText = "Drive"; //Text to show on pie menu
string      NonOwnerMessage = "GET YOUR OWN BIKE, SORRY !."; //Message when someone other than owner tries to sit

string      OwnerName;
integer     ListenCh = 4;
string      HornCommand = "h";
string      RevCommand = "r";
string      IdleCommand = "i";
string      StopCommand = "stop";
string      FlightCommand = "fly";
string      GroundCommand = "land";

string      FlightSound = "plane";
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
integer     DIR_FLIGHT = 105;

integer locked=TRUE;
integer burnout=FALSE;
integer TurnSpeedNum=1;
list turn_speeds=[.15,.12,.10,.01];
float TurnSpeed;

//  Utility functions
float min(float a, float b)
{   if (a < b) { return(a); } else { return(b); }}

//
//  Where will a 2D XY vector from p in direction v hit a box of [0..boxsize, 0..boxsize]?
//  Returns distance. Hit point is pos + distance*norm(dir)
float line_box_intersection(vector pos, vector dir, float boxsize)
{   dir.z = 0;
    pos.z = 0;
    dir = llVecNorm(dir);
    //  p = pos + dir * dist;               p is a point on the edge
    float dist1 = HUGE;                     // distance to X bound
    float dist2 = HUGE;                     // distance to Y bound
    //  Check Y axis edges 
    //  p1.x = pos.x + dir.x * dist
    if (dir.x < -TINY)                      // will hit on 0 edge, p1.x = 0
    {   dist1 = -pos.x/dir.x; }             // distance to x=0 edge        
    else if (dir.x > TINY)                // will hit on boxsize edge, p1.x = boxsize
    {   dist1 = (-(pos.x - boxsize))/dir.x; }   // boxsize = pos.x + dir.x * dist1
    //  Check X axis edges
    if (dir.y < TINY)
    {   dist2 = -pos.y/dir.y; }
    else if (dir.y > TINY)
    {   dist2 = (-(pos.y - boxsize))/dir.y; }
    if (dist1 < dist2) { return(dist1);}
    return(dist2);                  // return minimum distance       
}
integer nth = 0;    // ***TEMP***
//  Compute time to next sim crossing on current heading
float time_to_sim_cross()          
{   vector vel = llGetVel();        // get velocity
    vel.z = 0.0;                    // we only care about XY
    float speed = llVecMag(vel);    // calc speed
    if (speed < TINY) { return(HUGE); } // not moving, no problem
    vector pos = llGetPos();        // get position within region
    float dist = line_box_intersection(llGetPos(), vel, REGION_SIZE); // dist to edge
    nth++;  // ***TEMP DEBUG***
    if (nth % 10 == 0) // ***TEMP DEBUG***
    {   llOwnerSay((string)dist + "m to boundary, " + (string)speed + "m/sec."); } // ***TEMP***
    return(dist / speed);           // time to sim cross   
}

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


default
{
    state_entry()
    {
    
        Owner = llGetOwner();
        OwnerName = llKey2Name(Owner);
        TurnSpeedAdjust *= 0.01;
        ForwardPower = llList2Integer(ForwardPowerGears, 0);
        NumGears = llGetListLength(ForwardPowerGears);
        llSetSitText(SitText);
        llCollisionSound("", 0.0);
        llSitTarget(<.30,0.0,0.32>, llEuler2Rot(<0,-.15,0> ));
        TurnSpeed=llList2Float(turn_speeds, TurnSpeedNum-1);
        state Ground;
    }
}

state Ground
{
    state_entry()
    {
        llListen(ListenCh, "", NULL_KEY, "");
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
            Linear = <0,0,-2>;
    
        }
        set_vehicle_params();
    }
    
    on_rez(integer param)
    {
        llResetScript();
    }
    
    listen(integer channel, string name, key id, string message)
    {
        if(llGetOwnerKey(id) != Owner && locked) return;
        message = llToLower(message);
         if(message == StopCommand) llStopSound();
         else if (message=="start") llLoopSound("idle",1);
        else if((message == FlightCommand) && Active) state Flight;
        else if (message=="lock") locked=TRUE;
        else if (message=="unlock") locked=FALSE;
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
    {   if (change & CHANGED_REGION)            // if in new region
        {   float speed = llVecMag(llGetVel()); // get velocity
            llOwnerSay("Speed at region cross: " + (string)speed); // ***TEMP***
        }
        if((change & CHANGED_LINK) == CHANGED_LINK) 
        {
            sitting = llAvatarOnSitTarget();
            if((sitting != NULL_KEY) && !Active)
            {
                if(sitting != llGetOwner() && locked)
                {
                    llWhisper(0, NonOwnerMessage);
                    llUnSit(sitting);
                }else{
                    llSay(9890,"s " +(string)TurnSpeedNum);
                    llWhisper(11,"started");
                    llRequestPermissions(sitting, PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS | PERMISSION_CONTROL_CAMERA);
                    llTriggerSound(StartupSound, 1.0);
                    llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
                    llSetPos(llGetPos() + <0,0,0.15>);
                    llSetStatus(STATUS_PHYSICS, TRUE);
                    SimName = llGetRegionName();
                    llLoopSound(IdleSound,1);
                    llSetTimerEvent(0.1);
                    CurDir = DIR_NORM;
                    LastDir = DIR_NORM;
                }
            }else if((sitting == NULL_KEY) && Active)
            {
                llSetTimerEvent(0.0);
                llStopAnimation(DrivingAnim);
                Active = 0;
                llStopSound();
                llSetStatus(STATUS_PHYSICS, FALSE);
                llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
                llReleaseControls();
            }
        }
    }
    
    run_time_permissions(integer perms)
    {
        if(perms == (PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS | PERMISSION_CONTROL_CAMERA))
        {
            Active = 1;
            Linear = <0,0,-2>;
            Angular = <0,0,0>;
            llStopAnimation("sit");
            llStartAnimation(DrivingAnim);
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_DOWN | CONTROL_UP | CONTROL_RIGHT | CONTROL_LEFT | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT, TRUE, FALSE);
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
        }
    }
    
    control(key id, integer levels, integer edges)
    { Back=FALSE;
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
        //if((edges & levels & CONTROL_UP))
        // Switched this line for the current one. Now uses shift+left
        // for gear shifts
        if((edges & levels & CONTROL_LEFT))
        {
            if((Gear + 1) != NumGears)
            {
                ++Gear;
                llWhisper(0, "Gear " + (string)(Gear + 1));
                ForwardPower = llList2Float(ForwardPowerGears, Gear);
                if(Linear.x > 0) Linear.x = ForwardPower;
            }
        }else if((edges & levels & CONTROL_RIGHT))
        //else if((edges & levels & CONTROL_DOWN)) Relee Baysklef Edit:
        // Switched this line for the current one. Now uses shift+right
        // for gear shifts
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
    
    moving_end()
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
    
    timer()
    {  
        if(Linear != <0.0,  0.0, -2.0>)
        {   vector wlinear = Linear;        // working linear power
            float timetocross = time_to_sim_cross(); // time to sim crossing
            if (timetocross < BRAKING_TIME)         // if time to brake
            {   wlinear.x = min(wlinear.x, REGION_BRAKE);   // use brake value
            }
            llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <wlinear.x,wlinear.y,0.0>);
            llApplyImpulse(<wlinear.x,wlinear.y,-1.0>, TRUE);
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

state Flight
{
    state_entry()
    {
        llWhisper(11,"takeoff");
        Linear = <0,0,0>;
        llStopSound();
        llMessageLinked(LINK_ALL_CHILDREN, DIR_FLIGHT, "", NULL_KEY);
        llLoopSound(FlightSound, 1.0);
        llSetStatus(STATUS_PHYSICS, FALSE);
        llSetPos(llGetPos() + <0,0,0.25>);
        vector rot = llRot2Euler(llGetRot());
        llSetRot(llEuler2Rot(<0,0,rot.z>));
        llListen(ListenCh, "", NULL_KEY, "");
        llSetVehicleType(VEHICLE_TYPE_AIRPLANE);
        
        // linear friction
        llSetVehicleVectorParam(VEHICLE_LINEAR_FRICTION_TIMESCALE, <100.0, 100.0, 100.0>);
        
        // uniform angular friction
        llSetVehicleFloatParam(VEHICLE_ANGULAR_FRICTION_TIMESCALE, 0.5);
        
        // linear motor
        llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <0.0, 0.0, 0.0>);
        llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_TIMESCALE, 1.0);
        llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_DECAY_TIMESCALE, 1.0);
        
        // angular motor
        llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, <0.0, 0.0, 0.0>);
        llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed*3);
        llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_DECAY_TIMESCALE, .1);
        
        // hover
        llSetVehicleFloatParam(VEHICLE_HOVER_HEIGHT, 0.0);
        llSetVehicleFloatParam(VEHICLE_HOVER_EFFICIENCY, 0.0);
        llSetVehicleFloatParam(VEHICLE_HOVER_TIMESCALE, 360.0);
        llSetVehicleFloatParam(VEHICLE_BUOYANCY, 0.988);
        
        // linear deflection
        llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_EFFICIENCY, 0.0);
        llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_TIMESCALE, 1.0);
        
        // angular deflection
        llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_EFFICIENCY, 0.25);
        llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_TIMESCALE, 100.0);
        
        // vertical attractor
        llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_EFFICIENCY, 0.5);
        llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_TIMESCALE, 1.0);
        
        // banking
        llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, 0.0);
        llSetVehicleFloatParam(VEHICLE_BANKING_MIX, 1.0);
        llSetVehicleFloatParam(VEHICLE_BANKING_TIMESCALE, 1.0);
        
        // default rotation of local frame
        llSetVehicleRotationParam(VEHICLE_REFERENCE_FRAME, <0.00000, 0.00000, 0.00000, 0.00000>);
        
        // removed vehicle flags
        llRemoveVehicleFlags(VEHICLE_FLAG_NO_DEFLECTION_UP | VEHICLE_FLAG_HOVER_WATER_ONLY | VEHICLE_FLAG_HOVER_TERRAIN_ONLY | VEHICLE_FLAG_HOVER_UP_ONLY | VEHICLE_FLAG_LIMIT_MOTOR_UP | VEHICLE_FLAG_LIMIT_ROLL_ONLY);
        
        // set vehicle flags
        llSetVehicleFlags(VEHICLE_FLAG_HOVER_GLOBAL_HEIGHT);
        llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_LEFT | CONTROL_RIGHT | CONTROL_ROT_LEFT | CONTROL_ROT_RIGHT | CONTROL_UP | CONTROL_DOWN | CONTROL_LBUTTON, TRUE, FALSE);
        
        llSetStatus(STATUS_PHYSICS, TRUE);
    }
    
    listen(integer channel, string name, key id, string message)
    {
        if(llGetOwnerKey(id) != Owner && locked) return;
        message = llToLower(message);
        if(message == GroundCommand) state Ground;
        else if (message== "lock") locked=TRUE;
        else if (message=="unlock") locked=FALSE;
                if (message=="turn speed 1")      { TurnSpeedNum=1;TurnSpeed=llList2Float(turn_speeds,0);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed*3);}
        else if (message=="turn speed 2") { TurnSpeedNum=2;TurnSpeed=llList2Float(turn_speeds,1);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed*3);}
        else if (message=="turn speed 3") { TurnSpeedNum=3;TurnSpeed=llList2Float(turn_speeds,2);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed*3);}
      else if (message=="turn speed 4") {TurnSpeedNum=4; TurnSpeed=llList2Float(turn_speeds,3);llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, TurnSpeed*3);}
    }
    
    control(key name, integer levels, integer edges)
    {
        if((levels & CONTROL_LBUTTON))
        {
            llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <0,0,0>);
            llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, <0,0,0>);
            llSetStatus(STATUS_PHYSICS, FALSE);
            llSleep(0.1);
            llSetStatus(STATUS_PHYSICS, TRUE);
            return;
        }
        
        if((edges & levels & CONTROL_UP)) Linear.z += FlightUpPower;
        else if((edges & ~levels & CONTROL_UP)) Linear.z = 0.0;
        
        if((edges & levels & CONTROL_DOWN)) Linear.z -= FlightDownPower;
        else if((edges & ~levels & CONTROL_DOWN)) Linear.z = 0.0;
        
        if((edges & levels & CONTROL_FWD)) Linear.x += FlightForwardPower;
        else if((edges & ~levels & CONTROL_FWD)) Linear.x = 0.0;
        
        if((edges & levels & CONTROL_BACK)) Linear.x -= FlightReversePower;
        else if((edges & ~levels & CONTROL_BACK)) Linear.x = 0.0;
        
        if((edges & levels & CONTROL_LEFT)) Linear.y += FlightStrafePower;
        else if((edges & ~levels & CONTROL_LEFT)) Linear.y = 0.0;
        
        if((edges & levels & CONTROL_RIGHT)) Linear.y -= FlightStrafePower;
        else if((edges & ~levels & CONTROL_RIGHT)) Linear.y = 0.0;
        
        if((edges & levels & CONTROL_ROT_LEFT)) Angular.z = FlightTurnPower;
        else if((edges & ~levels & CONTROL_ROT_LEFT)) Angular.z = 0;
        
        if((edges & levels & CONTROL_ROT_RIGHT)) Angular.z = -FlightTurnPower;
        else if((edges & ~levels & CONTROL_ROT_RIGHT)) Angular.z = 0;
    }
    
    changed(integer change)
    {
        if((change & CHANGED_LINK) == CHANGED_LINK)
        {
            sitting = llAvatarOnSitTarget();
            if(sitting == NULL_KEY)
            {
                llSetTimerEvent(0.0);
                llStopAnimation(DrivingAnim);
                Active = 0;
                llStopSound();
                llSetStatus(STATUS_PHYSICS, FALSE);
                llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
                llReleaseControls();
                state Ground;
            }
        }
    }
    
    timer()
    {
        llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, Linear);

        llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, Angular);
    }
}

