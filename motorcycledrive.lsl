//  Motorcycle driving script
//  Automatic region crossing handling
//  EXPERIMENTAL
//
//  Animats
//  January, 2018
//  Derived from a motorcycle script of unknown origin.

//  Constants
float TINY = 0.0001;                // small value to prevent divide by zero
float HUGE = 100000000000.0;        // huge number bigger than any distance
float REGION_SIZE = 256.0;          // SL region size


//    Basic settings
                                    // region cross speed control
float BRAKING_TIME = 0;           // allow this braking time at boundary
float REGION_BRAKE = 0.0;           // how hard to brake
float REGION_CROSS_SPEED = 500.0;     // max speed at region cross
float AVATAR_MAX_DIST_TO_BIKE = 1.0;// avatar in trouble if sit pos wrong
integer SIT_TROUBLE_MAX = 50;         // in trouble if out of position this many

integer     TimerTick = 0;          // count of timer events
integer     SitTroubleCount = 0;      // timer ticks out of position
float       BankPower=6000;
float       ForwardPower; //Forward power
list        ForwardPowerGears = [10,20,26,30,38,150];
float       ReversePower = -14; //Reverse power
float       TurnPower =1000; //Turning power
float       TurnSpeedAdjust = 0.7; //how much effect speed has on turning, higher numbers effect more. 0.0 to disable
integer     Permission_Set = 0; // must initialize at startup
integer     Region_Cross_Count = 0; // number of region crosses
integer     Start_Test_Time = 0;    // time test started
integer     Need_Camera_Reset = FALSE;  // camera reset needed?

integer     Back=FALSE;


string      SitText = "Drive"; //Text to show on pie menu

integer     ListenCh = 4;
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
    if (dir.y < -TINY)
    {   dist2 = -pos.y/dir.y; }
    else if (dir.y > TINY)
    {   dist2 = (-(pos.y - boxsize))/dir.y; }
    if (dist1 < dist2) { return(dist1);}
    return(dist2);                  // return minimum distance       
}
//  Compute time to next sim crossing on current heading
float time_to_sim_cross()          
{   vector vel = llGetVel();        // get velocity
    vel.z = 0.0;                    // we only care about XY
    float speed = llVecMag(vel);    // calc speed
    if (speed < TINY) { return(HUGE); } // not moving, no problem
    vector pos = llGetPos();        // get position within region
    float dist = line_box_intersection(llGetPos(), vel, REGION_SIZE); // dist to edge
    float timetoedge = dist / speed;// time to next boundary
    if (((TimerTick % 10) == 0) && (timetoedge < 1.5)) // ***TEMP DEBUG***
    {   llOwnerSay((string)dist + "m to boundary, " + (string)speed + "m/sec."); } // ***TEMP***
    return(timetoedge);           // time to sim cross   
}
//  posasstring -- get current position as string
string posasstring()
{   vector pos = llGetPos();
    return(llGetRegionName() 
        + " (" + (string)((integer)pos.x)
        + "," + (string)((integer)pos.y)
        + "," + (string)((integer)pos.z) + ")");
}
//
//  ifsittrouble  - true if avatar position is valid
//
//  Check for everything which can go wrong with the vehicle/avatar
//  relationship.
//
integer ifsittrouble(key avatar)
{   integer trouble = FALSE;
    //  Check for avatar out of position
    if (avatar != NULL_KEY) 
    {   vector vehpos = llGetPos();
        list avatarinfo = llGetObjectDetails(avatar, 
                [OBJECT_POS, OBJECT_ROOT]);
        vector avatarpos = llList2Vector(avatarinfo,0);
        key avatarroot = llList2Key(avatarinfo,1);
        float avatardist = llVecMag(avatarpos - vehpos);
        if (avatardist > AVATAR_MAX_DIST_TO_BIKE)
            {   trouble = TRUE;
                llOwnerSay("Avatar out of position: " + 
                    (string)avatardist + "m from bike.");
            }
        integer agentinfo = llGetAgentInfo(avatar);
        if (agentinfo & (AGENT_SITTING | AGENT_ON_OBJECT) !=
            (AGENT_SITTING | AGENT_ON_OBJECT))
        {   trouble = TRUE;
            llOwnerSay("Avatar not fully seated.");
        }
        //  Check back link from avatar to root prim.
        if (avatarroot == NULL_KEY)
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is null.");
        }
        else if (avatarroot == avatar)
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is to avatar itself.");
        }
        else if (avatarroot != llGetKey())
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is wrong."); 
            llOwnerSay("Avatar link to root: " + 
                (string) avatarroot + "  Veh. root: " +
                (string) llGetKey());
        }
    } else {                    // unseated
        trouble = TRUE;
        llOwnerSay("Avatar not on sit target");
    }
    //  Check for proper permissions
    integer perms = llGetPermissions(); // what perms do we have?
    if (Permission_Set & perms != Permission_Set)
    {   llOwnerSay("Vehicle lost permissions. Have " + (string) perms + 
            " Should have " + (string) Permission_Set);
        trouble = TRUE;
    }
    return trouble;
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
//  shutdown -- shut down bike
shutdown()
{
    llSetTimerEvent(0.0);
    llStopAnimation(DrivingAnim);
    llStopSound();
    llSetStatus(STATUS_PHYSICS, FALSE);
    llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
    llReleaseControls();
    llOwnerSay("Bike shut down after "
        + (string)Region_Cross_Count + " region crossings.");
    llOwnerSay("Final bike location: " + posasstring());
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
            Region_Cross_Count++;               // tally
            llOwnerSay("Speed at region cross " 
                + (string)Region_Cross_Count + ": "  
                + (string)speed);           // ***TEMP***
            Need_Camera_Reset = TRUE;           // schedule a camera reset
        }
        if((change & CHANGED_LINK) == CHANGED_LINK) 
        {
            sitting = llAvatarOnSitTarget();
            if((sitting != NULL_KEY) && !Active)
            {
                llSay(9890,"s " +(string)TurnSpeedNum);
                llWhisper(11,"started");
                llRequestPermissions(sitting, Permission_Set);
                llTriggerSound(StartupSound, 1.0);
                llMessageLinked(LINK_ALL_CHILDREN, DIR_START, "", NULL_KEY);
                llSetPos(llGetPos() + <0,0,0.15>);
                llSetStatus(STATUS_PHYSICS, TRUE);
                SimName = llGetRegionName();
                llLoopSound(IdleSound,1);
                llSetTimerEvent(0.1);
                CurDir = DIR_NORM;
                LastDir = DIR_NORM;
            } else if((sitting == NULL_KEY) && Active)
            {
                shutdown();         // shut down bike and release
                Active = 0;
            }
        }
    }
    
    run_time_permissions(integer perms)
    {
        if(perms == (Permission_Set))
        {
            Active = 1;
            Linear = <0,0,-2>;
            Angular = <0,0,0>;
            llStopAnimation("sit");
            llStartAnimation(DrivingAnim);
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_DOWN | CONTROL_UP | CONTROL_RIGHT | CONTROL_LEFT | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT, TRUE, FALSE);
            set_camera_params();    // set up camera
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
    {   TimerTick++;                    // count timer ticks for debug
        //  Check for avatar out of position
        key avatar = llAvatarOnSitTarget();
        integer sittrouble = ifsittrouble(avatar);
        if (sittrouble)                 // possible malfunction
        {   SitTroubleCount++;       // tally sit troubles
            if (SitTroubleCount > SIT_TROUBLE_MAX)
            {   llOwnerSay("TROUBLE - partial unsit detected.");
                //  ***MORE*** Need to recover.
                //  First approach - unsit avatar and shut down.
                if (avatar != NULL_KEY)
                {   llOwnerSay("Trying to unsit avatar."); 
                    llUnSit(avatar);
                    shutdown(); // shut down bike
                    llUnSit(avatar);
                    state Ground;   // back to ground state
                }
            }
        }
        else 
        {   SitTroubleCount = 0;        // reset count
            if (Need_Camera_Reset)      // reset on first good tick after sim crossing
            {   set_camera_params();    // reset camera to avoid jerks at sim crossings
                ////string currentanim = llGetAnimation(avatar); // get current anim
                ////llOwnerSay("Current animation: " + currentanim); // ***TEMP***
                ////list anims = llGetAnimationList(avatar); // get current anim list
                ////llOwnerSay("Anims: " + llDumpList2String(anims, "," )); // ***TEMP***
                //  We restart the driving animation after every sim crossing, in case
                //  it was lost, which happens. This does notresult
                //  in the animation running more than once.
                llStartAnimation(DrivingAnim); // reset driving anim
            }
        }      

        //  Speed control
        if(Linear != <0.0,  0.0, -2.0>)
        {   vector wlinear = Linear;        // working linear power
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


