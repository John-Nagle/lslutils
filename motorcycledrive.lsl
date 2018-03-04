//  Motorcycle driving script
//  Automatic region crossing trouble prevention
//  EXPERIMENTAL.
//
//  Animats
//  March, 2018
//  Derived from a motorcycle script of unknown origin.

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
float TIMER_INTERVAL = 0.1;           // drive timer rate
integer TIMER_SIT_CHECK_EVERY = 5;  // check sit situation every N ticks
float SIT_FAIL_SECS = 10.0;         // in trouble if out of position this many

float SAFE_CROSS_SPEED = 3.0;       // 3m/sec max speed at double region cross
float MIN_SAFE_DOUBLE_REGION_CROSS_DIST = 5.0;      // slow if we can hit a double region crossing twice within this dist
float MIN_SAFE_DOUBLE_REGION_CROSS_TIME = 0.5;      // min time between double region crosses at speed

integer     TimerTick = 0;          // count of timer events
float       SitTroubleSecs = 0.0;   // timer ticks out of position
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
integer     Back=FALSE;                 // going backwards
integer     SitTrouble = FALSE;      // in sit trouble state
float       RegionCrossStartTime = 0.0;   // timestamp of region crossing
//  Status during region crossing
integer     crossStopped = FALSE;
vector      crossVel;
vector      crossAngularVelocity = <0,0,0>; // always zero for now
float       crossStartTime;             // starts at changed event, ends when avatar in place

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
//  Utility functions
float min(float a, float b)
{   if (a < b) { return(a); } else { return(b); }}
float max(float a, float b)
{   if (a > b) { return(a); } else { return(b); }}
float abs(float a)
{   if (a > 0) { return(a); } else { return(-a); }}
//
//  nearestcorner -- nearest region corner to position
//
//  Will need revision for systems with multiple region sizes
//
vector nearestcorner(vector pos)
{   vector corner = <0,0,0>;
    if (pos.x*2.0 > REGION_SIZE) corner.x = REGION_SIZE;
    if (pos.y*2.0 > REGION_SIZE) corner.y = REGION_SIZE;
    return(corner);
}

integer outsideregion(vector pos)                       // TRUE if outside region
{   return(pos.x < 0.0 || pos.x > REGION_SIZE || pos.y < 0.0 || pos.y > REGION_SIZE); }

//
//  regioncrossingseglength -- length of segment between next two region crossings in dir
//
//  Ignores Z direction.  
//  There is a diagram of this in regioncrossing.md.
//
float regioncrossingseglength(vector pos, vector dir)
{
    vector corner = nearestcorner(pos);             // nearest region corner
    vector dp = pos - corner;                       // work in coord system where corner is <0,0>
    float xcept = HUGE;                             // X intercept
    float ycept = HUGE;                             // Y intercept
    if (abs(dir.x) > TINY)                          // avoid divide by zero
    {   ycept = dp.y + dir.y * (-dp.x / dir.x); }   // Y intercept
    if (abs(dir.y) > TINY)
    {   xcept = dp.x + dir.x * (-dp.y / dir.y); }   // X intercept
    return(llSqrt(xcept*xcept + ycept*ycept));      // return segment distance   
}

//
//  regioncrossingprobe  --  returns length of segment between closest two next region crossings
//
//  The test is made along two test vectors some angle apart
//  Returns HUGE if not meaningful.
//  ***UNUSED*** at present
//
float regioncrossingprobe(vector pos, vector dir, rotation testangle1, rotation testangle2)
{
    vector v1 = dir * testangle1;                   // need two test vectors
    vector v2 = dir * testangle2;                   // 
    return(min(regioncrossingseglength(pos, v1), regioncrossingseglength(pos, v2))); // closest hit
}


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
//
//  slowforregioncross -- do we need to slow down for a region crossing?
//
integer slowforregioncross(vector pos, vector vel)
{   //  Dumb version
    vel.z = 0;                                          // XY plane only
    float speed = llVecMag(vel);                        // how fast are we going
    if (speed < SAFE_CROSS_SPEED) { return(FALSE); }    // slow enough that not a problem
    float disttosimcross = line_box_intersection(pos, vel, REGION_SIZE);    // dist to sim crossing
    float timetoedge = disttosimcross / speed;                    // speed to edge
    if (((TimerTick % 10) == 0) && (timetoedge < 1.5)) // ***TEMP DEBUG***
    {   llOwnerSay((string)disttosimcross + "m to region boundary at " + (string)pos + "  " + (string)speed + "m/sec."); } // ***TEMP***
    if (timetoedge < TIMER_INTERVAL*3.0)                // if very close to sim crossing
    {   float rcseglength = regioncrossingseglength(pos, vel); // distance between next two sim crosses
        float timebetweencrosses = rcseglength / speed; // time for double region cross
        ////llOwnerSay("Region crossing segment length: " + (string)rcseglength + 
        ////    "  time: " + (string)timebetweencrosses); // ***TEMP***
        if (rcseglength < MIN_SAFE_DOUBLE_REGION_CROSS_DIST) // too close together in space
        {   return(TRUE); }
        if (timebetweencrosses < MIN_SAFE_DOUBLE_REGION_CROSS_TIME) // too close together in time
        {   return(TRUE); }
    }
    return(FALSE);
}
//  posasstring -- get current position as string
string posasstring(string region, vector pos)
{   
    return(region
        + " (" + (string)((integer)pos.x)
        + "," + (string)((integer)pos.y)
        + "," + (string)((integer)pos.z) + ")");
}
//
//  ifnotseated  - true if avatar position is valid
//
//  Check for everything which can go wrong with the vehicle/avatar
//  relationship.
//
integer ifnotseated(key avatar)
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
        llOwnerSay("Avatar not on sit target.");
    }
    return trouble;
} 
  
integer ifnotperms() 
{   integer trouble = FALSE;
    //  Check for proper permissions
    integer perms = llGetPermissions(); // what perms do we have?
    if (Permission_Set & perms != Permission_Set)
    {   llOwnerSay("Vehicle lost permissions. Have " + (string) perms + 
            " Should have " + (string) Permission_Set);
        trouble = TRUE;
    }
    return trouble;
}

integer ifsittrouble(key avatar)
{   return(ifnotseated(avatar) || ifnotperms()); }

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
    llSetTimerEvent(TIMER_INTERVAL);
    CurDir = DIR_NORM;
    LastDir = DIR_NORM;
    crossStopped = FALSE;                       // not stopped during region cross
}
//  shutdown -- shut down bike
shutdown()
{
    llSetTimerEvent(0.0);
    llStopAnimation(DrivingAnim);
    llStopSound();
    llSetStatus(STATUS_PHYSICS, FALSE);
    crossStopped = FALSE;                       // not holding at region crossing
    llMessageLinked(LINK_ALL_CHILDREN , DIR_STOP, "", NULL_KEY);
    llReleaseControls();
    llOwnerSay("Bike shut down after "
        + (string)Region_Cross_Count + " region crossings.");
    llOwnerSay("Final bike location: " + posasstring(llGetRegionName(), llGetPos()));
}


default
{
    state_entry()
    {
        Permission_Set = PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS | PERMISSION_CONTROL_CAMERA;
        ////    | PERMISSION_TELEPORT;
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
    {   if (change & CHANGED_REGION)            // if in new region
        {   float speed = llVecMag(llGetVel()); // get velocity
            RegionCrossStartTime = 0.0;
            Region_Cross_Count++;               // tally
            llOwnerSay("Speed at region cross #" 
                + (string)Region_Cross_Count + ": "  
                + (string)speed + " m/s");
            Need_Camera_Reset = TRUE;           // schedule a camera reset
            if (llGetStatus(STATUS_PHYSICS))    // if physics on
            {   crossVel = llGetVel();              // save velocity
                crossAngularVelocity = <0,0,0>;     // there is no llGetAngularVelocity();
                llSetStatus(STATUS_PHYSICS, FALSE); // forcibly stop object
                crossStopped = TRUE;                // stopped during region crossing
                crossStartTime = llGetTime();       // timestamp
            } else {                                // this is bad. A partial unsit usuallly follows
                llOwnerSay("TROUBLE - second region cross started before first one completed, at " 
                    + posasstring(llGetRegionName(), llGetPos()));
            }
        }
        if((change & CHANGED_LINK) == CHANGED_LINK) // rider got on or off
        {
            sitting = llAvatarOnSitTarget();
            if((sitting != NULL_KEY) && !Active)
            {
                startup();              // start up bike                // new avatar sitting
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
            llOwnerSay("Got permissions. Now steering from arrow keys."); // ***TEMP***
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
    {   TimerTick++;                                // count timer ticks for debug
        //  Check for avatar out of position and shut down if total fail.
        //  Prevents runaway bike
        key avatar = llAvatarOnSitTarget();
        if (TimerTick % TIMER_SIT_CHECK_EVERY == 0) // check is expensive, only do once a second or so
        {
            integer sittrouble = ifsittrouble(avatar);
            if (sittrouble)                 // possible malfunction
            {   SitTroubleSecs = SitTroubleSecs + TIMER_INTERVAL * TIMER_SIT_CHECK_EVERY; // time trouble persisted
                if (SitTroubleSecs > SIT_FAIL_SECS) // persistent malfunction
                {   if (!SitTrouble) { llOwnerSay("TROUBLE - partial unsit detected."); }
                    //  Try to recover. Just shuts down
                    SitTrouble = TRUE;
                    llSetVelocity(<0,0,0>,TRUE); // force stop
                    shutdown();                 // force shutdown
                    return;
                }
            }
            else 
            {   
                SitTrouble  = FALSE;
                SitTroubleSecs = 0.0;   
            }     // reset count
        }
        if (Need_Camera_Reset && (!crossStopped) && (!ifnotperms()))      // reset on first good tick after sim crossing
        {   set_camera_params();    // reset camera to avoid jerks at sim crossings
            //  We restart the driving animation after every sim crossing, in case
            //  it was lost, which happens. This does not result
            //  in the animation running more than once.
            llStartAnimation(DrivingAnim); // reset driving anim
        }
        //  Stop temporarily during region crossing until rider catches up.
        if (crossStopped)                               // if stopped at region crossing
        {   if (!ifnotseated(avatar))                   // if avatar is back in place
            {   llSetStatus(STATUS_PHYSICS, TRUE);      // physics back on
                llSetVelocity(crossVel, FALSE);         // use velocity from before
                llSetAngularVelocity(crossAngularVelocity, FALSE);  // and angular velocity
                crossStopped = FALSE;                   // no longer stopped
                float crosstime = llGetTime() - crossStartTime;
                llOwnerSay("Avatar back in place. Region crossing complete in " + (string)crosstime + "secs.");
            } else {
                return;                                 // still crossing, just wait
            }
        }     
        //  Speed control
        vector pos = llGetPos();
        vector vel = llGetVel();
        vel.z = 0.0;
        if (slowforregioncross(pos, vel))
        {   llOwnerSay("Slowing for double region cross at " + (string)pos + "  vel " + (string)llVecMag(vel));
            vector dir = llVecNorm(vel);                    // direction of movement
            llSetVelocity(dir * SAFE_CROSS_SPEED, FALSE); // forcibly slow. Non-realistic             
        } else {
            if (Linear != <0.0,  0.0, -2.0>)
            {   vector wlinear = Linear;        // working linear power
                llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <wlinear.x,wlinear.y,0.0>);
                llApplyImpulse(<wlinear.x,wlinear.y,-1.0>, TRUE);
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

