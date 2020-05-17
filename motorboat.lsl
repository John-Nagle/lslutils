//  Simple boat script with checks for parcel keep-out areas
//
//  Animats Advanced Vehicle Technology
//  Configuration
//
integer SAVED_POSITION_COUNT = 3;               // save this many backup positions
float REGION_SIZE = 256.0;                      // size of all regions
float MOVESPEED = 2.0;                          // speed to move when exiting region when in trouble
float MAX_SPEED = 50.0;                         // top speed
float MAX_SPEED_REVERSE = 8.0;                  // max speed in reverse
float SPEED_INCR = 1.0;                         // increase/decrease this much
float TURN_RATE = 10.0;                         // turn rate when turning


//  Global variables
float speed;                                    // user-set speed
vector angular_motor;                           // user-set turn
//  Data for restricted area recovery
list safe_positions = [];
list safe_rotations = [];
list safe_region_corners = [];

save_safe_position(vector pos, rotation rot, vector regioncorner)           // save last N safe positions
{   if (pos.x < 0.0 || pos.x > REGION_SIZE || pos.y < 0.0 || pos.y > REGION_SIZE) { return; } // do not save out of region points
    if (llGetListLength(safe_positions) >= SAVED_POSITION_COUNT)           // l mit listh growth
    {   safe_positions = llDeleteSubList(safe_positions,0,0);
        safe_rotations = llDeleteSubList(safe_rotations,0,0); 
        safe_region_corners = llDeleteSubList(safe_region_corners,0,0);
    }
    safe_positions += pos;                                                  // add new to list
    safe_rotations += rot;   
    safe_region_corners += regioncorner;
}

string parcelflags2string(integer flags)                                    // print flags for messages
{   string s = "";
    if ((flags & PARCEL_FLAG_ALLOW_FLY) == 0) { s += "No fly. "; }
    if ((flags & PARCEL_FLAG_ALLOW_SCRIPTS) == 0) { s += "No scripts. "; }
    if ((flags & PARCEL_FLAG_ALLOW_ALL_OBJECT_ENTRY) == 0) { s += "No object entry. "; }
    if ((flags & PARCEL_FLAG_USE_ACCESS_GROUP)) { s += "Access allowed for group. "; }
    if ((flags & PARCEL_FLAG_USE_ACCESS_LIST)) { s += "Has access list."; }
    if ((flags & PARCEL_FLAG_USE_BAN_LIST)) { s += "Has ban list."; }
    if ((flags & 0x1000)) { s += "Has 0x01000 flag."; } 
    return(s);
}

//  Are we allowed to enter this parcel at this test point?
//  Only tests for scripts allowed. Testing for object entry is done by physics fail,
//  and testing for avatar entry is not reliable with the current API.
integer parcel_entry_allowed_here(vector sensorpos)
{
    integer parcelflags = llGetParcelFlags(sensorpos);  // get flags at that point 
    if ((parcelflags & PARCEL_FLAG_ALLOW_SCRIPTS) == (PARCEL_FLAG_ALLOW_SCRIPTS))
        { return(TRUE); }                       // no restrictions
    //  Who owns this place?
    list ownerlist = llGetParcelDetails(sensorpos, [PARCEL_DETAILS_OWNER]);
    key ownerkey = llList2Key(ownerlist, 0);
    string ownerstring = "secondlife:///app/agent/" + (string)ownerkey + "/about";
    llOwnerSay("Restricted area owned by '" + ownerstring +  "' at " + (string)sensorpos + ": " + parcelflags2string(parcelflags)); // ***TEMP***
    return(FALSE);                              // for now, say no     
}

key get_parcel_owner(vector pos)                    // get key of owner of parcel
{    list ownerlist = llGetParcelDetails(pos, [PARCEL_DETAILS_OWNER]);
     return(llList2Key(ownerlist, 0));
}

//  Are we allowed to move here?
integer parcel_entry_allowed()
{   return(parcel_entry_allowed_here(llGetPos()));
}

//  Try to recover from restricted areas. Call from timer.
restricted_area_recovery()
{
    integer physicson = llGetStatus(STATUS_PHYSICS);    // is physics on, as it should be?
    ////if (!parcel_entry_allowed()) { llSetStatus(STATUS_PHYSICS, FALSE); physicson = FALSE; } // force physics off
    //  Ban line hit recovery
    if (!physicson)                         // physics is not on. Probably hit a ban line
    {   ////llWhisper(0,"Restricted area. Beginning automatic back-out.");   // note hit ban line
        key ownerkey = get_parcel_owner(llGetPos());          // who owns this place?
        string ownerstring = "secondlife:///app/agent/" + (string)ownerkey + "/about"; // expanded when printed, not here
        if (llGetListLength(safe_positions) > 0)         // if valid pos available
        {   vector last_safe_pos = llList2Vector(safe_positions,0); // get oldest pos available
            rotation last_safe_rot = llList2Rot(safe_rotations,0);
            vector last_safe_region_corner = llList2Vector(safe_region_corners,0);
            vector region_corner = llGetRegionCorner();
            if (region_corner == last_safe_region_corner)                       // if safe position is in same region
            {   llSetPos(last_safe_pos);                                        // move to safe position
                llOwnerSay("Backing out of restricted area by moving to  " + string(last_safe_pos));
                llSetRot(last_safe_rot);
                llSleep(0.5);                                                   // wait for physics-off move
                llSetStatus(STATUS_PHYSICS, TRUE);                              // turn physics back on
            } else {                                                            // we need to move across region boundary
                llOwnerSay("Backing out of restricted area region boundary of " + llGetRegionName());
                //  Try doing this with a physical move
                vector pos = llGetPos();
                vector posglobal = llGetRegionCorner() + pos;                               // are here
                vector goalposglobal = last_safe_pos + last_safe_region_corner;             // want to get here
                vector movevec = goalposglobal - posglobal;                                 // move vector
                vector movedir = llVecNorm(movevec);                                        // direction of move
                    ////llOwnerSay("Goal: " + (string)last_safe_pos + "  Distance: " + (string)llVecMag(movevec) +  " in " + (string)movedir); // ***TEMP***
                llSetRot(last_safe_rot);                                            // set rotation before going physical
                llSleep(0.5);
                llSetStatus(STATUS_PHYSICS, TRUE);                                  // back to real physics
                llSetAngularVelocity(<0,0,0>, FALSE);                           // and stop rotation
                llSetVelocity(<0,0,0>, FALSE);                                  // and stop
                llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <0,0,0>);   // stop engine
                llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, <0,0,0>);
                integer tries = 30;                                             // infinite loop prevention, should not matter
                while (region_corner == llGetRegionCorner() && tries > 0)       // move until region changes or fail
                {   llSetVelocity(movedir * MOVESPEED, FALSE);                  // try to move clear of ban line
                    llSleep(0.5);                                               // move physically
                    tries--;                                                    // prevent runaway
                }
                if (region_corner != llGetRegionCorner())                       // crossed region, end automatic motion.
                {   llWhisper(0,"Backed out across region boundary."); }        // if further motion is needed that happens on next cycle
                llSetVelocity(<0,0,0>, FALSE);                  // and stop
            } // end must cross region boundary
            ////llOwnerSay("Recovery complete"); // ***TEMP***
            llSleep(0.5);
            if (parcel_entry_allowed())
            {   llWhisper(0,"Successful back-out, clear of '" + (string)ownerstring + "' restricted area.");
            } else {
                llWhisper(0,"Unable to get clear of '" + (string)ownerstring + "' restricted area. Sorry");
            }
        } else {
            llWhisper(0,"Trouble immediately after startup. No safe position to move to is stored. Sorry.");
        }
        speed = 0;                          // force stop
        angular_motor=<0,0,0>;              // stop commanded rotation
        llSetVelocity(<0,0,0>, FALSE);      // really force stop
        llSetAngularVelocity(<0,0,0>, FALSE); // and stop rotation
        llSleep(0.5);                       // because llWhisper won't work for a while after leaving a no-script area
        llWhisper(0,"You have control."); 
    } else {                                    // good position, save as safe for potential recovery
        save_safe_position(llGetPos(), llGetRot(),llGetRegionCorner()); 
    }
}

default
{
    state_entry()
    {   
        llMessageLinked(LINK_ALL_CHILDREN, 0, "stop", NULL_KEY);
        llSetSitText("RideMe!");
        llStopSound();
        llSetTimerEvent(0.0);
        speed=0;
        llSitTarget(<0.1, 0.47, 0.7>, ZERO_ROTATION); //Sit: Front/Back, Sides, Up/Down
        llSetCameraEyeOffset(<-10.0, 0.0, 6.0>); //Camera position with no dynamic HUD
        llSetCameraAtOffset(<3.0, 0.0, 2.0>);
        llSetVehicleType(VEHICLE_TYPE_BOAT);
        llSetVehicleFlags(VEHICLE_FLAG_HOVER_UP_ONLY | VEHICLE_FLAG_HOVER_WATER_ONLY);
        // remove these flags 
        llRemoveVehicleFlags( VEHICLE_FLAG_HOVER_TERRAIN_ONLY 
                      | VEHICLE_FLAG_LIMIT_ROLL_ONLY 
                      | VEHICLE_FLAG_HOVER_GLOBAL_HEIGHT);
        
        // least for forward-back, most friction for up-down
        llSetVehicleVectorParam( VEHICLE_LINEAR_FRICTION_TIMESCALE, <2, 3, 2> );
    
        // uniform angular friction (setting it as a scalar rather than a vector)
        llSetVehicleFloatParam( VEHICLE_ANGULAR_FRICTION_TIMESCALE, 2 );

        // linear motor wins after about five seconds, decays after about a minute
        llSetVehicleFloatParam( VEHICLE_LINEAR_MOTOR_TIMESCALE, 5 );
        llSetVehicleFloatParam( VEHICLE_LINEAR_MOTOR_DECAY_TIMESCALE, 60 );

        // agular motor wins after four seconds, decays in same amount of time
        llSetVehicleFloatParam( VEHICLE_ANGULAR_MOTOR_TIMESCALE, 2 );
        llSetVehicleFloatParam( VEHICLE_ANGULAR_MOTOR_DECAY_TIMESCALE, 5 );

        // hover / float
        llSetVehicleFloatParam( VEHICLE_HOVER_HEIGHT, 0.3);
        llSetVehicleFloatParam( VEHICLE_HOVER_EFFICIENCY,.5 );
        llSetVehicleFloatParam( VEHICLE_HOVER_TIMESCALE, 2.0 );
        llSetVehicleFloatParam( VEHICLE_BUOYANCY, 0.5 );

        // halfway linear deflection with timescale of 3 seconds
        llSetVehicleFloatParam( VEHICLE_LINEAR_DEFLECTION_EFFICIENCY, 0.5 );
        llSetVehicleFloatParam( VEHICLE_LINEAR_DEFLECTION_TIMESCALE, 3 );

        // angular deflection 
        llSetVehicleFloatParam( VEHICLE_ANGULAR_DEFLECTION_EFFICIENCY, 0.5 );
        llSetVehicleFloatParam( VEHICLE_ANGULAR_DEFLECTION_TIMESCALE, 10 );
    
        // somewhat bounscy vertical attractor 
        llSetVehicleFloatParam( VEHICLE_VERTICAL_ATTRACTION_EFFICIENCY, 0.5 );
        llSetVehicleFloatParam( VEHICLE_VERTICAL_ATTRACTION_TIMESCALE, 2 );

        // weak negative damped banking
        llSetVehicleFloatParam( VEHICLE_BANKING_EFFICIENCY, 1 );
        llSetVehicleFloatParam( VEHICLE_BANKING_MIX, 0.1 );
        llSetVehicleFloatParam( VEHICLE_BANKING_TIMESCALE, .75 );

        // default rotation of local frame
        llSetVehicleRotationParam( VEHICLE_REFERENCE_FRAME, <0, 0, 0, 1> );
    }
    
    changed(integer change)
    {
        if (change & CHANGED_LINK)
        {
            key agent = llAvatarOnSitTarget();
            if (agent)
            {
                if (FALSE) //// agent != llGetOwner()) // allow non-owner
                {
                    llSay(0, "You aren't the owner");
                    llUnSit(agent);
                    llPushObject(agent, <0,0,100>, ZERO_VECTOR, FALSE);
                }
                else
                {
                    // You sit and are owner so get controls
                    llSetStatus(STATUS_PHYSICS, TRUE);
                    llSetStatus(STATUS_ROTATE_X | STATUS_ROTATE_Y | STATUS_ROTATE_Z, TRUE);
                    llRequestPermissions(agent,PERMISSION_TAKE_CONTROLS);
                }
            }
            else
            {
                // You stand so boat stops
                llMessageLinked(LINK_ALL_CHILDREN, 0, "stop", NULL_KEY);
                llSetStatus(STATUS_PHYSICS, FALSE);
                llSetStatus(STATUS_ROTATE_X | STATUS_ROTATE_Y | STATUS_ROTATE_Z, FALSE);
                llReleaseControls();
                llStopSound();
                llSetTimerEvent(0.0);
            }
        }
    }
    
    run_time_permissions(integer perm)
    {
        if (perm)
        {
            // Take these controls and lets go
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_RIGHT | CONTROL_LEFT | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT | CONTROL_UP | CONTROL_DOWN, TRUE, FALSE);
            llMessageLinked(LINK_ALL_CHILDREN, 0, "start", NULL_KEY);
            llLoopSound("ACMB",0.5);
            llSetTimerEvent(0.3);
        }
    }
    
    //  Control boat from arrow keys. Very basic.
    control(key id, integer level, integer edge)
    {

        if(level & CONTROL_FWD)
        {
            // Set cruising speed faster
            if(speed < MAX_SPEED)                   // not too fast
            {
                speed += SPEED_INCR;
            }
        }
        if(level & CONTROL_BACK)
        {
            // Set cruising speed slower
            if(speed > -MAX_SPEED_REVERSE)
            {
                speed -= SPEED_INCR;
            }
        }
        if(level & (CONTROL_RIGHT|CONTROL_ROT_RIGHT))
        {
            // Turn right
            angular_motor.x += TURN_RATE;
            angular_motor.z -= 1;
        }
        if(level & (CONTROL_LEFT|CONTROL_ROT_LEFT))
        {
            // Turn left
            angular_motor.x -= TURN_RATE;
            angular_motor.z += 1;
        }
        if(level & CONTROL_UP)
        {
            // Does nothing
        }
        if(level & CONTROL_DOWN)
        {        
            // Stops boat engine when down is pressed
            speed = 0;
        }   
    }

    timer()
    {   //  Parcel permissions trouble detection
        restricted_area_recovery();                             // deal with ban lines and no-script areas
        // the timer actually moves vehicle
        llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, <speed,0,0>);
        llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, angular_motor);
        // reset turning angle or you would go around in circles
        angular_motor=<0,0,0>;
    }    
}

