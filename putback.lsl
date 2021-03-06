//
//  Put back
//
//  Puts a physical object back where it belongs after
//  no avatar is within NEARBY_AVATAR distance in the same region.
//
//  The object's normal state is physics off. A collision
//  turns physics on. The object can then roll or be pushed.
//
//  Region crossing is handled properly.
//
//  Animats
//  November, 2017

//  Configuration constants - changeable if desired
float NEARBY_AVATAR = 20.0;         // don't move if avatar this close
float MAX_DISTANCE_FROM_HOME = 50.0;// anti-theft - max distance before return
//  Constants
float REGION_CROSS_VEL = 4.0;       // speed for crossing region boundary


//  State variables
vector base_pos;                    // where to move back to
vector base_region_corner;          // corner of initial region
rotation base_rot;                  // initial rotation

init()                              // called at startup
{   llSetTimerEvent(0.0);           // clear any timer
    llSetStatus(STATUS_PHYSICS, FALSE); //physics off
    llSetStatus(STATUS_BLOCK_GRAB_OBJECT, FALSE); // but grabbable 
    llSetStatus(STATUS_BLOCK_GRAB, FALSE); // but grabbable

}

move_back()                         // go back to base pos
{   integer trouble = FALSE;        // no trouble yet
    if (llGetRegionCorner() != base_region_corner) // if in wrong region
    {   //  Need to move object across a region boundary. This is hard.
        //  We do it using the physics engine with gravity and collisions off.
        llOwnerSay("Outside home region at " + 
            llGetRegionName() + " " + 
            (string)llGetPos());
        //  Set object so it responds to physics, can pass through anything,
        //  and cannot fall. Now it can be moved.
        llSetStatus(STATUS_PHANTOM, TRUE);  // can move through everything
        llSetStatus(STATUS_PHYSICS, TRUE);  // but responds to forces
        llSetVelocity(<0.0,0.0,0.0>, FALSE);// bring it to a stop
        llSleep(0.5);                       // allow stopping time
        float gravitymult = llList2Float(llGetPhysicsMaterial(),0); // save gravity mult
        float disttohome = 999999999.0;         // for stuck detector only
        while (llGetRegionCorner() != base_region_corner) // while in wrong region
        {   vector tohome = (base_region_corner + base_pos)
                    - (llGetRegionCorner() +  llGetPos());    // dir to get home
            // Reset status every time. Sometimes Physics gets turned off somehow.
            llSetPhysicsMaterial(GRAVITY_MULTIPLIER, 0.0, 0.0, 0.0, 0.0);
            llSetStatus(STATUS_PHANTOM, TRUE);  // can move through everything
            llSetStatus(STATUS_PHYSICS, TRUE);  // but responds to forces                
            llSetVelocity(llVecNorm(tohome) * REGION_CROSS_VEL, FALSE); // push
            float newdisttohome = llVecMag(tohome); 
            string s = llGetObjectName() + " trying to return, "
                + (string) llVecMag(tohome) + "m from home and at " 
                + llGetRegionName() + ", " + (string)llGetPos();
                
           if (newdisttohome >= disttohome)// not getting closer - trouble
            {   s = s + " and stuck! Please retrieve object.";
                llInstantMessage(llGetOwner(), s); // tell owner
                trouble = TRUE;             // stuck, must report
                llSleep(30.0);              // avoid spam
            }
            llOwnerSay(s);                  // debug info
            disttohome = newdisttohome;     // update dist to home
            llSleep(0.5);
        }            
        llSetVelocity( <0.0,0.0,0.0>, FALSE ); // turn off push
        llOwnerSay("Back in home region.");
        //  Restore gravity multiplier
        llSetPhysicsMaterial(GRAVITY_MULTIPLIER, gravitymult, 0.0, 0.0, 0.0);
    }
    llSetStatus(STATUS_PHYSICS, FALSE);
    llSetStatus(STATUS_PHANTOM, TRUE);
    do {
        llSetPos(base_pos);         // max move 10 meters
        llSetRot(base_rot);
        llSleep(0.5);               // prevent runaway
    } while (llVecDist(base_pos, llGetPos()) > 1.0); // get close
    llSetPos(base_pos);             // final move to goal
    llSetRot(base_rot);
    llSetStatus(STATUS_PHANTOM, FALSE);
    if (trouble) { llInstantMessage(llGetOwner(), "Safely back home."); }
}

default
{
    state_entry()
    {   init();
        state waiting;
    }
    on_rez(integer sparam)
    {   init();
        state waiting;
    }

}

state waiting               // idle waiting for something to happen
{   
    on_rez(integer sparam)  // in case someone put it away active
    {   state default; }
    
    collision_start(integer tnum)
    {   if (llDetectedType(0) & (ACTIVE | AGENT))
        {   base_pos = llGetPos();
            base_region_corner = llGetRegionCorner(); // region in world coords
            base_rot = llGetRot();
            state hit;          // we've been hit
        }
    }
    
    collision_end(integer tnum)
    {
    }
}
 
state hit                       // hit at least once
{
    on_rez(integer sparam)
    {   state default; }

    
    state_entry()
    {   base_pos = llGetPos();  // save base pos before physics on
        base_region_corner = llGetRegionCorner(); // region in world coords
        base_rot = llGetRot();
        llSetTimerEvent(5.0);   // start idle timer
        llSetStatus(STATUS_PHYSICS, TRUE); // now physical
    }
    
    
    //  As long as someone is nearby, it won't reset
    timer()
    {   vector tohome = (base_region_corner + base_pos)
                    - (llGetRegionCorner() +  llGetPos());    // dir to get home
        if (llVecMag(tohome) > MAX_DISTANCE_FROM_HOME) // if too far from home
        {   llSay(0,"Too far from home! Going home now.");
            llSetTimerEvent(0.0);   // stop timer
            state fix;   
        }
        llSensor("", NULL_KEY, AGENT, NEARBY_AVATAR, PI); // nearby avatar check
    } 
    
    no_sensor()                 // no nearby avatar triggers moveback
    {   llSetTimerEvent(0.0);   // stop timer
        state fix;   
    }
      
}

state fix                       // restore to original location
{
    on_rez(integer sparam)
    {   state default; }

    state_entry()
    {   move_back();
        llSleep(2.0);           // allow settling time
        state waiting;
    }
}// END ////
