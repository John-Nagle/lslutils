//
//  regionrx -- region cross fix library
//
//  Animats
//  March, 2018
//
////#include "vehicleutils.lsl"                 // pure functions, no state
//    Basic settings
float TIMER_INTERVAL = 0.1;                 // check timer rate
float MAX_CROSSING_TIME = 10.0;             // stuck if crossing takes more than this long
//  Constants
integer TICK_NORMAL = 0;                    // normal tick event
integer TICK_CROSSSTOPPED = 1;              // stopped for region crossing
integer TICK_FAULT = 2;                     // fault on timer event

//
//  Globals
//
//  Status during region crossing
integer     crossStopped = FALSE;
vector      crossVel;
vector      crossAngularVelocity = <0,0,0>; // always zero for now
float       crossStartTime;                 // starts at changed event, ends when avatar in place
integer     crossFault = FALSE;             // no fault yet
//  Global status
integer     gTimerTick = 0;                 // number of timer ticks
integer     gRegionCrossCount = 0;          // number of regions crossed
list        gSitters = [];                  // current sitters (keys)
list        gSitterDistances = [];          // distance to seat of sitter when seated (float)

integer updatesitters(integer verbose)          // update list of sitters - internal
{                              
    gSitters = [];                              // rebuild list of sitters
    gSitterDistances = [];                      // and sitter distances
    integer linknum;
    for (linknum = 1; linknum < llGetNumberOfPrims(); linknum++)    // check all links for sitters
    {   key avatar = llAvatarOnLinkSitTarget(linknum);
        if (avatar != NULL_KEY)                 // found a seated avatar
        {   gSitters += avatar;                 // add to avatar list
            float disttoseat = avatardisttoseat(avatar); // add initial sit distance
            gSitterDistances += disttoseat; // add initial sit distance
            string avatarname = llList2String(llGetObjectDetails(avatar, [OBJECT_NAME]),0);
            if (verbose) { llOwnerSay("Now on prim #" + (string)linknum + " :" + avatarname + " distance to seat " + (string)disttoseat); }
        }
    }
    integer sittercount = llGetListLength(gSitters);
    if (verbose) { llOwnerSay((string)sittercount + " riders."); }
    return(sittercount);
}
//  
//  handlechanged --  call this on every "changed" event
//
integer handlechanged(integer change, integer verbose)  // returns TRUE if any riders
{   if (change & CHANGED_REGION)                // if in new region
    {   float speed = llVecMag(llGetVel());     // get velocity
        gRegionCrossCount++;                    // tally
        if (verbose) llOwnerSay("Speed at region cross #" 
            + (string)gRegionCrossCount + ": "  
            + (string)speed + " m/s");
        if (llGetStatus(STATUS_PHYSICS))        // if physics on
        {   crossVel = llGetVel();              // save velocity
            crossAngularVelocity = <0,0,0>;     // there is no llGetAngularVelocity();
            llSetStatus(STATUS_PHYSICS, FALSE); // forcibly stop object
            crossFault = FALSE;                 // no fault yet
            crossStopped = TRUE;                // stopped during region crossing
            crossStartTime = llGetTime();       // timestamp
        } else {                                // this is bad. A partial unsit usuallly follows
            if (verbose) llOwnerSay("TROUBLE - second region cross started before first one completed, at " 
                + posasstring(llGetRegionName(), llGetPos()));
        }
    }
    if((change & CHANGED_LINK) == CHANGED_LINK)     // rider got on or off
    {
        integer sittercount = updatesitters(verbose);
        if (sittercount == 0)
        {   llSetTimerEvent(0.0);   }               // no sitters, no timer
        else
        {   llSetTimerEvent(TIMER_INTERVAL); }      // sitters, run timer
    }
    return(llGetListLength(gSitters) > 0);          // returns TRUE if anybody on board
}

//
//  handletimer  --  call this on every timer tick
//
integer handletimer(integer verbose)                // returns 0 if normal, 1 if cross-stopped, 2 if fault
{
    gTimerTick++;                                    // count timer ticks for debug
    //  Stop temporarily during region crossing until rider catches up.
    if (crossStopped)                               // if stopped at region crossing
    {   integer allseated = TRUE;
        integer i;
        for (i = 0; i < llGetListLength(gSitters); i++)
        {   if (ifnotseated(llList2Key(gSitters,i), llList2Float(gSitterDistances,i), FALSE))
            {   allseated = FALSE; }
        }
        if (allseated)                              // if all avatars are back in place
        {   llSetStatus(STATUS_PHYSICS, TRUE);      // physics back on
            llSetVelocity(crossVel, FALSE);         // use velocity from before
            llSetAngularVelocity(crossAngularVelocity, FALSE);  // and angular velocity
            crossStopped = FALSE;                   // no longer stopped
            float crosstime = llGetTime() - crossStartTime;
            if (verbose) llOwnerSay("Avatar(s) back in place. Region crossing complete in " + (string)crosstime + "secs.");
            ////llOwnerSay("Velocity in: " + (string)llVecMag(crossVel) + "  out: " + (string)llVecMag(llGetVel()));
        } else {
            if ((llGetTime() - crossStartTime) > MAX_CROSSING_TIME)  // taking too long?
            {   if (!crossFault)                    // once only
                {   llOwnerSay("TROUBLE - crossing is taking too long. Probably stuck. Try teleporting out.");
                    return(2);
                } 
            }
            return(1);                              // still cross-stopped
        }
    }
    return(0);                                      // not in trouble
}


