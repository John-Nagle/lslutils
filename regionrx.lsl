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

integer LOG_DEBUG = 0;                      // logging severity levels
integer LOG_NOTE = 1;
integer LOG_WARN = 2;
integer LOG_ERR = 3;
integer LOG_FAULT = 4;
integer LOG_FATAL = 5;
list LOG_SEVERITY_NAMES = ["DEBUG", "NOTE", "WARNING", "ERROR", "FAULT", "FATAL"];  // names for printing


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
integer     gLogMsgLevel = LOG_DEBUG;       // display messages locally above this level
integer     gTimerTick = 0;                 // number of timer ticks
integer     gRegionCrossCount = 0;          // number of regions crossed
string      gTripId = "???";                // random trip ID, for matching log messages
list        gSitters = [];                  // current sitters (keys)
list        gSitterDistances = [];          // distance to seat of sitter when seated (float)

//
//  logrx - logs to server or locally
//
logrx(integer severity, string msgtype, string msg, float val)
{
    if (severity >= gLogMsgLevel)           // in-world logging
    {   llOwnerSay(llList2String(LOG_SEVERITY_NAMES,severity) + " " + posasstring(llGetRegionName(), llGetPos()) + " " + msgtype + ": " + msg + " " + (string)val);   }
    //  Remote logging. Only works if there's another script listening for LOG messages
    list logdata = [];
    logdata = logdata + ["tripid"] + gTripId + ["severity"] + severity + ["type"] + msgtype + ["msg"] + msg + ["auxval"] + val;
    string s = llList2Json(JSON_OBJECT, logdata);   // encode as JSON
    llMessageLinked(LINK_THIS, 0, s, "LOG"); // put message on logger script queue.
}

initregionrx(integer loglevel)                              // initialization - call at vehicle start
{   gRegionCrossCount = 0;
    gTimerTick = 0;
    crossFault = FALSE;                     // no crossing fault
    crossStopped = FALSE;                   // not crossing
                                            // trip ID is a random ID to connect messages
    gTripId = llSHA1String((string)llFrand(1.0) + (string)llGetOwner() + (string)llGetPos());
    gLogMsgLevel = loglevel;                // set logging level
}

integer updatesitters()                     // update list of sitters - internal
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
            logrx(LOG_NOTE, "SITTER", "on prim #" + (string)linknum + " :" + avatarname + " distance to seat ", disttoseat);
        }
    }
    integer sittercount = llGetListLength(gSitters);
    logrx(LOG_NOTE, "RIDERCOUNT ","", (float)sittercount);
    return(sittercount);
}
//  
//  handlechanged --  call this on every "changed" event
//
integer handlechanged(integer change)           // returns TRUE if any riders
{   if (change & CHANGED_REGION)                // if in new region
    {   float speed = llVecMag(llGetVel());     // get velocity
        gRegionCrossCount++;                    // tally
        logrx(LOG_NOTE, "CROSSSPEED", "", speed);
        if (llGetStatus(STATUS_PHYSICS))        // if physics on
        {   crossVel = llGetVel();              // save velocity
            crossAngularVelocity = <0,0,0>;     // there is no llGetAngularVelocity();
            llSetStatus(STATUS_PHYSICS, FALSE); // forcibly stop object
            crossFault = FALSE;                 // no fault yet
            crossStopped = TRUE;                // stopped during region crossing
            crossStartTime = llGetTime();       // timestamp
        } else {                                // this is bad. A partial unsit usuallly follows
            logrx(LOG_ERR, "SECONDCROSS", "second region cross started before first one completed. Cross time: ", llGetTime()-crossStartTime);
        }
    }
    if((change & CHANGED_LINK) == CHANGED_LINK)     // rider got on or off
    {
        integer sittercount = updatesitters();
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
integer handletimer()                               // returns 0 if normal, 1 if cross-stopped, 2 if fault
{
    gTimerTick++;                                    // count timer ticks for debug
    //  Stop temporarily during region crossing until rider catches up.
    if (crossStopped)                               // if stopped at region crossing
    {   integer allseated = TRUE;
        integer i;
        integer sittercount = llGetListLength(gSitters); // number of sitters
        for (i = 0; i < sittercount; i++)
        {   if (ifnotseated(llList2Key(gSitters,i), llList2Float(gSitterDistances,i), gLogMsgLevel >= LOG_DEBUG))
            {   allseated = FALSE; }
        }
        if (allseated)                              // if all avatars are back in place
        {   llSetStatus(STATUS_PHYSICS, TRUE);      // physics back on
            llSetVelocity(crossVel, FALSE);         // use velocity from before
            llSetAngularVelocity(crossAngularVelocity, FALSE);  // and angular velocity
            crossStopped = FALSE;                   // no longer stopped
            float crosstime = llGetTime() - crossStartTime;
            logrx(LOG_NOTE, "CROSSEND", "Region crossing complete in ",crosstime);
            ////llOwnerSay("Velocity in: " + (string)llVecMag(crossVel) + "  out: " + (string)llVecMag(llGetVel()));
        } else {
            if ((llGetTime() - crossStartTime) > MAX_CROSSING_TIME)  // taking too long?
            {   if (!crossFault)                    // once only
                {   logrx(LOG_FAULT, "CROSSFAIL", "Crossing is taking too long. Probably stuck. Try teleporting out.", llGetTime()-crossStartTime);
                    crossFault = TRUE;
                    return(2);
                } 
            }
            return(1);                              // still cross-stopped
        }
    }
    return(0);                                      // not in trouble
}


