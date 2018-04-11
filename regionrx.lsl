//
//  regionrx -- region cross fix library
//
//  Animats
//  March, 2018
//
//    Basic settings
float TIMER_INTERVAL = 0.1;                 // check timer rate
float MAX_CROSSING_TIME = 30.0;             // stuck if crossing takes more than this long
integer MAX_SECS_BETWEEN_MSGS = 60;         // message at least once this often
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
integer     crossHover = FALSE;             // not hovering across a region crossing
float       crossHoverHeight;               // height during region crossing
//  Global status
integer     gLogMsgLevel = LOG_DEBUG;       // display messages locally above this level
integer     gLogSerial = 0;                 // log serial number

integer     gTimerTick = 0;                 // number of timer ticks
float       gDistanceTraveled = 0.0;        // distance traveled
vector      gPrevPos = <0,0,0>;             // previous position
integer     gRegionCrossCount = 0;          // number of regions crossed
string      gTripId = "???";                // random trip ID, for matching log messages
integer     gLastMsgTime = 0;               // time last message was sent
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
    gLastMsgTime = llGetUnixTime();         // time we last sent a message
    logdata = logdata + ["tripid"] + gTripId + ["severity"] + severity + ["eventtype"] + msgtype + ["msg"] + msg + ["auxval"] + val
        + ["timestamp"] + gLastMsgTime + ["serial"] + gLogSerial;
    string s = llList2Json(JSON_OBJECT, logdata);   // encode as JSON
    llMessageLinked(LINK_THIS, 0, s, "LOG"); // put message on logger script queue.
    gLogSerial++;                           // serial number within 
}

initregionrx(integer loglevel)              // initialization - call at vehicle start
{   gRegionCrossCount = 0;
    gTimerTick = 0;
    gLogSerial = 0;                         // reset log serial number
    vector pos = llGetPos();                // get starting position
    gPrevPos = pos + llGetRegionCorner();   // global pos
    gPrevPos.z = 0.0;                       // only care about XY
    gDistanceTraveled = 0.0;
    crossFault = FALSE;                     // no crossing fault
    crossStopped = FALSE;                   // not crossing
    crossHover = TRUE;                      // assuming hovering so we will turn hover off
                                            // trip ID is a random ID to connect messages
    gTripId = llSHA1String((string)llFrand(1.0) + (string)llGetOwner() + (string)llGetPos());
    gLogMsgLevel = loglevel;                // set logging level
    key driver = llAvatarOnSitTarget();     // key of driver
    string driverdisplayname = llGetDisplayName(driver);    // log driver name
    string drivername = llKey2Name(driver); // "login name / display name"
    logrx(LOG_NOTE,"STARTUP", drivername + "/" + driverdisplayname,0.0);      // log startup
    logrx(LOG_NOTE,"DRIVERKEY", (string)driver, 0.0); // Driver's key, for when names become changeable
}

integer updatesitters()                     // update list of sitters - internal
{                              
    gSitters = [];                              // rebuild list of sitters
    gSitterDistances = [];                      // and sitter distances
    integer linknum;
    integer primcount = llGetNumberOfPrims();   // do once before loop
    for (linknum = 1; linknum <= primcount; linknum++)    // check all links for sitters
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
    vector pos = llGetPos();
    vector gpos = pos + llGetRegionCorner();        // global pos
    gpos.z = 0.0;                                   // only care about XY
    gDistanceTraveled += llVecMag(gPrevPos - gpos); // distance traveled add
    gPrevPos = gpos;                                // save position
    
    //  Hover control -- hover when outside region and unsupported.
    //  Should help on flat terrain. Too dumb for region crossings on hills.
    if (outsideregion(pos))                         // if outside region
    {   if (!crossHover)                            // if not hovering
        {   llSetVehicleFloatParam(VEHICLE_HOVER_HEIGHT, crossHoverHeight);     // anti-sink
            llSetVehicleFlags(VEHICLE_FLAG_HOVER_GLOBAL_HEIGHT);  
            crossHover = TRUE;                      // 
        }
    } else {
        crossHoverHeight = pos.z;
        if (crossHover)
        {   llRemoveVehicleFlags(VEHICLE_FLAG_HOVER_GLOBAL_HEIGHT);
            crossHover = FALSE;
        }
    }
 
    //  Stop temporarily during region crossing until rider catches up.
    if (crossStopped)                               // if stopped at region crossing
    {   integer allseated = TRUE;
        integer i;
        integer sittercount = llGetListLength(gSitters); // number of sitters
        for (i = 0; i < sittercount; i++)
        {   if (ifnotseated(llList2Key(gSitters,i), llList2Float(gSitterDistances,i), gLogMsgLevel <= LOG_DEBUG))
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
            if (llGetUnixTime() - gLastMsgTime > 2.0)
            {   logrx(LOG_WARN, "CROSSSLOW","Waiting for avatar(s) to cross regions.",0.0);            // send at least one message every 60 seconds
            }
            return(1);                              // still cross-stopped
        }
    }
    if (llGetUnixTime() - gLastMsgTime > MAX_SECS_BETWEEN_MSGS)
    {   logrx(LOG_NOTE, "TICK","", gDistanceTraveled/1000.0); }          // send at least one message every 60 seconds
    return(0);                              // not in trouble
}


