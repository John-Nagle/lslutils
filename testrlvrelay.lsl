integer ALLOW_REMOVE_DETACH = TRUE;
integer DEBUG = TRUE;
 
 
//~ RestrainedLife Viewer Relay Script example code
//~ By Marine Kelley, Maike Short, Felis Darwin
 
//~ This code is provided AS-IS, OPEN-SOURCE and holds NO WARRANTY of accuracy,
//~ completeness or performance. It may only be distributed in its full source code,
//~ this header and disclaimer and is not to be sold.
 
 
 
// ---------------------------------------------------
//                     Constants
// ---------------------------------------------------
 
integer RLVRS_PROTOCOL_VERSION = 1014; // version of the protocol, stated on the specification page
 
string PREFIX_RL_COMMAND = "@";
string PREFIX_METACOMMAND = "!";
 
integer RLVRS_CHANNEL = -1812221819;  // RLVRS in numbers
integer DIALOG_CHANNEL = -1812220409; // RLVDI in numbers
 
integer MAX_OBJECT_DISTANCE = 1000; // was 100;     // 100m is llShout distance
integer MAX_TIME_AUTOACCEPT_AFTER_FORCESIT = 60; // seconds
 
integer PERMISSION_DIALOG_TIMEOUT = 30;
 
integer LOGIN_DELAY_WAIT_FOR_PONG = 10;
integer LOGIN_DELAY_WAIT_FOR_FORCE_SIT = 60;
 
integer PING_INTERVAL = 60; //== Time between pings
 
integer MODE_OFF = 0;
integer MODE_ASK = 1;
integer MODE_AUTO = 2;
 
// ---------------------------------------------------
//                      Variables
// ---------------------------------------------------
 
integer nMode;
 
list lRestrictions; // restrictions currently applied (without the "=n" part)
key kSource;        // UUID of the object I'm commanded by, always equal to NULL_KEY if lRestrictions is empty, always set if not
 
string sPendingName; // name of initiator of pending request (first request of a session in mode 1)
key sPendingId;      // UUID of initiator of pending request (first request of a session in mode 1)
string sPendingMessage; // message of pending request
integer sPendingTime;
 
// used on login
integer timerTickCounter; // count the number of time events on login (forceSit has to be delayed a bit)
integer loginWaitingForPong;
integer loginPendingForceSit;
 
key     lastForceSitDestination;
integer lastForceSitTime;
 
// ---------------------------------------------------
//               Low Level Communication
// ---------------------------------------------------
 
//NS
// always use this function to set the mode, so we can, for example, change the color of the HUD to show the state
set_mode(integer mode)
{
     nMode = mode;
     // example of color setting that I use on HUD based relay
    if (nMode == 0) {
        llSetColor(<0.0,1.0,0.0>,ALL_SIDES);
    } else if (nMode == 1) {
        llSetColor(<1.0,1.0,0.0>,ALL_SIDES);
    } else if (nMode == 2) {
        llSetColor(<1.0,0.0,0.0>,ALL_SIDES);
    }
}

//NS
// locking code, called to prevent detach of HUD or worn relays
lock_relay()
{
    sendRLCmd("@detach=n");   
}

unlock_relay()
{
    sendRLCmd("@detach=y");
}

 
debug(string x)
{
    if (DEBUG)
    {
        llOwnerSay("DEBUG: " + x);
    }
}
 
// acknowledge or reject
ack(string cmd_id, key id, string cmd, string ack)
{
    llSay(RLVRS_CHANNEL, cmd_id + "," + (string)id + "," + cmd + "," + ack);
}
 
// cmd begins with a '@' 
sendRLCmd(string cmd)
{   debug(" issuing RLV command " + cmd);
    llOwnerSay(cmd);
}
 
// get current mode as string
string getModeDescription()
{
    if (nMode == 0) return "RLV Relay is OFF"; 
    else if (nMode == 1) return "RLV Relay is ON (permission needed)"; 
    else return "RLV Relay is ON (auto-accept)"; 
}
 
// check that this command is for us and not someone else
integer verifyWeAreTarget(string message)
{
    list tokens = llParseString2List(message, [","], []);
    if (llGetListLength(tokens) == 3) // this is a normal command
    {
      if (llList2String(tokens, 1) == llGetOwner()) // talking to me ?
      {
         return TRUE;
      }
    }
    return FALSE;
}
 
// ---------------------------------------------------
//               Permission Handling
// ---------------------------------------------------
 
// are we already under command by this object?
integer isObjectKnow(key id)
{
    // first some error handling
    if (id == NULL_KEY)
    {
        return FALSE;
    }
 
    // are we already under command by this object?
    if (kSource == id)
    {
        return TRUE;
    }
 
    // are we not under command by any object but were we forced to sit on this object recently?
    if ((kSource == NULL_KEY) && (id == lastForceSitDestination))
    {
        debug("on last force sit target");
        if (lastForceSitTime + MAX_TIME_AUTOACCEPT_AFTER_FORCESIT > llGetUnixTime())
        {
            debug("and recent enough to auto accept");
            return TRUE;
        }
    }
 
    return FALSE;
}
 
 
// check whether the object is in llSay distance.
// The specification requires llSay instead of llShout or llRegionSay
// to be used to limit the range. But this has to be checked here again
// because the objects are not trustworthy.
integer isObjectNear(key id)
{
    vector myPosition = llGetRootPosition();
    list temp = llGetObjectDetails(id, ([OBJECT_POS]));
    vector objPostition = llList2Vector(temp,0);
    float distance = llVecDist(objPostition, myPosition);
    return distance <= MAX_OBJECT_DISTANCE;
}
 
// do a basic check on the identity of the object trying to issue a command
integer isObjectIdentityTrustworthy(key id)
{
    key parcel_owner=llList2Key (llGetParcelDetails (llGetPos (), [PARCEL_DETAILS_OWNER]), 0);
    key parcel_group=llList2Key (llGetParcelDetails (llGetPos (), [PARCEL_DETAILS_GROUP]), 0);
    key object_owner=llGetOwnerKey(id);
    key object_group=llList2Key (llGetObjectDetails (id, [OBJECT_GROUP]), 0);
 
    debug("owner= " + (string) parcel_owner + " / " + (string) object_owner);
    debug("group= " + (string) parcel_group + " / " + (string) object_group);
 
    if (object_owner==llGetOwner ()        // IF I am the owner of the object
      || object_owner==parcel_owner        // OR its owner is the same as the parcel I'm on
      || object_group==parcel_group        // OR its group is the same as the parcel I'm on
    )
    {
        return TRUE;
    }
    llOwnerSay("Object identity is untrustworthy, but allowing anyway."); // ***TEMP***
    return TRUE;    // ***TEMP***
    return FALSE;
}
 
 
// Is this a simple request for information or a meta command like !release?
integer isSimpleRequest(list list_of_commands) 
{
    integer len = llGetListLength(list_of_commands);
    integer i;
 
    // now check every single atomic command
    for (i=0; i < len; ++i)
    {
        string command = llList2String(list_of_commands, i);
        if (!isSimpleAtomicCommand(command))
        {
           return FALSE;
        }
    }
 
    // all atomic commands passed the test
    return TRUE;
}
 
// is this a simple atmar command
// (a command which only queries some information or releases restrictions)
// (e. g.: cmd ends with "=" and a number (@version, @getoutfit, @getattach) or is a !-meta-command)
integer isSimpleAtomicCommand(string cmd)
{
    // check right hand side of the "=" - sign
    integer index = llSubStringIndex (cmd, "=");
    if (index > -1) // there is a "=" 
    {
        // check for a number after the "="
        string param = llGetSubString (cmd, index + 1, -1);
        if ((integer)param!=0 || param=="0") // is it an integer (channel number)?
        {
            return TRUE;
        }
 
        // removing restriction
        if ((param == "y") || (param == "rem"))
        {
            return TRUE;
        }
    }
 
    // check for a leading ! (meta command)
    if (llSubStringIndex(cmd, PREFIX_METACOMMAND) == 0)
    {
        return TRUE;
    }
 
    // check for @clear
    // Note: @clear MUST NOT be used because the restrictions will be reapplied on next login
    // (but we need this check here because "!release|@clear" is a BROKEN attempt to work around
    // a bug in the first relay implementation. You should refuse to use relay versions < 1013
    // instead.)
    if (cmd == "@clear")
    {
        return TRUE;
    }
 
    // this one is not "simple".
    return FALSE;
}
 
// If we already have commands from this object pending
// because of a permission request dialog, just add the
// new commands at the end.
// Note: We use a timeout here because the player may
// have "ignored" the dialog.
integer tryToGluePendingCommands(key id, string commands)
{
    if ((sPendingId == id) && (sPendingTime + PERMISSION_DIALOG_TIMEOUT > llGetUnixTime()))
    {
        debug("Gluing " + (string) sPendingMessage + " with " + commands);
        //NS
        sPendingMessage = sPendingMessage + "|" + commands;
        //sPendingMessage = sPendingMessage + commands;
        return TRUE;
    }
    return FALSE;
}
 
 
// accept !release even if out of range
handleCommandsWhichAreAcceptedOutOfRange(key id, string message)
{
    if (id != kSource)
    {
        return;
    }
 
    list tokens = llParseString2List (message, [","], []);
    if (llGetListLength (tokens) < 3)
    {
        return;
    }
    string commands = llList2String(tokens, 2);
    list list_of_commands = llParseString2List(commands, ["|"], []);
    if (llListFindList(list_of_commands, ["!release"]) > -1)
    {
        debug("accepted !release although it was out of range");
        releaseRestrictions();
    }
}
 
 
// verifies the permission. This includes mode 
// (off, permission, auto) of the relay and the
// identity of the object (owned by parcel people).
integer verifyPermission(key id, string name, string message)
{
    // is it switched off?
    if (nMode == MODE_OFF)
    {
        return FALSE;
    }
 
    list tokens = llParseString2List (message, [","], []);
    if (llGetListLength (tokens) < 3)
    {
        return FALSE;
    }
    string commands = llList2String(tokens, 2);
    list list_of_commands = llParseString2List(commands, ["|"], []);
 
    // if we are already having a pending permission-dialog request for THIS object,
    // just add the new commands at the end of the pending command list.
    if (tryToGluePendingCommands(id, commands))
    {
        return FALSE;
    }
 
    // accept harmless commands silently
    if (isSimpleRequest(list_of_commands))
    {
        return TRUE;
    }
 
    // check whether this object belongs here
    integer trustworthy = isObjectIdentityTrustworthy(id);
    string warning = "";
    ////if (!trustworthy)
    ////{
    ////    warning = "\n\nWARNING: This object is not owned by the people owning this parcel. Unless you know the owner, you should deny this request.";
    ////}
 
    // ask in permission-request-mode and/OR in case the object identity is suspisous.
    if (nMode == MODE_ASK || !trustworthy)
    {
        sPendingId=id;
        sPendingName=name;
        sPendingMessage = message;
        sPendingTime = llGetUnixTime();
        if (llKey2Name(llGetOwnerKey(id)) != "")
        {
            name += " (owned by " + llKey2Name(llGetOwnerKey(id)) + ")";
        }
        llDialog (llGetOwner(), name + " would like control your viewer." + warning + ".\n\nDo you accept ?", ["Yes", "No"], DIALOG_CHANNEL);
        debug("Asking for permission");
        return FALSE;
    }
    return TRUE;
}
 
 
// ---------------------------------------------------
//               Executing of commands
// ---------------------------------------------------
 
// execute a non-parsed message
// this command could be denied here for policy reasons, (if it were implemenetd)
// but this time there will be an acknowledgement
execute(string name, key id, string message)
{
    list tokens=llParseString2List (message, [","], []);
    if (llGetListLength (tokens)==3) // this is a normal command
    {
        string cmd_id=llList2String (tokens, 0); // CheckAttach
        key target=llList2Key (tokens, 1); // UUID
        if (target==llGetOwner ()) // talking to me ?
        {
            list list_of_commands=llParseString2List (llList2String (tokens, 2), ["|"], []);
            integer len=llGetListLength (list_of_commands);
            integer i;
            string command;
            string prefix;
            for (i=0; i<len; ++i) // execute every command one by one
            {
                // a command is a RL command if it starts with '@' or a metacommand if it starts with '!'
                command=llList2String (list_of_commands, i);
                prefix=llGetSubString (command, 0, 0);
 
                if (prefix==PREFIX_RL_COMMAND) // this is a RL command
                {
                    executeRLVCommand(cmd_id, id, command);
                }
                else if (prefix==PREFIX_METACOMMAND) // this is a metacommand, aimed at the relay itself
                {
                    executeMetaCommand(cmd_id, id, command);
                }
            }
        }
    }
}
 
// executes a command for the restrained life viewer 
// with some additinal magic like book keeping
executeRLVCommand(string cmd_id, string id, string command)
{
    // we need to know whether whether is a rule or a simple command
    list tokens_command=llParseString2List (command, ["="], []);
    string behav=llList2String (tokens_command, 0); // @getattach:skull
    string param=llList2String (tokens_command, 1); // 2222
    integer ind=llListFindList (lRestrictions, [behav]);
 
debug("param=" + param);
 
    if (param=="n" || param=="add") // add to lRestrictions
    {
        //NS lock relay when we add restrictions
        if (ind<0) {
            lRestrictions+=[behav];
            lock_relay();
        }
        kSource=id; // we know that kSource is either NULL_KEY or id already
    }
    else if (param=="y" || param=="rem") // remove from lRestrictions
    {
        if (ind>-1) lRestrictions=llDeleteSubList (lRestrictions, ind, ind);
        //NS unlock relay when there are no restrictions
        if (llGetListLength (lRestrictions)==0) {
            kSource=NULL_KEY;
            unlock_relay();
        }
        removeFromPendingList(behav);
    }
    else if (param == "force" && !ALLOW_REMOVE_DETACH)
    {
        debug("force: " + behav);
        list temp = llParseString2List(behav, [":"], []);
        string commandName = llList2String (temp, 0); // @sit
        if (commandName == "@detach" || commandName == "@remoutfit")
        {
            debug("rejecting remove/detach");
            llWhisper(0, "Not stripping");
            ack(cmd_id, id, command, "ko");
            return;
        }
    }
 
    workaroundForAtClear(command);
    rememberForceSit(command);
    sendRLCmd(command); // execute command
    ack(cmd_id, id, command, "ok"); // acknowledge
}
 
// check for @clear
// Note: @clear MUST NOT be used because the restrictions will be reapplied on next login
// (but we need this check here because "!release|@clear" is a BROKEN attempt to work around
// a bug in the first relay implementation. You should refuse to use relay versions < 1013
// instead.)
workaroundForAtClear(string command)
{
    if (command == "@clear")
    {
        releaseRestrictions();
    }
}
 
// remembers the time and object if this command is a force sit
rememberForceSit(string command)
{
    list tokens_command=llParseString2List (command, ["="], []);
    string behav=llList2String (tokens_command, 0); // @sit:<uuid>
    string param=llList2String (tokens_command, 1); // force
 
    // clear lastForceSitDestination in case we are now prevented from standing up and
    // the force sit was long ago. Note: lRestrictions is checked to prevent the
    // clearance in case @unsit is just send again on login
    if (behav == "@unsit")
    {
        if (llListFindList(lRestrictions, ["@unsit"]) < 0)
        {
            if (lastForceSitTime + MAX_TIME_AUTOACCEPT_AFTER_FORCESIT < llGetUnixTime())
            {
                debug("clearing lastForceSitDestination");
                lastForceSitDestination = NULL_KEY;
            }
        }
    }
 
    if (param != "force")
    {
        return;
    }
 
    tokens_command=llParseString2List(behav, [":"], []);
    behav=llList2String (tokens_command, 0); // @sit
    param=llList2String (tokens_command, 1); // <uuid>
    debug("'force'-command:" + behav + "/" + param);
    if (behav != "@sit")
    {
        return;
    }
    lastForceSitDestination = (key) param;
    lastForceSitTime = llGetUnixTime();
    debug("remembered force sit");
}
 
// executes a meta command which is handled by the relay itself
executeMetaCommand(string cmd_id, string id, string command)
{
    if (command==PREFIX_METACOMMAND+"version") // checking relay version
    {
        ack(cmd_id, id, command, (string)RLVRS_PROTOCOL_VERSION);
    }
    else if (command==PREFIX_METACOMMAND+"release") // release all the restrictions (end session)
    {
        releaseRestrictions();
        ack(cmd_id, id, command, "ok");
    }
}
 
removeFromPendingList(string behav)
{
    // TODO: implement me    
}
 
 
// lift all the restrictions (called by !release and by turning the relay off)
releaseRestrictions ()
{
    kSource=NULL_KEY;
    //NS
    unlock_relay();
    integer i;
    integer len=llGetListLength (lRestrictions);
    for (i=0; i<len; ++i)
    {
        sendRLCmd(llList2String (lRestrictions, i)+"=y");
    }
    lastForceSitDestination = NULL_KEY;
    lRestrictions = [];
    loginPendingForceSit = FALSE;
}
 
 
// ---------------------------------------------------
//            initialisation and login handling
// ---------------------------------------------------
 
init() {
    //NS
    //nMode = 1;  old
    set_mode(MODE_ASK);
    unlock_relay();
    
    kSource = NULL_KEY;
    lRestrictions = [];
    sPendingId = NULL_KEY;
    sPendingName = "";
    sPendingMessage = "";
    llListen (RLVRS_CHANNEL, "", "", "");
    llListen (DIALOG_CHANNEL, "", llGetOwner(), "");
    llOwnerSay (getModeDescription());
    debug((string) llGetFreeMemory());
}
 
// sends the known restrictions (again) to the RL-viewer
// (call this functions on login)
reinforceKnownRestrictions()
{
    integer i;
    integer len=llGetListLength(lRestrictions);

    //NS
    if (len >0) {
        lock_relay();
    } else {
        unlock_relay();
    }
    
    string restr;
    debug("kSource=" + (string) kSource);
    for (i=0; i<len; ++i)
    {
        restr=llList2String(lRestrictions, i);
        debug("restr=" + restr);
        sendRLCmd(restr+"=n");
        if (restr=="@unsit")
        {
            loginPendingForceSit = TRUE;
        }
    }
}
 
// send a ping request and start a timer
pingWorldObjectIfUnderRestrictions()
{
    loginWaitingForPong = FALSE;
    if (kSource != NULL_KEY)
    {
        ack("ping", kSource, "ping", "ping");
        timerTickCounter = 0;
        llSetTimerEvent(1.0);
        loginWaitingForPong = TRUE;
    }
}
 
sendForceSitDuringLogin()
{
    key target = lastForceSitDestination;
    if (target == NULL_KEY)
    {
        target = kSource;
    }
    debug("Force sit during login on " + (string) target);
    sendRLCmd ("@sit:"+(string)target+"=force");
}
 
default
{
    state_entry()
    {
        init();
    }
 
    on_rez(integer start_param)
    {
        // relogging, we must refresh the viewer and ping the object if any
        // if mode is not OFF, fire all the stored restrictions
        if (nMode)
        {
            reinforceKnownRestrictions();
            pingWorldObjectIfUnderRestrictions();
        } else {
            //NS
            unlock_relay();
        }
        // remind the current mode to the user
        llOwnerSay(getModeDescription());
    }
 
 
    timer()
    {
        timerTickCounter++;
        debug("timer (" + (string) timerTickCounter + "): waiting for pong: " + (string) loginWaitingForPong + " pendingForceSit: " + (string) loginPendingForceSit);
        if (loginWaitingForPong && (timerTickCounter >= LOGIN_DELAY_WAIT_FOR_PONG))
        {
            llWhisper(0, "Lucky Day: " + llKey2Name(llGetOwner()) + " is freed because the device is not available.");
            loginWaitingForPong = FALSE;
            loginPendingForceSit = FALSE;
            releaseRestrictions();
        }
 
        if (loginPendingForceSit)
        {
            integer agentInfo = llGetAgentInfo(llGetOwner());
            if (agentInfo & AGENT_SITTING)
            {
                loginPendingForceSit = FALSE;
                debug("is sitting now");
            }
            else if (timerTickCounter == LOGIN_DELAY_WAIT_FOR_FORCE_SIT)
            {
                llWhisper(0, "Lucky Day: " + llKey2Name(llGetOwner()) + " is freed because sitting down again was not possible.");
                loginPendingForceSit = FALSE;
                releaseRestrictions();
            }
            else
            {
                sendForceSitDuringLogin();
            }
        }
 
        if (!loginPendingForceSit && !loginWaitingForPong)
        {
            llSetTimerEvent(0.0);
        }
    }
 
    listen(integer channel, string name, key id, string message)
    {
        if (channel==RLVRS_CHANNEL)
        {
            debug("LISTEN: " + message);
            if (!verifyWeAreTarget(message))
            {
               return;
            }
 
            if (nMode== MODE_OFF)
            {
                debug("deactivated - ignoring commands");
                return; // mode is 0 (off) => reject
            }
            debug("Got message (active world object " + (string) kSource + "): name=" + name+ "id=" + (string) id + " message=" + message);
            if (!isObjectNear(id)) 
            {   debug("Object is not near.");
                handleCommandsWhichAreAcceptedOutOfRange(id, message);
                return;
            }
 
 
            if (kSource != NULL_KEY && kSource != id)
            {
                debug("already used by another object => reject");
                return;
            }
 
            loginWaitingForPong = FALSE; // whatever the message, it is for me => it satisfies the ping request
 
            if (!isObjectKnow(id))
            {
                debug("asking for permission because kSource is NULL_KEY");
                ////if (!verifyPermission(id, name, message))
                {   llOwnerSay("Would not allow, but allowing anyway for debug."); // ***TEMP***
                    ////return;
                }
            }
 
            debug("Executing: " + (string) kSource);
            execute(name, id, message);
        }
        else if (channel==DIALOG_CHANNEL)
        {
            if (id != llGetOwner())
            {
                return; // only accept dialog responses from the owner
            }
            if (sPendingId!=NULL_KEY)
            {
                if (message=="Yes") // pending request authorized => process it
                {
                    execute(sPendingName, sPendingId, sPendingMessage);
                }
 
                // clear pending request
                sPendingName="";
                sPendingId=NULL_KEY;
                sPendingMessage="";
            }
        }
    }
 
    touch_start(integer num_detected)
    {
        // touched by user => cycle through OFF/ON_PERMISSION/ON_ALWAYS modes
        key toucher=llDetectedKey(0);
        if (toucher==llGetOwner())
        {
            if (kSource != NULL_KEY)
            {
                //
                llOwnerSay("Sorry, you cannot change the relay mode while it is locked.");
                //NS tell us who it is locked by
                if (llKey2Name(kSource) != "") {
                    llOwnerSay("The relay is locked by "+llKey2Name(kSource));
                } else {
                    llOwnerSay("The relay was locked by an unnamed object, key="+(string)kSource+", removing restriction.");
                    releaseRestrictions();
                }
                // try a sensor command
                //llSensor("",llGetKey(kSource),ACTIVE | PASSIVE | SCRIPTED ,96.0,PI);
                return;
            }

            //++nMode;
            //if (nMode>2) nMode=0;
            
            // NS
            set_mode( (nMode+1)%3 );

            if (nMode==MODE_OFF) releaseRestrictions ();
            llOwnerSay (getModeDescription());
        }
    }
 
    changed(integer change)
    {
        if (change & CHANGED_OWNER) 
        {
             llResetScript();
        }
    }
    
    sensor(integer num)
    {
        llOwnerSay("The locking object is within sensor range.");
    }
    
    no_sensor()
    {
        llOwnerSay("The locking object is not found within sensor range - releasing restrictions");
        releaseRestrictions();
    }
        
}

