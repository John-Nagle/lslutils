//
//  bhvscheduler.lsl --- Behavior scheduler for NPCs.
//
//  Animats
//  November, 2019
//  License: GPLv3.
//
//
//  This is the scheduler for the behavior tasks of Animats NPCs.
//  One behavior runs at a time, the highest priority one.
//  This task controls those behavior tasks.
//
//  Tasks do all movement and animation by sending messages to this
//  task, to avoid race conditions.
//
//  Startup
//
//  At startup, this task sends a broadcast message to all
//  scripts in all prims, asking behaviors to re-register. 
//  Behaviors send a message back, with their script name,
//  and prim number. The scheduler registers them, 
//  assigns them a "num" value for messages, and 
//  replies to the register message to the correct prim
//  on a common channel. The behavior is then live,
//  but stopped and at zero priority. It can then
//  request a run priority from the scheduler.
//  Behaviors, when they start up, try to register
//  every few seconds until registered.
//
//  Run
//
//  When the scheduler selects a behavior to run, it sends
//  a "run" message to the desired prim and num value to enter
//  run mode. This includes a serial number of the run cycle,
//  to prevent race conditions. The behavior than has control
//  and can make path planning and animation requests by sending
//  messages to the scheduler. These include the "num" value and
//  the run cycle number. Those have to match the scheduler's
//  currently active behavior, or the request will be rejected.
//  This prevents race conditions during behavior switching.
//  Rejected requests produce a "stop" message from scheduler to
//  behavior.
//
//  Stop
//
//  A stop message to a behavior may indicate that it has been
//  preempted by a higher priority behavior. The behavior can
//  no longer have movement requests executed. But it can still
//  compute and sense the world. When a behavior is done
//  and no longer needs to run, it can request priority 0,
//  which will yield a "stop" message, and cause the scheduler
//  to find something else to do.
//
//  
#include "assert.lsl"
#include "debugmsg.lsl"
#include "npc/bhv/bhvcall.lsl"
#include "npc/pathcall.lsl"

//
//  Constants
//
#define BHVBEHAVIORSSTRIDE 4    // stride of behavior list
#define BHVOFFPRIORITY  0       // do not run if at this priority
#define BHVMNUMSTART -1999      // link message nums (mnum) for scripts start here
#define BHVMNUMEND  -1900       // max of 100 behaviors 

//
//  Character defaults. Behaviors override the speed.
//
#define CHARACTER_SPEED  2.5                // (m/sec) speed
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
//
//  Useful fns.
//
#define getroot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))   // get root key of object.

//
//  Globals
//
list gBehaviors;                // [scriptname,linknum,msgnum,priority]
#define BHVSCRIPTNAME(ix) (llList2String(gBehaviors,(ix)))  // structures, the hard way
#define BHVLINKNUM(ix) (llList2Integer(gBehaviors,(ix)+1))
#define BHVMNUM(ix) (llList2Integer(gBehaviors,(ix)+2))
#define BHVPRIORITY(ix) (llList2Integer(gBehaviors,(ix)+3))
integer gBehaviorMsgnum = BHVMNUMSTART; // msg number for comm with behavior

integer gActiveBehavior = -1;   // index into active behavior table 
integer gActivePriority = 0;    // priority of active behavior
float gHeight = -1;             // height and width for NPC
float gWidth = -1;
integer gVerbose = FALSE;       // verbose mode on for debug
integer gChartype = CHARACTER_TYPE_A;   // humanoid
integer gCharacterRunning = TRUE;  // system is running
string  gCharacterName = "NPC"; // reset from first word of name

//
//  Services.
//
integer gAnimService = -1;                                          // animation service
//
//  Debug system
//
#ifdef BHVDEBUG                                                     // if debug menu enabled
//
//  Debug dialog info
//
#define BHVDIALOGCHANNEL -22748364                                  // arbitrary and local
integer gBhvDialogListenHandle;
integer gBhvDialogTime = 0;                             
list BHVMSGLEVBUTTONS = DEBUG_MSG_NAME_LIST;
string BHVDIALOGDEBUG = "Debug options";
string BHVDIALOGMAIN = "Owner options";
#define BHVDIALOGTIMEOUT 30.0                                       // remove listen

//
//  bhvmsglevindex -- get msg level from button pushed
//
integer bhvmsglevindex(string message)
{   string pstring = llStringTrim(llDumpList2String(llParseString2List(message,["⬤"],[" "]),""),STRING_TRIM);
    integer i;
    for (i = 0; i < llGetListLength(BHVMSGLEVBUTTONS); i++)
    {   if (llList2String(BHVMSGLEVBUTTONS,i) == pstring) { return(i); }}  // find
    return(-1);                         // no find
}
//
//  bhvmsglevdialog -- debug dialog
//
//  Message level (Error, Warning, Info) and Verbose on/off.
//
bhvmsglevdialog(key toucherid)
{   llListenRemove(gBhvDialogListenHandle);                                        // delete any old listens for this                  
    gBhvDialogListenHandle = llListen(BHVDIALOGCHANNEL, "", toucherid, "");       // listening for reply
    list buttons = [];                          // no buttons yet
    integer i;
    for (i=0; i<llGetListLength(BHVMSGLEVBUTTONS); i++)
    {   string s = llList2String(BHVMSGLEVBUTTONS,i);   // get msg level name
        if (i == gDebugMsgLevel) { s = "⬤ " + s; }      // add dot
        buttons += s;
    }
    buttons += [" "," "];                       // filler to align buttons
    buttons += "Dump log";                      // Dump log now
    if (gVerbose)                               // verbose checkbox 
    {   buttons += "Verbose ☒"; }
    else 
    {   buttons += "Verbose ☐";}
    gBhvDialogTime = llGetUnixTime();           // timestamp for dialog removal
    ////llOwnerSay("Popping up dialog box");    // ***TEMP***
    llDialog(toucherid, BHVDIALOGDEBUG, buttons, BHVDIALOGCHANNEL);
}
//
//  bhvownerdialog -- main owner dialog
//
//  Stop, Run, Reset, and Debug
//
bhvownerdialog(key toucherid)
{   llListenRemove(gBhvDialogListenHandle);                                        // delete any old listens for this                  
    gBhvDialogListenHandle = llListen(BHVDIALOGCHANNEL, "", toucherid, "");       // listening for reply
    list buttons = [];                          // no buttons yet
    string s = "Stop";                          // stop and run get a dot if active
    if (!gCharacterRunning) { s = "⬤ " + s; } 
    buttons += s;
    s = "Run";
    if (gCharacterRunning) { s = "⬤ " + s; } 
    buttons += s;
    buttons += "Reset";
    buttons += "Debug";
    gBhvDialogTime = llGetUnixTime();           // timestamp for dialog removal
    ////llOwnerSay("Popping up dialog box");    // ***TEMP***
    llDialog(toucherid, BHVDIALOGMAIN, buttons, BHVDIALOGCHANNEL);
}
#endif // BHVDEBUG
//
//  bhvbroadcastmsglev -- send message level to everybody
//
bhvbroadcastmsglev(integer msglev, integer verbose, integer dumplog)
{   gDebugMsgLevel = msglev;                    // set message level
    llOwnerSay("Setting debug message level to " + (string)msglev); // set msg lev
    ////llMessageLinked(LINK_SET,BHVMSGFROMSCH,llList2Json(JSON_OBJECT,["request","msglev","msglev",msglev]),""); // tell everybody
    debugMsgLevelBroadcast(msglev, verbose, dumplog);             // send to path system
}
//
//  init
//
init()
{
    //  Reset to ground state
    gBehaviors = [];            // [scriptname,primnum,msgnum,priority]
    gBehaviorMsgnum = BHVMNUMSTART; // msg number for comm with behavior
    gActiveBehavior = -1;       // index into active behavior table 
    gActivePriority = 0;        // priority of active behavior
    gActiveToken = 0;           // sequence number for activity
    //
    //  Init the path planning system
    //
    vector scale = llGetScale();                                    // scale of character, usually a dummy box
    gHeight = scale.z;                                              // height is scale
    gWidth = llVecMag(<scale.x,scale.y,0.0>);                       // diameter of enclosing circle
    debugMsg(DEBUG_MSG_WARN,"Resetting. character height: " + (string)gHeight + "m. Width: " + (string)gWidth + "m.");
    pathInit(gWidth, gHeight, gChartype);                           // set up pathfinding system
    pathTurnspeed(CHARACTER_TURNSPEED_DEG*DEG_TO_RAD);              // how fast to turn, rarely changed
    //  Reset all behaviors
    ////llOwnerSay("Resetting all behaviors.");  // ***TEMP***
    llMessageLinked(LINK_SET,BHVMSGFROMSCH,llList2Json(JSON_OBJECT,["request","reset"]),"");    // reset all behaviors
    //  Display name of character
    gCharacterName = llList2String(llGetObjectDetails(getroot(llGetKey()),[OBJECT_NAME]),0);   // name of root prim
    integer spaceIndex = llSubStringIndex(gCharacterName, " ");
    if (spaceIndex >0)
    {   gCharacterName  = llGetSubString(gCharacterName, 0, spaceIndex - 1); }          // first name of character
    //  Set up character
    displayname();
    llSetTimerEvent(5.0);                                           // for stall timer check only
    bhvbroadcastmsglev(gDebugMsgLevel, gVerbose, FALSE);            // set initial message level
}
//
//  displayname  -- display the NPC's name and some other info
//
displayname()
{   string msg = gCharacterName;
    vector color = <1.0,1.0,1.0>;                                   // white
    if (!gCharacterRunning)                                         // if stopped
    {   color = <1.0,0.0,0.0>;                                      // red
        msg = "STOPPED\n" + msg;                                    // note stopped
    }
    llSetText(msg,color,1.0);                                       // set the message
}
//
//  registerbehavior -- register a new behavior script
//
registerbehavior(string scriptname, string service, integer primnum)
{
    integer bhvix = llListFindList(gBehaviors, [scriptname]);       // index if registered
    if (bhvix >= 0)                                                 // if already registered
    {   assert((bhvix % BHVBEHAVIORSSTRIDE) == 0);                  // must be strided properly 
    } else {
    //  Add new behavior to list
        bhvix = llGetListLength(gBehaviors);                        // index of new behavior
        gBehaviors += [scriptname, primnum, gBehaviorMsgnum, BHVOFFPRIORITY];   // strided entry
        debugMsg(DEBUG_MSG_WARN,"New behavior #" + (string) gBehaviorMsgnum + ": " + scriptname);
        llOwnerSay("New behavior #" + (string) gBehaviorMsgnum + ": " + scriptname); // ***TEMP***
        gBehaviorMsgnum++;
    }
    //
    //  Record services we know about. Currently used only for "anim" service,
    //  which does the animations.
    if (service != "")
    {   if (service == "anim")
        {   gAnimService = bhvix;                                   // save its index for sending
        } else {
            llOwnerSay("Warning: unknown service name: " + service);// unknown service
        }
    }
    //  Send register reply to appropriate prim.
    //  Will be ignored by behaviors with other string names.
    string jsn = llList2Json(JSON_OBJECT,["reply","register", "scriptname", BHVSCRIPTNAME(bhvix),
        "mnum", BHVMNUM(bhvix), "schedlink",llGetLinkNumber(),"height",gHeight, "width", gWidth,"chartype",gChartype]);
    llMessageLinked(BHVLINKNUM(bhvix),BHVMSGFROMSCH, jsn, "");
}

//
//  findbehavior -- find behavior by mnum
//
integer findbehavior(integer mnum)
{
    integer bhvix;
    for (bhvix=0; bhvix<llGetListLength(gBehaviors); bhvix += BHVBEHAVIORSSTRIDE)
    {   if (BHVMNUM(bhvix) == mnum)                  // if find
        {   return(bhvix); }                        // return index into strided list
    }
    return(-1);                                     // no find  
}
//
//  setpriority -- set priority of a behavior
//
setpriority(integer bhvix, integer pri)
{   assert(pri >= 0);
    gBehaviors = llListReplaceList(gBehaviors,[pri],bhvix+3,bhvix+3); // replace priority
    if (bhvix == gActiveBehavior)                   // if request is from the currently active behavior
    {   schedbhv();                                 // either priority drop or some kind of out of sync condition. Reschedule.
        return;
    }
    if (pri > gActivePriority)                      // this will start a new task
    {
        schedbhv();                                 // run the scheduler
    } 
}

//
//  getbhvtorun -- get highest priority behavior. 
//
//  Inefficient, but probably not worth optimizing; this is called maybe several times
//  a minute.
//
//  Returns strided index into behaviors or -1.
//
integer getbhvtorun()
{
    integer ix;                                 // index into strided list
    integer pri = BHVOFFPRIORITY + 1;           // min priority of interest
    //  Get list of behaviors at winning priority
    list bhvindexes = [];                       // ones at highest priority
    for (ix=0; ix<llGetListLength(gBehaviors); ix += BHVBEHAVIORSSTRIDE)
    {   integer ixpri = BHVPRIORITY(ix);        // priority at this index
        if (ixpri > pri)                        // if new highest priority
        {   bhvindexes = [ix]; pri = ixpri; }   // restart list
        else if (ixpri == pri)                  // if same as higest priority
        {   bhvindexes += ix; }                 // add to list
    }
    //  Pick one at random from list
    integer len = llGetListLength(bhvindexes);  // number of tasks we could run
    if(len <= 0) { return(-1); }                // nothing ready to run
    integer randtask = (integer)llFrand(llGetListLength(bhvindexes));
    assert(randtask < len);                     // do not entirely trust llFrand's upper bound
    return(llList2Integer(bhvindexes,randtask));// return some random bhv at this pri
}
//
//  schedbhv - schedule and start next behavior
//
schedbhv()
{
    if (gActiveBehavior >= 0)                   // stop whatever was running
    {   stopbhv(gActiveBehavior);   
        gActiveBehavior = -1;                   // nobody running
        gActivePriority = 0;                
        gActiveToken = (gActiveToken+1)%(PATHMAXUNSIGNED-1);// advance run serial number, nonnegative
    }
    if (!gCharacterRunning) { return; }            // if shutdown, return
    integer bhvix = getbhvtorun();              // get next behavior to run
    if (bhvix < 0) { return; }                  // nothing to run
    //  Starting new task
    gActiveBehavior = bhvix;
    gActivePriority = BHVPRIORITY(bhvix);       // priority of this behavior
    startbhv(bhvix);
}
//
//  startbhv  -- start behavior
//
startbhv(integer bhvix)
{
    debugMsg(DEBUG_MSG_INFO,"Starting " + BHVSCRIPTNAME(gActiveBehavior));
    pathIgnoreOldReplies();                     // make any old replies from path system stale to avoid getting out of sync
    llMessageLinked(BHVLINKNUM(bhvix),BHVMNUM(bhvix),llList2Json(JSON_OBJECT,["request","start","token",gActiveToken]),""); 
}

//
//  stopbhv -- stop behavior
//
stopbhv(integer bhvix)
{   debugMsg(DEBUG_MSG_INFO,"Stopping " + BHVSCRIPTNAME(gActiveBehavior)); 
    llMessageLinked(BHVLINKNUM(bhvix),BHVMNUM(bhvix),llList2Json(JSON_OBJECT,["request","stop"]),""); 
}

//
//  dopathbegin -- behavior wants to do a path request
//
dopathbegin(integer bhvix, string jsn)
{   debugMsg(DEBUG_MSG_NOTE,"Path begin req: " + jsn);              // get things started
    key target = (key)llJsonGetValue(jsn,["target"]);
    vector regioncorner = (vector)llJsonGetValue(jsn,["regioncorner"]); // send region corner along with goal
    vector goal = (vector)llJsonGetValue(jsn,["goal"]);
    float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
    float speed = (float)llJsonGetValue(jsn,["speed"]);
    pathBegin(target, regioncorner, goal, stopshort, speed);                     // begin path planning
}
//
//  doturn -- behavior wants to do a path request
//
doturn(integer bhvix, string jsn)
{   debugMsg(DEBUG_MSG_NOTE,"Turn req: " + jsn);
    float heading = (float)llJsonGetValue(jsn,["heading"]);            // get heading
    pathTurn(heading);

}
//
//  pathUpdateCallback -- the path system has completed a request from us.
//
//  Send it back to the appropriate behavior.
//
pathUpdateCallback(integer status, key hitobj)
{   integer bhvix = gActiveBehavior;
    if (bhvix < 0) { return; }          // no behavior active?
    llMessageLinked(BHVLINKNUM(bhvix),BHVMNUM(bhvix),llList2Json(JSON_OBJECT,["reply","pathbegin","status",status,"hitobj",hitobj]),""); 
}
//
//  doanim  -- behavior wants to do an animation request.
//  
//  This affects only idle anims for now. So, no tray carrying, etc. at this time.
//
//  Anim requests don't get a callback.
//
doanim(integer bhvix, list anims)
{   debugMsg(DEBUG_MSG_NOTE,"Anim req: " + llDumpList2String(anims,","));                // get things started
    if (gAnimService < 0)
    {   debugMsg(DEBUG_MSG_ERROR,"No animation service registered."); return; } // We're missing a system component
    //  Send to the anim service in the behaviors prim. This is so the anims go in the behaviors prim, whic will be modifiable.
    llMessageLinked(BHVLINKNUM(gAnimService), BHVMNUM(gAnimService), llList2Json(JSON_OBJECT,["request","anim", "stand", llList2Json(JSON_ARRAY, anims)]),"");
}
//
//
//  Main program
//
default
{
    on_rez(integer start_param)
    {
        llResetScript();
    }
    
    changed(integer change)
    {
        if (change & CHANGED_REGION_START)                      // region restart
        {   llResetScript(); }                                  // reset the NPC completely
    }
 
    state_entry()
    {
        init();
    }

    timer()                                         // timer tick
    {     
        pathTick();                                 // for stall timer
#ifdef BHVDEBUG
        {   //  Get rid of dead dialog if any
            if (gBhvDialogListenHandle != 0)
            {   if (llGetUnixTime() - gBhvDialogTime > BHVDIALOGTIMEOUT) // if dead dialog listen to flush
                {   llListenRemove(gBhvDialogListenHandle);
                    gBhvDialogListenHandle = 0; 
                }
            }       
        } 
#endif // BHVDEBUG
    }
    link_message(integer sender_num, integer num, string jsn, key id)
    {   ////debugMsg(DEBUG_MSG_NOTE, jsn);                        // ***TEMP*** dump incoming JSON
        if (num == BHVMSGTOSCH)                    // one of our behaviors wants to talk to us
        {
            string reqtype = llJsonGetValue(jsn,["request"]);   // get request type
            if (reqtype == "register")               // a behavior is checking in
            {   registerbehavior(llJsonGetValue(jsn,["scriptname"]),
                    llJsonGetValue(jsn,["service"]),
                    (integer)llJsonGetValue(jsn,["linknum"]));   // register this behavior
                return;
            }
            //  All requests after this point have a mnum
            integer mnum = (integer)llJsonGetValue(jsn,["mnum"]);    // message number, indicates who is calling
            integer bhvix = findbehavior(mnum);         // look up behavior
            if (bhvix < 0)
            {   debugMsg(DEBUG_MSG_WARN,"Unregistered behavior: " + jsn); // probably still initializing
                return;
            }
            if (reqtype == "priority")                  // wants to set priority
            {   
                integer pri = (integer)llJsonGetValue(jsn,["pri"]);      // priority
                if (pri < 0) { pri = 0; }               // prevent breaking scheduler
                setpriority(bhvix, pri);                // set priority - may start the behavior
                return;
            }
            //  All requests past this point are only valid from the active behavior
            //  with the current token.
            integer token = (integer)llJsonGetValue(jsn,["token"]);
            if (bhvix != gActiveBehavior || token != gActiveToken)  // if not allowed to control NPC now
            {   debugMsg(DEBUG_MSG_WARN,"Stale request from " + BHVSCRIPTNAME(bhvix) + ": " + jsn); // ignored, stale
                stopbhv(bhvix);                         // tell it to stop trying
                return;
            }
            if (reqtype == "pathbegin")                 // behavior wants to do a path operation
            {   dopathbegin(bhvix, jsn);                // do the path begin operation
                return;
            }
            else if (reqtype == "pathturn")             // behavior wants to do a path operation
            {   doturn(bhvix, jsn);                     // do a turn operation
                return;
            }
            else if (reqtype == "anim")                 // do animation request
            {   string animsjson = llJsonGetValue(jsn,["anims"]);   // get anim list
                list anims = llJson2List(animsjson);    // get anims as string
                doanim(bhvix, anims);
                return;
            }
            else if (reqtype == "say")                  // say something, from root prim
            {   string msg = llJsonGetValue(jsn,["msg"]);
                llSay(0,msg);
                return;
            }
            debugMsg(DEBUG_MSG_ERROR,"Invalid message to behavior scheduler: " + jsn);    // behavior error

        } else if (num == PATHSTARTREPLY)               // the path system is done doing something for us
        {
            pathLinkMsg(sender_num, num, jsn, id);      // handle
        }
    }
    
    collision_start(integer num_detected)
    {   key hitobj = llDetectedKey(0);                  // first object hit
        if (gActiveBehavior < 0) { return; }            // no behavior active
        //  Tell active behavior about it.
        llMessageLinked(BHVLINKNUM(gActiveBehavior),BHVMNUM(gActiveBehavior),llList2Json(JSON_OBJECT,["request","collisionstart","hitobj",hitobj]),""); 
    }
    
#ifdef BHVDEBUG                                            // enable debug interface
    
    touch_start(integer total_number)               // brings up debug menu
    {   key toucherid = llDetectedKey(0);
        llListenRemove(gBhvDialogListenHandle);     // just in case                 
        gBhvDialogListenHandle = 0;    
        if (toucherid != llGetOwner())              // owner only              
        {   return;
        }
        bhvownerdialog(toucherid);                  // bring up owner dialog
    }
      
    listen(integer channel, string name, key id, string message)
    {   
        llListenRemove(gBhvDialogListenHandle);     // remove dialog
        gBhvDialogListenHandle = 0;                 // no longer listening 
        integer dumplog = FALSE;                    // not dumping log yet 
        integer buttonIndex = bhvmsglevindex(message); // is it a valid button?
        if (buttonIndex >= 0)
        {   
            gDebugMsgLevel = buttonIndex;           // new button index
            
        } else if (message == "Reset")              // reset this script which restarts everybody
        {   llResetScript(); 
        } else if (message == " ")                  // unused button
        {   return; 
        } else if (llSubStringIndex(message,"Verbose") >= 0)    // toggle verbose mode
        {   gVerbose = !gVerbose;                   // toggle verbose mode 
        } else if (message == "Dump log")           // time to dump log
        {   dumplog = TRUE;                         // do it
        } else if (message == "Debug")
        {   bhvmsglevdialog(id);                    // bring up debug options menu
        } else if (llSubStringIndex(message,"Run") >= 0)  // with or without dot
        {   if (!gCharacterRunning)                    // if stopped
            {   gCharacterRunning = TRUE;              // now running
                schedbhv();                            // start scheduler
                displayname();                         // show as running
            }
        } else if (llSubStringIndex(message,"Stop") >= 0) // with or without dot
        {   if (gCharacterRunning)                     // if running
            {   gCharacterRunning = FALSE;             // now stopped
                pathStop();                            // shut down any movement
                schedbhv();                            // run scheduler to cause stop
                llOwnerSay("Stopping " + gCharacterName);
                displayname();                          // show as stopped
            }
        } else {
            llSay(DEBUG_CHANNEL, "Dialog option bug: "+ message); // bad
            return;
        }
        bhvbroadcastmsglev(gDebugMsgLevel, gVerbose, dumplog);   // tell everybody msg level changed
    }
#endif // BHVDEBUG
}




