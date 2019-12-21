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
#include "npc/bhvcall.lsl"
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
string BHVDIALOGINFO = "\nNPC debug options.";
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
//  bhvmsglevdialog
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
    buttons += "Dump log";                      // Dump log now
    if (gVerbose)                               // verbose checkbox 
    {   buttons += "Verbose ☒"; }
    else 
    {   buttons += "Verbose ☐";}
    buttons += "Reset";                         // fill line and add reset button
    gBhvDialogTime = llGetUnixTime();           // timestamp for dialog removal
    llOwnerSay("Popping up dialog box");    // ***TEMP***
    llDialog(toucherid, BHVDIALOGINFO, buttons, BHVDIALOGCHANNEL);
}
//
//  bhvbroadcastmsglev -- send message level to everybody
//
bhvbroadcastmsglev(integer msglev, integer verbose, integer dumplog)
{   gDebugMsgLevel = msglev;                    // set message level
    llOwnerSay("Setting debug message level to " + (string)msglev); // set msg lev
    llMessageLinked(LINK_SET,BHVMSGFROMSCH,llList2Json(JSON_OBJECT,["request","msglev","msglev",msglev]),""); // tell everybody
    debugMsgLevelBroadcast(msglev, verbose, dumplog);             // send to path system
}

#endif // BHVDEBUG
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
    llOwnerSay("Character height: " + (string)gHeight + "m. Width: " + (string)gWidth + "m.");
    pathInit(gWidth, gHeight, CHARACTER_TYPE_A);                    // set up pathfinding system
    pathTurnspeed(CHARACTER_TURNSPEED_DEG*DEG_TO_RAD);              // how fast to turn, rarely changed
    //  Reset all behaviors
    llOwnerSay("Resetting all behaviors.");  // ***TEMP***
    llMessageLinked(LINK_SET,BHVMSGFROMSCH,llList2Json(JSON_OBJECT,["request","reset"]),"");    // reset all behaviors
    //  Display name of character
    string name = llList2String(llGetObjectDetails(getroot(llGetKey()),[OBJECT_NAME]),0);   // name of root prim
    integer spaceIndex = llSubStringIndex(name, " ");
    if (spaceIndex >0)
    {   name  = llGetSubString(name, 0, spaceIndex - 1); }          // first name of character
    //  Set up character
    vector color = <1.0,1.0,1.0>;                                   // white
    llSetText(name, color, 1.0);                                    // set hover text
    llSetTimerEvent(5.0);                                           // for stall timer check only
}
//
//  registerbehavior -- register a new behavior script
//
registerbehavior(string scriptname, integer primnum)
{
    integer bhvix = llListFindList(gBehaviors, [scriptname]);       // index if registered
    if (bhvix >= 0)                                                 // if already registered
    {   assert((bhvix % BHVBEHAVIORSSTRIDE) == 0);                  // must be strided properly 
    } else {
    //  Add new behavior to list
        bhvix = llGetListLength(gBehaviors);                        // index of new behavior
        gBehaviors += [scriptname, primnum, gBehaviorMsgnum, BHVOFFPRIORITY];   // strided entry
        debugMsg(DEBUG_MSG_WARN,"New behavior #" + (string) gBehaviorMsgnum + ": " + scriptname);
        gBehaviorMsgnum++;
    }
    //  Send register reply to appropriate prim.
    //  Will be ignored by behaviors with other string names.
    string jsn = llList2Json(JSON_OBJECT,["reply","register", "scriptname", BHVSCRIPTNAME(bhvix),
        "mnum", BHVMNUM(bhvix), "schedlink",llGetLinkNumber(),"height",gHeight, "width", gWidth]);
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
    ////llOwnerSay("getbhvtorun hits: " + llDumpList2String(bhvindexes,","));   // ***TEMP***   
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
{   debugMsg(DEBUG_MSG_WARN,"Path begin req: " + jsn);              // get things started
    key target = (key)llJsonGetValue(jsn,["target"]);
    vector goal = (vector)llJsonGetValue(jsn,["goal"]);
    float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
    float speed = (float)llJsonGetValue(jsn,["speed"]);
    integer dogged = (target != "") && (target != NULL_KEY);        // dogged mode if pursue target

#ifdef UNITTEST
    //  ***MORE***
    //  ***TEMP DUMMY*** send back a fake normal completion for test purposes
    integer status = 0;
    key hitobj = NULL_KEY;
    llSleep(1.0);   // prevent dummy test from going too fast
    llMessageLinked(BHVLINKNUM(bhvix),BHVMNUM(bhvix),llList2Json(JSON_OBJECT,["reply","pathbegin","status",status,"hitobj",hitobj]),""); 
#else
    pathBegin(target, goal, stopshort, dogged, speed);                     // begin path planning
#endif // UNITTEST
}
//
//  doturn -- behavior wants to do a path request
//
doturn(integer bhvix, string jsn)
{   debugMsg(DEBUG_MSG_WARN,"Turn req: " + jsn);
    float heading = (float)llJsonGetValue(jsn,["heading"]);            // get heading
#ifdef UNITTEST
    //  ***MORE***
    //   ***TEMP DUMMY*** send back a normal completion for test purposes.
    integer status = 0;
    key hitobj = NULL_KEY;
    llSleep(1.0);   // prevent dummy test from going too fast
    llMessageLinked(BHVLINKNUM(bhvix),BHVMNUM(bhvix),llList2Json(JSON_OBJECT,["reply","pathturn","status",status]),""); 
#else
    pathTurn(heading);
#endif // UNITTEST

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
{   debugMsg(DEBUG_MSG_WARN,"Anim req: " + llDumpList2String(anims,","));                // get things started
    string anim = llList2String(anims,0);                       // only first anim for now ***TEMP*** need to fix AO
    llMessageLinked(LINK_THIS, 1,anim,"");                      // tell our AO what we want
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
    {   debugMsg(DEBUG_MSG_WARN, jsn);                        // ***TEMP*** dump incoming JSON
        if (num == BHVMSGTOSCH)                    // one of our behaviors wants to talk to us
        {
            string reqtype = llJsonGetValue(jsn,["request"]);   // get request type
            if (reqtype == "register")               // a behavior is checking in
            {   registerbehavior(llJsonGetValue(jsn,["scriptname"]),
                    (integer)llJsonGetValue(jsn,["linknum"]));   // register this behavior
                return;
            }
            //  All requests after this point have a mnum
            integer mnum = (integer)llJsonGetValue(jsn,["mnum"]);    // message number, indicates who is calling
            integer bhvix = findbehavior(mnum);     // look up behavior
            if (bhvix < 0)
            {   debugMsg(DEBUG_MSG_ERROR,"Invalid mnum: " + jsn); return; }
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
        bhvmsglevdialog(toucherid);
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
        } else {
            llSay(DEBUG_CHANNEL, "Dialog option bug: "+ message); // bad
            return;
        }
        bhvbroadcastmsglev(buttonIndex, gVerbose, dumplog);   // tell everybody
    }
#endif // BHVDEBUG
  
}




