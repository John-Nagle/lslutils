//
//  Escalator manager for Second Life
//
//  Animats
//  nagle@animats.com
//  April, 2020
//  License: GPL
//
//  Rezzes and aligns an object from this object's content.
//  Handles resizing.
//
//  This is for objects which have an associated object which cannot be made a child,
//  cannot be made a prims, usually
//  because they are for keyframe animation.
//
//  Configuration
//
//  Additional object we must rez.
//
//  For multiple sizes of escalator.
//
//  Steps ref point is at center of upper edge of top step.
//  Frame ref point is at center of edge of platform.
//  We align those and offset from there.
//  Positions captured from the edit window of a successful assembly.
//
//  Step length is 0.33
//  Step height is 0.23
//  Step width is 1.00
//
#ifdef DEBUG                                        // define DEBUG for messages
#define DEBUGPRINT(msg) { llOwnerSay(msg); }
#else
#define DEBUGPRINT(msg) 
#endif
string  STEPSNAME = "steps";                        // Steps object must contain " 27 steps" or appropriate number
float   STEPHEIGHT = 0.23;                          // used to calculate number of steps needed
float   NONSTEPHEIGHT = 2.25;                       // part of escalator height not devoted to steps
integer EXTRASTEPS = 1;                             // steps to add so motion works
integer STEPSANIMLINK = 2;
integer STEPSANIMFACE = ALL_SIDES;
integer RAILANIMLINK = 1;
integer RAILANIMFACE = 3;
integer TRAFFICLIGHTLINK = 1;
integer TRAFFICLIGHTFACE = 4;
integer ESTOPFACE = 6;                              // emergency stop button
float   ESTOPTIME = 30.0;                           // restart after this long after estop

float STEPSANIMRATE = -2.0;                         // steps animation speed
float RAILANIMRATE = 0.80;                          // railing rate per meter of escalator length
float TRAFFICLIGHTGLOW = 0.2;                       // glow brightness for red and green lights

vector STEPSOFFSET = <0.0, -0.33, -0.23>;             // fine tuning of step position so passengers are carried properly
vector TOPREFOFFSET = <0.0, 2.15, -1.0>;            // rel position of center of top step edge to bounding box

vector INITIALROTANG = <0,0,0>;                 // initial rotation angle, radians

integer MAXINT = 16777216;                      // 2^24
integer MINCHAN = 100000;                       // for range of allowed unique channel numbers
integer COMMANDCHANNEL = -8372944;              // randomly chosen channel. Not secret


string SOUNDNAME="escalator1";          // escalator sound effect
string SOUNDBRAKE="escalatorbrake";     // escalator emergency stop
float VOLUME=0.20;                      // audio level


//  Global variables
integer gRezzedObjectID = 0;            // channel for deleting rezzed object
vector gPrevPos = ZERO_VECTOR;
rotation gPrevRot = ZERO_ROTATION;
integer gDirection = 0;                 // direction of motion (-1, 0, or +1)
integer gLastDirection = 1;             // last moving direction. Remote uses this.
integer gLocked = FALSE;                // owner only
integer gDialogChannel;                 // for talking to user
integer gDialogHandle = 0;              // listening for dialog response
vector gScale = ZERO_VECTOR;            // scale of object
integer gCommandHandle = 0;             // for command listener
integer gReplyHandle = 0;               // for steps reply listener
integer gEstopPushed = FALSE;           // true if e-stop pushed
vector gTopref = ZERO_VECTOR;           // reference position of top of steps

//
//  vector_mult - multiply vector by vector elementwise
//
vector vector_mult(vector v0, vector v1)
{   return (<v0.x*v1.x, v0.y*v1.y, v0.z*v1.z>); }// there should be a builtin for this

//
//  setup_command_listen -- start listening for commands from a remote control
//
setup_command_listen()
{  
    llListenRemove(gCommandHandle);                  // kill any existing listen
    gCommandHandle = llListen(COMMANDCHANNEL,"", "","");    // listen on command channel
}
    
command_listen(integer channel, string msgname, key id, string message)
{
    key ownerkey = llGetOwnerKey(id);                       // owner of sender of message
    if (ownerkey != llGetOwner()) { return; }               // ignore if msg not from owner
    DEBUGPRINT("Incoming msg: " + message);
    string request = llJsonGetValue(message, ["request"]);
    if (request == JSON_NULL || request == JSON_INVALID)  { return; }                  // return silently if not "request"
    string name = llJsonGetValue(message, ["name"]);
    string id = llJsonGetValue(message, ["id"]);
    string dir = llJsonGetValue(message, ["dir"]); 
    //  To match, the request must either match on ID, or the name in the request must be
    //  a substring of our object name. Case insensitive match.
    if ((name == JSON_INVALID || name == JSON_NULL)   // JSON parse failed
            || (llSubStringIndex(llToLower(llGetObjectName()), llToLower(name)) < 0 && id != (string)llGetKey()))
    {   llOwnerSay("Message invalid or not for us: " + message); return; }
    do_command(request, dir);                                    // do the command        
}

dialog_listen(integer channel, string name, key id, string message)
{
    llListenRemove(gDialogHandle);              // turn off the listener
    gDialogHandle = 0;                          // once only
    //  Process response
    integer direction = gDirection;
    if (llSubStringIndex(message, "Up") >= 0) { direction = 1; }
    else if (llSubStringIndex(message, "Down") >= 0) { direction = -1; }
    else if (llSubStringIndex(message, "Stop") >= 0) { direction = 0; }
    else if (llSubStringIndex(message, "Lock") >= 0) { gLocked = !gLocked; } 
    set_escalator_state(direction);             // control the escalator
}

//
//  do the command from the remote control
//
do_command(string request, string dir)
{   if (request == "start")
    {   if (dir == "1" || dir == "-1") { gLastDirection = (integer) dir; }    // set dir if specified
        set_escalator_state(gLastDirection); // restart in previous direction
    } else if (request == "stop")
    {   set_escalator_state(0);             // stop 
    } else if (request == "status")         // just report current condition
    {
        string stat = "stopped";
        if (gDirection != 0)
        {   stat = "running";                               // true if running
        }
        //  Tell remote controller what we are doing.
        llShout(COMMANDCHANNEL, llList2Json(JSON_OBJECT, ["reply", "status", "status", stat, "dir", (string) gDirection, "name", llGetObjectName()]));                          
    } else {   llShout(COMMANDCHANNEL, llList2Json(JSON_OBJECT, ["reply", "error", "error", "Invalid request:" + request]));}
    
}

//
//  set_traffic_light -- set traffic lights to up, down, or stop.
//
set_traffic_light(integer dir)
{   float texyoffset = 0;                   // the texture has 4 lights in 4 quadrants
    float texxoffset = 0.5;                 // show lights as dark     
    float glow = 0.0;                       // no glow
    integer fullbright = FALSE;
    if (dir < 0)                            // down
    {   texyoffset = 0.5;                   // reverses red and green
        texxoffset = 0.0;                    // red/green area of texture
        glow = TRAFFICLIGHTGLOW;            // glow on
        fullbright = TRUE;                  // emit light
    } else if (dir > 0)                     // up 
    {   texyoffset = 0.0; 
        texxoffset = 0.0;                   // red/green area of texture
        glow = TRAFFICLIGHTGLOW;
        fullbright = TRUE;
    } 
    llOffsetTexture(texxoffset, texyoffset, TRAFFICLIGHTFACE);  // selects proper area of texture
    llSetLinkPrimitiveParamsFast(TRAFFICLIGHTLINK,              // sets full bright and glow
        [PRIM_FULLBRIGHT, TRAFFICLIGHTFACE, fullbright, 
        PRIM_GLOW, TRAFFICLIGHTFACE, glow]);
}

set_escalator_anims()
{
    //  Animation of flat steps which are part of the escalator base and do not move.
    if (gDirection != 0)
    {   llSetLinkTextureAnim(RAILANIMLINK, ANIM_ON|LOOP|SMOOTH, 
                RAILANIMFACE, 1, 1, 1.0, -1.0, RAILANIMRATE*gDirection);
        llSetLinkTextureAnim(STEPSANIMLINK, ANIM_ON|LOOP|SMOOTH, 
                STEPSANIMFACE, 1, 1, 1.0, -1.0, STEPSANIMRATE*gDirection); // start animation
        llLoopSound(SOUNDNAME, VOLUME);                 // escalator sound 
    }
    else
    {   //  Stop. Maintains same orientation as active anim
        llSetLinkTextureAnim(STEPSANIMLINK, SMOOTH, STEPSANIMFACE, 1, 1, 1.0, -1.0, 1.0); // stop previous animation
        llSetLinkTextureAnim(RAILANIMLINK, SMOOTH, RAILANIMFACE, 1, 1, 1.0, -1.0, 1.0); // stop previous animation
        llStopSound();                                  // silence
    }

}

set_escalator_state(integer direction)
{
    if (direction == gDirection) { return; }    // no change in dir
    gEstopPushed = FALSE;                       // clear any e-stop
    gDirection = direction;                     // change direction
    llShout(gRezzedObjectID, llList2Json(JSON_OBJECT, // tell steps to change direction
        ["command","DIR", "direction", (string)gDirection]));
    set_traffic_light(gDirection);                      // set green and red lights
    if (gDirection != 0)
    {   llSleep(5.0);   }                       // allow time for step state change before changing anims
    //  Animation of flat steps which are part of the escalator base and do not move.
    set_escalator_anims();
    string stat = "stopped";
    if (gDirection != 0)
    {   stat = "running";                               // true if running
        gLastDirection = direction;                     // for remote stop/start
    }
    //  Tell remote controller what we are doing.
    llShout(COMMANDCHANNEL, llList2Json(JSON_OBJECT, ["reply", "status", "status", stat, "dir", (string) gDirection, "name", llGetObjectName()]));                          
}
//
//  find_object_by_namepart -- look up object in prim inventory by a substrint of the name
//
string find_object_by_namepart(string namepart) 
{
    integer cnt =  llGetInventoryNumber(INVENTORY_OBJECT); // number of objects in prim inventory
    integer i;
    for (i=0; i<cnt; i++)                               // search for a step object with the right number of steps
    {   string objname = llGetInventoryName(INVENTORY_OBJECT,i); // nth inventory object
        if (llSubStringIndex(objname, namepart) >= 0)   // if substring match
        {   return(objname);    }                       // find
    }
    return("");                                         // no find
}
//
//  place_steps -- rez and place object
//
//  Steps are rezzed at the same place as the escalator frame, then adjusted by the steps themselves to align.
//
place_steps(string name)
{
    //  Delete old object
    if (gRezzedObjectID) 
    {   
        llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,["command","DIE"]));
    }
    //  Get reference point for escalator frame.
    list bounds = llGetBoundingBox(llGetKey());         // get bounds of frame
    vector lobound = llList2Vector(bounds,0);           // low bound, own coords
    vector hibound = llList2Vector(bounds,1);           // high bound, own coords
    float stepareaheight = hibound.z - lobound.z - NONSTEPHEIGHT;               // height 
    integer stepcount = llCeil(stepareaheight / STEPHEIGHT) + EXTRASTEPS; // number of steps needed
    DEBUGPRINT("Height " + (string)stepareaheight + " needs " + (string)stepcount + " steps."); // ***TEMP*** 
    //  The object's inventory may contain step sets of various sizes. This rezzes the appropriate one.
    string namepart = " " + (string) stepcount + " " + name;  // steps object must contain space + number of steps + name
    name = find_object_by_namepart(namepart);           // look up object
    if (name == "")                                     // if no find
    {   llSay(DEBUG_CHANNEL, "No steps of the right size in the prim. Need \"" + namepart + "\".");
        return;                                         // can't start
    }
    gTopref = <(hibound.x+lobound.x)*0.5,lobound.y,hibound.z>;   // center of top bound line
    DEBUGPRINT("Top ref, local, unadjusted: " + (string)gTopref);    // ***TEMP***
    rotation rot = llGetRot();                          // rotation to world
    gTopref = (gTopref + TOPREFOFFSET + STEPSOFFSET)*rot + llGetPos(); 
    DEBUGPRINT("Top ref, global, adjusted: " + (string)gTopref);    // ***TEMP***
    //  Rez new object
    rot = rot * llEuler2Rot(INITIALROTANG*DEG_TO_RAD); // rotate before rezzing
    vector pos = llGetPos();            // rez at escalator root, adjust later in steps
    //  unique integer channel number in range frand can handle
    integer randomid = - (integer)(llFrand(MAXINT-MINCHAN) + MINCHAN);
    gRezzedObjectID = randomid;                         // random channel for comms with steps
    gReplyHandle = llListen(gRezzedObjectID,"", "","");    // listen on reply channel  
    llRezObject(name, pos, <0,0,0>, rot, randomid); // create the object
    gPrevPos = llGetPos();                  // previous location
    gPrevRot = llGetRot();
    DEBUGPRINT("Placed " + name + " id: " + (string) randomid); // ***TEMP***
}

move_check() 
{   //  Check whether escalator moved. via edit.
    if ((llVecMag(llGetPos()-gPrevPos) > 0.001)     // if we moved, delete and replace object
    || (llFabs(llAngleBetween(llGetRot(),gPrevRot)) > 0.0005))
    {
        place_steps(STEPSNAME);
    }
}
//
//  estop_pushed  -- someone pushed emergency stop
//
estop_pushed()
{
    DEBUGPRINT("E-Stop pushed."); 
    if (gDirection == 0)                                // must be in normal run
    {   return; }                                       // ignore if not running                      
    set_escalator_state(0);                             // stop escalator
    llPlaySound(SOUNDBRAKE,1.0);                        // play brake sound  
    gEstopPushed = TRUE;
    llSetTimerEvent(ESTOPTIME);                         // restart after time interval
}

default
{
    state_entry()
    {   gScale = llGetScale();                          // save object size
        setup_command_listen();
        gDialogChannel = - (integer)(llFrand(MAXINT-MINCHAN) + MINCHAN);  // for dialog msgs
        string objectname = STEPSNAME;                  // identifies steps object in prim
        place_steps(objectname);
        gDirection = 0;                                 // not running
        set_escalator_anims();                          // set anims
        set_traffic_light(0);                           // set green and red lights                           
    }

    collision_start(integer total_number)               // using it forces a recheck
    {
        move_check();                                   // moved by editing?
    }
    
    on_rez(integer rezid) 
    {   llResetScript();
    }
    
    touch_start(integer num_detected)           // user interface
    {   
        key toucherID = llDetectedKey(0);       // who touched?
        integer touchedface =  llDetectedTouchFace(0);  // what did they touch?
        DEBUGPRINT("Touch on face " + (string)touchedface);
        move_check();                           // check if moved before dialog
        if (touchedface == ESTOPFACE)           // if emergency stop pushed
        {   estop_pushed(); return; }            // emergency stop pushed
        if (gLocked && (toucherID != llGetOwner())) { return; } // ignore touch if locked and not owner
        llListenRemove(gDialogHandle);          // remove any old listener
        gDialogHandle = 0;                      // no listener now
        list choices = [];                      // dialog box option
        if (gDirection == 1) { choices += "⬤ Up"; } else {choices += "Up"; }
        if (gDirection == -1) { choices += "⬤ Down"; } else {choices += "Down"; }
        if (gDirection == 0) { choices += "⬤ Stop"; } else {choices += "Stop"; }
        if (gLocked) { choices += "☑ Locked"; } else {choices += "☐ Locked"; }
        llDialog(toucherID, "Escalator control", choices, gDialogChannel);
        gDialogHandle = llListen(gDialogChannel, "", toucherID, ""); // wait for dialog
        gEstopPushed = FALSE;                   // we are now using the timer
        llSetTimerEvent(60.0);                  // just for dialog cleanup
    }
        
    listen(integer channel, string name, key id, string message)
    {
        if (channel == COMMANDCHANNEL)                                  // remote control listen
        {   command_listen(channel, name, id, message); }
        else if (channel == gDialogChannel)                             // dialog listen
        {   dialog_listen(channel, name, id, message); }
        else if (channel == gRezzedObjectID)
        {   DEBUGPRINT("Belt reply: " + message);                   // handshake with newly rezzed belt
            string reply = llJsonGetValue(message, ["reply"]);      // what message?
            if (reply == "rezzed")                                  // just rezzed?
            {
                llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,
                    ["command","REFPOS", "refpt", gTopref]));     // send position of reference point, global coords
                llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,
                    ["command","DIR", "direction", (string)gDirection]));
                llListenRemove(gReplyHandle);                       // rezzing is a one time thing
                gReplyHandle = 0;    
            }
        }
        else 
        {   llSay(DEBUG_CHANNEL, "Unexpected message on channel " + (string)channel + ": " + message); }
    }
    

    timer()
    {   // dialog listener cleanup
        llSetTimerEvent(0);
        if (gDialogHandle)                              // if had dialog up and ignored
        {   llListenRemove(gDialogHandle);
            gDialogHandle = 0;
        }
        if (gEstopPushed)                               // restart after E-stop
        {   gEstopPushed = FALSE;                       // clear E-stop
            set_escalator_state(gLastDirection);        // restart in same direction
        }
    }
    
    changed(integer change)
    {   if (change & CHANGED_SCALE)                     // can't do that
        {   llSay(0,"Cannot change size of this object.");
            llSetScale(gScale);                         // resizing this would break it
        }
        if (change & CHANGED_OWNER) { setup_command_listen(); }    // reset who we listen to on ownership change
    }
}
