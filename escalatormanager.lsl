//
//  Rez and size manager
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
//  SINGLE SIZE VERSION for escalator - no resizing yet
//
//  Positions captured from the edit window of a successful assembly.
//
//// CHILDPOS = <233.19861, 50.00000, 37.93129>;
//// PARENTPOS = <233.00000, 50.00000, 38.15352>;
//// CHILDOFFSET = <0.19861, 0.0, -0.222> // need to swap axes
integer STEPSANIMLINK = 2;
integer STEPSANIMFACE = ALL_SIDES;
integer RAILANIMLINK = 1;
integer RAILANIMFACE = 3;
integer TRAFFICLIGHTLINK = 1;
integer TRAFFICLIGHTFACE = 4;

float STEPSANIMRATE = -1.5;                         // steps animation speed
float RAILANIMRATE = 0.9;                           // railing animation speed, matched to escalator
float TRAFFICLIGHTGLOW = 0.2;                       // glow brightness for red and green lights




vector CHILDOFFSET = <0.0, 0.19861, -0.222>;    // root of child rel to self

string OBJECTNAME = "Steps";
vector INITIALROTANG = <0,0,0>;                 // initial rotation angle, radians

integer MAXINT = 16777216;              // 2^24
integer MINCHAN = 100000;               // for range of allowed unique channel numbers

string SOUNDNAME="escalator1";          // escalator sound effect
float VOLUME=0.20;                      // audio level


//  Global variables
integer gRezzedObjectID = 0;            // channel for deleting rezzed object
vector gPrevPos = ZERO_VECTOR;
rotation gPrevRot = ZERO_ROTATION;
integer gDirection = 0;                 // direction of motion (-1, 0, or +1)
integer gLocked = FALSE;                // owner only
integer gDialogChannel;                 // for talking to user
integer gListenHandle = 0;              // listening for dialog response

//
//  vector_mult - multiply vector by vector elementwise
//
vector vector_mult(vector v0, vector v1)
{   return (<v0.x*v1.x, v0.y*v1.y, v0.z*v1.z>); }// there should be a builtin for this

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
//
//  place_object -- rez and place object
//
place_object(string name, vector offset)
{
    //  Delete old object
    if (gRezzedObjectID) 
    {   
        llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,["command","DIE"]));
    }
    //  Rez new object
    rotation rot = llGetRot();
    rot = rot * llEuler2Rot(INITIALROTANG*DEG_TO_RAD); // rotate before rezzing
    offset = offset * rot;              // to world coords
    vector pos = llGetPos() + offset;
    //  unique integer channel number in range frand can handle
    integer randomid = - (integer)(llFrand(MAXINT-MINCHAN) + MINCHAN); 
    llRezObject(name, pos, <0,0,0>, rot, randomid); // create the object
    gRezzedObjectID = randomid;
    gPrevPos = llGetPos();                  // previous location
    gPrevRot = llGetRot();
    ////llOwnerSay("Placed " + name + " id: " + (string) randomid); // ***TEMP***
    llSleep(3.0);                           // allow object time to rez
    llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,
        ["command","DIR", "direction", (string)gDirection]));
}

move_check() 
{   //  Check whether escalator moved. via edit.
    if ((llVecMag(llGetPos()-gPrevPos) > 0.001)     // if we moved, delete and replace object
    || (llFabs(llAngleBetween(llGetRot(),gPrevRot)) > 0.0005))
    {
        vector objectoffset = CHILDOFFSET;     // offset for new object
        place_object(OBJECTNAME, objectoffset);
    }
}

default
{
    state_entry()
    {   gDialogChannel = - (integer)(llFrand(MAXINT-MINCHAN) + MINCHAN);  // for dialog msgs
        vector objectoffset = CHILDOFFSET;              // offset for new object
        place_object(OBJECTNAME, objectoffset);
        gDirection = 0;                                 // not running
        llSetLinkTextureAnim(STEPSANIMLINK, 0, STEPSANIMFACE, 1, 1, 1.0, -1.0, 0); // stop previous animation
        llSetLinkTextureAnim(RAILANIMLINK, 0, RAILANIMFACE, 1, 1, 1.0, -1.0, 0); // stop previous animation
        llStopSound();                                  // silence
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
        move_check();                           // check if moved before dialog
        if (gLocked && (toucherID != llGetOwner())) { return; } // ignore touch if locked and not owner
        llListenRemove(gListenHandle);          // remove any old listener
        list choices = [];                      // dialog box option
        if (gDirection == 1) { choices += "⬤ Up"; } else {choices += "Up"; }
        if (gDirection == -1) { choices += "⬤ Down"; } else {choices += "Down"; }
        if (gDirection == 0) { choices += "⬤ Stop"; } else {choices += "Stop"; }
        if (gLocked) { choices += "☑ Locked"; } else {choices += "☐ Locked"; }
        llDialog(toucherID, "Escalator control", choices, gDialogChannel);
        gListenHandle = llListen(gDialogChannel, "", toucherID, ""); // wait for dialog
        llSetTimerEvent(60.0);                  // just for cleanup
    }
        
    listen(integer channel, string name, key id, string message)
    {
        llListenRemove(gListenHandle);              // turn off the listener
        ////llOwnerSay("Dialog string: " + name + " message: " + message);
        //  Process response
        integer direction = gDirection;
        if (llSubStringIndex(message, "Up") >= 0) { direction = 1; }
        else if (llSubStringIndex(message, "Down") >= 0) { direction = -1; }
        else if (llSubStringIndex(message, "Stop") >= 0) { direction = 0; }
        else if (llSubStringIndex(message, "Lock") >= 0) { gLocked = !gLocked; } 
        if (direction == gDirection) { return; }    // no change in dir
        gDirection = direction;                     // change direction
        ////llOwnerSay("Step direction: " + (string)gDirection);   
        llShout(gRezzedObjectID, llList2Json(JSON_OBJECT,
            ["command","DIR", "direction", (string)gDirection]));
        //  Animation of flat steps which are part of the escalator base and do not move.
        if (gDirection != 0)
        {   llSetLinkTextureAnim(RAILANIMLINK, ANIM_ON|LOOP|SMOOTH, 
                RAILANIMFACE, 1, 1, 1.0, -1.0, RAILANIMRATE*gDirection);
            llSetLinkTextureAnim(STEPSANIMLINK, ANIM_ON|LOOP|SMOOTH, 
                STEPSANIMFACE, 1, 1, 1.0, -1.0, STEPSANIMRATE*gDirection); // start animation
            llLoopSound(SOUNDNAME, VOLUME);                 // escalator sound 
        }
        else
        {   
            llSetLinkTextureAnim(RAILANIMLINK, 0, RAILANIMFACE, 1, 1, 1.0, -1.0, 0); // stop previous animation
            llSetLinkTextureAnim(STEPSANIMLINK, 0, STEPSANIMFACE, 1, 1, 1.0, -1.0, 0); // stop previous animation
            llStopSound();                                  // silence
        }
        set_traffic_light(gDirection);                      // set green and red lights                           
    }

    timer()
    {   // listener cleanup
        llSetTimerEvent(0); 
        llListenRemove(gListenHandle);
    }
}

