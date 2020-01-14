//
//  Pathfinding probe amulet
//
//  Wear, touch to turn on, then click on things in mouselook.
//  Displays useful info about pathfinding properties of objects.
//
//  Animats
//  January, 2020
//
//  License: GPLv3.
//
//  Constants
//
float GLOWON = 0.25;                    // glow value when on
float LOOKRANGE = 50.0;                 // (m) range to ray cast
float MAXWALKABLEDEG = 64.0;            // (degrees) max slope for walkable
integer AMULETLINK = 2;                 // which link should glow 
//
//  Names of pathfinding types, for display.
//
list PFNAMES = [
"Movable obstacles, movable phantoms, physical, or volumedetect object",
"Avatar",
"Pathfinding character",
"Walkable object",
"Static obstacle",
"Material volume",
"Exclusion volume"];
//
//  List of pathfinding types which stop the search.
//  These are the ones that contribute to the static navmesh.
//
list PFSTOPS = [OPT_WALKABLE, OPT_STATIC_OBSTACLE,OPT_EXCLUSION_VOLUME];

//
//  mouseloooktouch -- clicked on something in mouselook
//
mouselooktouch()
{
    vector pos = llGetCameraPos();                  // position of camera
    rotation rot = llGetRot();                      // direction of mouselook
    vector castpos = pos + <LOOKRANGE,0,0>*rot;     // far end of ray cast
    list castresult = llCastRay(pos, castpos, [RC_DATA_FLAGS,RC_GET_NORMAL|RC_GET_ROOT_KEY,RC_DETECT_PHANTOM,TRUE,RC_MAX_HITS,10]); // get target info
    ////llOwnerSay("Cast result to " + (string)castpos + ": " + llDumpList2String(castresult,",")); // ***TEMP***
    integer status = llList2Integer(castresult,-1);
    if (status <= 0) { return; }                    // no hit or error
    integer i;
    integer covered = FALSE;                        // something is covering the walkable
    integer walkable = FALSE;                       // found walkable
    for (i=0; i<3*status; i+=3)                     // advance through strided list
    {
        key hitid = llList2Key(castresult,i+0);           // key of hit
        vector hitpos = llList2Vector(castresult,i+1);    // pos of hit
        vector hitnormal = llList2Vector(castresult,i+2); // normal at hit
        string hitname = "Ground";
        integer pftype = OPT_OTHER;
        integer physics = FALSE;
        integer phantom = FALSE;
        if (hitid != NULL_KEY)                          // if not ground
        {   hitname = llKey2Name(hitid);                // name of hit
            list props = llGetObjectDetails(hitid,[OBJECT_NAME,OBJECT_PATHFINDING_TYPE,OBJECT_PHYSICS,OBJECT_PHANTOM]); // obj info
            if (props == [])
            {   llSay(0,"Target object not found."); return; }
            hitname = llList2String(props,0);           // name
            pftype = llList2Integer(props,1);           // pathfinding type
            physics = llList2Integer(props,2);          // true if physical
            phantom = llList2Integer(props,3);          // true if phantom
        } else {                                        // is ground
            pftype = OPT_WALKABLE;                      // ground is walkable
        }
        string pftypename = "Other";
        if (pftype >= 0) { pftypename = llList2String(PFNAMES,pftype); } // name of type
        if (pftype == OPT_LEGACY_LINKSET)               // need more analysis for this pathfinding type
        {   if (phantom) 
            {   pftypename = "Phantom"; }
            else if (physics)
            {   pftypename = "Physics object"; covered = TRUE; }
            else
            {   pftypename = "Movable obstacle"; covered = TRUE; }
        } else if (pftype == OPT_WALKABLE)              // for walkables, have to check slope
        {
            float slopedeg = RAD_TO_DEG*llAcos(hitnormal*<0,0,1>);  // slope of walkable
            if (slopedeg > MAXWALKABLEDEG)              // if too steep
            {   pftypename += ", but not here; " + (string)llCeil(slopedeg) + "° slope";
            } else {
                walkable = TRUE;
            }
        } else if (pftype == OPT_AVATAR || pftype == OPT_CHARACTER)
        {   covered = TRUE;                             // these can cover something
        }
        llSay(0,hitname + " (" + pftypename + ")");     // result 
        integer stopix = llListFindList(PFSTOPS,[pftype]);// is this a type to stop for?
        if (stopix >= 0)                                // if stops search
        {   if (covered) { walkable = FALSE; }       // if covered, not walkable
            if (walkable) 
            {   llSay(0,"CAN walk here."); }
            else
            {   llSay(0,"CANNOT walk here."); }
            return;
        }
    }
    llSay(0,"Out of range.");                       // object not in scan range
}
//
//  Default state - inactive.
//
//  Click on amulet to activate.
//
default
{
    state_entry()
    {   //  Inactivate amulet
        llSetLinkPrimitiveParams(AMULETLINK,[PRIM_GLOW,ALL_SIDES,0.0,
            PRIM_FULLBRIGHT,ALL_SIDES, FALSE]);
        llReleaseControls();     
    }

    touch_start(integer total_number)
    {   if (llGetOwner() != llDetectedKey(0)) { return; } // touched by another, ignore
        state active;
    }
}

//
//  Amulet active
//
state active
{
    state_entry()
    {
        llSetLinkPrimitiveParams(AMULETLINK,[PRIM_GLOW,ALL_SIDES,GLOWON,
            PRIM_FULLBRIGHT,ALL_SIDES, TRUE]); // glow
        llRequestPermissions(llGetOwner(),PERMISSION_TAKE_CONTROLS|PERMISSION_TRACK_CAMERA);        
    }
    
    run_time_permissions(integer perm)
    {   ////llOwnerSay("Got perms: " + (string)perm);
        if (perm & PERMISSION_TAKE_CONTROLS)
        {   llTakeControls(CONTROL_ML_LBUTTON,TRUE,FALSE); }
    }
    
    
    touch_start(integer total_number)
    {
        if (llGetOwner() != llDetectedKey(0)) { return; } // touched by another, ignore
        state default;
    }
    
    control( key id, integer level, integer edge )
    {
        ////llOwnerSay((string)level);
        if ((level & CONTROL_ML_LBUTTON) && (edge & CONTROL_ML_LBUTTON))
        {   mouselooktouch();               // mouselook touch event
        }
    }
    
    changed(integer change)
    {
        if (change & (CHANGED_REGION | CHANGED_TELEPORT))   // if changed region, often have to re-request permissions
        {
            llSetTimerEvent(1.0);           // ask in timer until we get them
        }
    }
    
    attach(key id)
    {   if (id == NULL_KEY) { return; }     // detach
        llSetTimerEvent(1.0);               // ask for perms in timer 
    }
    
    timer()
    {
        integer perms = llGetPermissions(); // what permissions do we have?
        if ((perms & (PERMISSION_TAKE_CONTROLS|PERMISSION_TRACK_CAMERA)) == (PERMISSION_TAKE_CONTROLS|PERMISSION_TRACK_CAMERA)) // if have needed perms
        {   llSetTimerEvent(0.0);           // no more asking
            ////llOwnerSay("Have perms, taking controls.");   // ***TEMP***
            llTakeControls(CONTROL_ML_LBUTTON,TRUE,FALSE);  // take controls when recovering permissions
            return;
        }
        llRequestPermissions(llGetOwner(),PERMISSION_TAKE_CONTROLS|PERMISSION_TRACK_CAMERA); // ask again
        ////llOwnerSay("Re-requesting permissions.");
    }
}

