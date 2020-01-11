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
float GLOWON = 0.50;                    // glow value when on
float LOOKRANGE = 50.0;                 // (m) range to ray cast
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
    ////llOwnerSay("Mouselook touch");          // ***TEMP***
    ///vector pos = llGetPos();                        // position of avatar
    vector pos = llGetCameraPos();                  // position of camera
    rotation rot = llGetRot();                      // direction of mouselook
    vector castpos = pos + <LOOKRANGE,0,0>*rot;     // far end of ray cast
    list castresult = llCastRay(pos, castpos, [RC_DATA_FLAGS,RC_GET_NORMAL|RC_GET_ROOT_KEY,RC_DETECT_PHANTOM,TRUE,RC_MAX_HITS,10]); // get target info
    ////llOwnerSay("Cast result to " + (string)castpos + ": " + llDumpList2String(castresult,",")); // ***TEMP***
    integer status = llList2Integer(castresult,-1);
    if (status <= 0) { return; }                    // no hit or error
    integer i;
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
            {   pftypename = "Physics object"; }
            else
            {   pftypename = "Movable obstacle"; }
        }
        llSay(0,hitname + " (" + pftypename + ")");     // result 
        integer stopix = llListFindList(PFSTOPS,[pftype]);// is this a type to stop for?
        if (stopix >= 0) { return; }                    // yes, done
    }  
}
//
//  Default state - inactive.
//
//  Click on amulet to activate.
default
{
    state_entry()
    {   //  Inactivate amulet
        llSetPrimitiveParams([PRIM_GLOW,ALL_SIDES,0.0,
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
        llSetPrimitiveParams([PRIM_GLOW,ALL_SIDES,GLOWON,
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

}

