//
//  Simple wind vane
//
integer ENABLE_SOUND = TRUE;                            // turn on sound
float TIMER_INTERVAL = 5.0;                             // not too often
float SOUND_MIN_MOVE = 1.0;                             // minimum move for sound (deg)
list SQUEAKS = ["squeak1", "squeak2", "squeak3"];       // squeak sounds to be chosen randomly
string PARENTNAME = "vane";                             // name of parent prim must contain this
float WIND_SPEED_SCALE = 3.0;                           // multiply wind speed by this to get volume
rotation VANE_ROT;                                      // vane object rotation applied after turning vane
float gPrevAngle = 0.0;                                 // previous angle
integer gPrevSemiRandom = 0;                            // previous semi-random value
integer gParentPrim = -1;                               // parent prim to which we are attached

float abs(float x)
{   if (x < 0) { return(-x); }
    return(x);
}

//  Find prim by name, approximately. 
//  Ignore self.
//  Breaks ties based on distance.
//  If only SL had hierarchies, like every other 3D system.
integer find_prim_by_name(string namestart)
{   integer foundprim = -1;                             // prim found or -1
    float founddist = 1000000.0;                        // big dist
    namestart = llToLower(namestart);                   // case insensitive comapre
    integer primcnt = llGetNumberOfPrims();             // limit
    integer selfprim = llGetLinkNumber();               // me
    vector selfpos = llGetPos();                        // my pos
    integer i;
    for (i=1; i<primcnt; i++)
    {   string primname = llToLower(llGetLinkName(i));
        if ((i != selfprim) && (llSubStringIndex(primname, namestart) >= 0))  // if find
        {   vector otherpos = llList2Vector(llGetLinkPrimitiveParams(i,[PRIM_POSITION]),0);
            float dist = llVecMag(selfpos-otherpos);    // dist from me
            if (dist < founddist) 
            {   founddist = dist;
                foundprim = i;
                ////llOwnerSay("Parent prim (#" + (string)i + "): " + primname);
            }
        }
    }     
    return(foundprim);
}

find_parent() 
{   gParentPrim = find_prim_by_name(PARENTNAME);    
    if (gParentPrim < 0)
    {   llOwnerSay("Can't find parent prim. Vane is broken."); }
}

//  semi-random -- random number from 0..lim-1, but never the same as last time.
integer semi_random(integer lim)                        // drum machines do this
{   integer n = (integer)llFrand(lim);                  // random number from 0..lim-1
    if (n == gPrevSemiRandom) { n = (n+1) % lim; }      // never same as last time
    gPrevSemiRandom = n;                                // save to prevent dup
    return(n);
}

play_sound(float volume)
{   string sound = llList2String(SQUEAKS, semi_random(llGetListLength(SQUEAKS)));  // pick a random squeak
    llPlaySound(sound, volume);
}

default
{
    state_entry()
    {   find_parent();                                  // find parent prim
        VANE_ROT = llEuler2Rot(<0,90,0>*DEG_TO_RAD);    // rotation to apply before turning
        llSetTimerEvent(TIMER_INTERVAL);
    }

    timer()
    {   if (gParentPrim < 0)
        {   llOwnerSay("Vane broken. Will not move.");
            llSetTimerEvent(0.0);
            return;
        }
        rotation rootrot = llList2Rot(llGetLinkPrimitiveParams(gParentPrim,[PRIM_ROTATION]),0); // orientation of base
        vector rootrotang = llRot2Euler(rootrot);
        rootrot = llEuler2Rot(<0,0,rootrotang.z>);      // we just want the Z part
        ////llOwnerSay("Root rot: " + (string)(llRot2Euler(rootrot)*RAD_TO_DEG));
        vector wind = llWind(ZERO_VECTOR);                             // get wind
        float winddir = llAtan2(wind.y, wind.x);
        float windspeed = llVecMag(wind);               // wind speed
        
        ////llOwnerSay("Wind: " + (string) wind + "  " + (string) (winddir*RAD_TO_DEG) + " deg.");
        winddir = -winddir - PI/2;                        // because big end should be away from wind
        rotation rot = llEuler2Rot( <0,0,winddir>);     // Convert to a rotation
        if (ENABLE_SOUND && (abs(winddir - gPrevAngle) > SOUND_MIN_MOVE*DEG_TO_RAD))
        {   play_sound(windspeed*WIND_SPEED_SCALE + 0.5);  // play squeak if moved much
            gPrevAngle = winddir;
        }
        llSetLocalRot((VANE_ROT*rot)/rootrot);          // Point weather vane
        llSetTimerEvent(0.5+llFrand(TIMER_INTERVAL));   // randomize timer so it is not repetitive
    }
    
    changed(integer changes)
    {   if (changes & CHANGED_LINK)
        {   find_parent();                              // find link id of parent prim
        }
    }
    
    on_rez(integer param)
    {   find_parent(); }
}

