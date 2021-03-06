//  
//  Unsit helper - more graceful unsits
//
//
//
//  Globals
//
integer gSitting = FALSE;                                   // true if sitting
vector  gLastSitPos;                                        // last pos seen sitting
float gAvatarHeight = 2.0;                                  // assumed avatar height, updated later
float gAvatarRadius = 0.5;                                  // assumed avatar radius of enclosing cylinder
list TESTPTS = [<0,1,0>, <1,0,0>, <0,-1,0>, <-1,0,0>];      // test around avatar for open space

//
//  teststandpos -- is this a good place to stand?
//
//  Returns < 0 if bad, z value of ground if good.
//  Input pos is at avatar midpoint Z.
//
float teststandpos(vector pos)
{
    vector start = pos + <0,0,gAvatarHeight>;                // from well above top of avatar
    vector end = pos - <0,0,gAvatarHeight>;                  // to half its length below ground
    list castresult = llCastRay(start, end, []);            // vertical cast ray test
    integer status = llList2Integer(castresult,-1);         // final entry in list is status
    if (status < 0) { return(-1.0); }                       // cast ray problem, fail this point
    if (status == 0) { return(0.0); }                       // no hits, open space.
    key hitkey = llList2Key(castresult,0);                  // what we hit
    vector hitpos = llList2Vector(castresult,1);            // where we hit
    vector disttohit = pos - hitpos;
    integer tooclose = disttohit.z < gAvatarHeight/2;        // too close
    ////llOwnerSay("hit " + llKey2Name(hitkey) + " below" + (string)start + " at " + (string)hitpos + " too close: " + (string)tooclose);  // ***TEMP***

    if  (tooclose)                                          // if ground is too close
    {   return(-1.0); }
    return(hitpos.z);                                       // loc of ground  
}
//
//  testaroundstandpos - test for enough clearance for an avatar around pos
//
integer testaroundstandpos(vector pos)
{
    if (teststandpos(pos) < 0) { return(FALSE); }           // obstacle below pos
    integer j;
    for (j=0; j<llGetListLength(TESTPTS); j++)              // test around avatar placement
    {   vector testoffsetpos = pos + llList2Vector(TESTPTS,j)*gAvatarRadius;
        float z = teststandpos(testoffsetpos);              // point near desired avatar placement
        if (z < 0) { return(FALSE); }                       // fails       
    }
    return(TRUE);                                           // success, need to make boolean                
}
//
//  findstandpos -- find an unobstructed place to stand
//
vector findstandpos(vector pos)
{
    integer i;
    integer TRIES = 32;
    for (i=0; i<TRIES; i++)                                 // spiral outwards from straight ahead
    {
        float radius = i/8.0;                               // looking outward 
        float angle = (i % 8)*PI/4;                         // angle
        vector tryoffset = <llSin(angle),llCos(angle),0>*radius;   // offset, starting from straight ahead
        vector trypos = pos + tryoffset/llGetRot();         // pos to test
        if (tryoffset.x >= 0)                               // ***TEMP*** don't go behind chair
        {   if (testaroundstandpos(trypos))                     // success
            {   llOwnerSay("Found open space at " + (string) trypos + " offset " + (string)tryoffset + " angle " + (string)(angle*RAD_TO_DEG));
                return(trypos);
            }  
        }
    }
    return(ZERO_VECTOR);
}
//
//  stoodup -- avatar just stood up
//
stoodup(vector pos)
{   list bb = llGetBoundingBox(llGetOwner());               // bounding box of avatar
    vector mincorner = llList2Vector(bb,0);                 // corners of bounding box
    vector maxcorner = llList2Vector(bb,1);
    vector avatardims = maxcorner - mincorner;              // dimensions of avatar
    llOwnerSay("Avatar dimensions: " + (string)avatardims); // ***TEMP***
    gAvatarHeight = avatardims.z;
    float avatardiam = avatardims.x;
    if (avatardims.y > avatardims.x) { avatardiam = avatardims.y; } // this language needs built-in min
    gAvatarRadius = (avatardiam * 0.5)*1.1;               // avatar radius, with safety margin
    vector target = findstandpos(pos);
    if (target == ZERO_VECTOR) { return; }
    llMoveToTarget(target, 1.0);
    llSleep(1.0);
    llStopMoveToTarget();
   
}

default
{
    state_entry()
    {
        llSetTimerEvent(1.0);                               // always running timer - bad.
    }
        
    timer()
    {  
        {
            integer sitting = (llGetAgentInfo(llGetOwner()) & AGENT_SITTING) != 0;  // true if sitting now
            ////llOwnerSay("Sitting: " + (string)sitting);
            if (gSitting & !sitting)
            {   ////vector pos = llGetPos();
                vector pos = gLastSitPos;
                llOwnerSay("Stood up at " + (string)pos);
                stoodup(pos);
            }
            if (sitting) { gLastSitPos = llGetPos(); }      // record sit position before getting up
            gSitting = sitting;                             // update sitting status
        }   
    }
}

