//
//
//  simplekfmmove.lsl -- simple movement for an animesh character.
//
//  This is completely blind - no reaction to environment.
//  
//
//  Animats
//  January, 2013
//
//  License: GPLv3
//
//
//  The path.
//
//  [point, point, wait time, point, wait time...]
//  There can be multiple points between waits.
//
list PATHPOINTS = [<100,100,27>,10.0,<100,120,27>,10.0,<120,120,27>,10.0,<120,100,27>,10.0];    // visit these points




//
//  Globals
//
integer gPathpointIndex = 0;                                // current position in list
float gWaitTime;                                            // time to wait after move
float gPathMoveMaxTurnspeed = 1.57;                         // (radians/sec) turn rate limit 
float gPathMoveMaxSpeed = 2.5;                              // (m/sec) movement speed

//
//  getnextmovelist  -- get next list of points after current position in list
//
list getnextmovelist()
{
    list pts;
    integer i;
    integer waittime = 10;                          // default wait time
    integer pointslength = llGetListLength(PATHPOINTS); // length of points array
    for (i=gPathpointIndex; i<pointslength; i++)
    {   integer itemtype = llGetListEntryType(PATHPOINTS,i);   // type of list entry
        if (itemtype == TYPE_VECTOR)                // if element is vector
        {   pts += llList2Vector(PATHPOINTS,i);   } // add this point
        else if (itemtype == TYPE_FLOAT)            // this is the wait time
        {   pts += llList2Float(PATHPOINTS,i);      // get wait time at end of move 
            gPathpointIndex = (i+1) % pointslength; // where to start next time
            return(pts);                            // return result 
        } else {
            llSay(DEBUG_CHANNEL,"Path points list is not in right format: " + llDumpList2String(PATHPOINTS,","));
            return([]);                             // fails 
        }     
    }
    llSay(DEBUG_CHANNEL,"Path points list needs a delay time at the end: " + llDumpList2String(PATHPOINTS,","));
    return([]); 
}


//
//  pathmovemovementend -- movement has finished, feed in next section if any
//
pathmovemovementend()
{   
    if (gWaitTime <= 0.0)                                       // if no wait requested
    {   advance(); }                                            // do next set of points
    else
    {   llSetTimerEvent(gWaitTime);                             // wait for dwell time
        //  ***Switch to standing anim here***
    }
}





//
//  advance -- advance to next point
//
advance()
{   list pts = getnextmovelist();                               // next move to do
    llOwnerSay("Advance: " + llDumpList2String(pts,","));        // ***TEMP***
    if (pts == []) { return; }                                  // no points
    gWaitTime = llList2Float(pts,-1);                           // get last element of list which is wait time
    pts = llDeleteSubList(pts,-1,-1);                              // drop last element, leaving only points
    pts = llGetPos() + pts;                                     // start at current position
    list kfmmoves = pathexebuildkfm(llGetPos(), llGetRot(), pts);   // build list of commands to do
    llOwnerSay("KFM moves: " + llDumpList2String(kfmmoves,","));    // ***TEMP***
    if (kfmmoves != [])                             // if something to do (if only one point stored, nothing happens)
    {   llSetKeyframedMotion(kfmmoves, [KFM_MODE, KFM_FORWARD]);            // begin motion
        //  ***Switch to walking anim here***
    }

}
//
//  RotFromXAxis -- vector to rotation, XY plane, from X axis.
//
rotation RotFromXAxis(vector dv)
{   return(llAxes2Rot(llVecNorm(dv),<0,0,1>%llVecNorm(dv),<0,0,1>)); }
//
//  NormRot -- normalize a quaternion
//
rotation NormRot(rotation Q)
{    
    float MagQ = llSqrt(Q.x*Q.x + Q.y*Q.y +Q.z*Q.z + Q.s*Q.s);    
    Q.x = Q.x/MagQ;    
    Q.y = Q.y/MagQ;    
    Q.z = Q.z/MagQ;    
    Q.s = Q.s/MagQ;
    return Q;
}
//
//  pathexebuildkfm  -- build keyframe motion list from points
//
list pathexebuildkfm(vector startpos, rotation startrot, list pts)
{
    list kfmdata = [];                          // [pos, rot, time ... ]
    integer i;
    integer length = llGetListLength(pts);
    vector pos = startpos;
    ////gPathMoveLastdest = llList2Vector(pts,-1);  // last point. We should end up here.
    rotation rot = startrot;
    //  Vectors off the end are ZERO_VECTOR. Code relies on this.
    for (i=1; i<length; i++)                    // skip 1, we are already there.
    {   vector pprev = llList2Vector(pts,i-1);  // previous point
        vector p0 = llList2Vector(pts,i);       // point we are going to
        vector p1 = llList2Vector(pts,i+1);
        kfmdata += pathexecalckfm(pos, rot, pprev, p0, p1);
        pos += llList2Vector(kfmdata,-3);       // update pos in world coords
        rot *= llList2Rot(kfmdata,-2);          // update rot in world coords      
        pprev = p0;
    }
    return(kfmdata);                            // list ready for KFM
}

//
//  pathexecalckfm -- calc the keyframe parameters for one point
//
//  We are moving from pprev to p0.
//
list pathexecalckfm(vector pos, rotation rot, vector pprev, vector p0, vector p1)
{
    vector rp = p0 - pos;                       // p0 in relative coords - advances us to p0
    ////rp.z += gPathHeight * 0.5;              // add half-height, because path is at ground level
    //  Rotation is to the average direction of the previous and next sections in the XY plane.
    vector invec = p0-pprev;                    // incoming direction
    vector outvec = p1-p0;                      // outgoing direction
    float inveclen = llVecMag(invec);           // distance of this move
    vector invecnorm = llVecNorm(<invec.x, invec.y, 0>);
    vector outvecnorm = llVecNorm(<outvec.x,outvec.y,0>);
    if (p1 == ZERO_VECTOR) { outvecnorm = invecnorm; } // last section, no turn
    vector dir = llVecNorm(invecnorm+outvecnorm);// next direction
    rotation rr = RotFromXAxis(dir) / rot;      // relative rotation
    rr = NormRot(rr);                           // why is this necessary?
    //  Time computation. Speed is limited by rotation rate.
    float angle = llFabs(llAngleBetween(ZERO_ROTATION, rr));    // how much rotation is this?
    float rsecs = angle / gPathMoveMaxTurnspeed; // minimum time for this move per rotation limit
    float rt = inveclen / gPathMoveMaxSpeed;     // minimum time for this move per speed limit
    if (rsecs > rt) { rt = rsecs; }             // choose longer time
    if (rt < 0.15) { rt = 0.15; }               // minimum time for KFM step
    return([rp, rr, rt]);                       // [rel pos, rel rot, rel time]
} 




//
//  The main program of the move task.
//
default
{
    state_entry()
    {
        advance();
    }

    on_rez(integer rezparam) 
    {   llResetScript(); }

    
    timer()
    {   llSetTimerEvent(0.0);                   // no timing while moving
        advance();                              // do next step
    }
    
    moving_end()
    {   pathmovemovementend(); }   
    
}
