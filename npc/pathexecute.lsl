//
//  pathexecute.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  This is where the path components are assembled and keyframe
//  animation lists are created and executed.
//
//  Speed and turning rate are set here, too
//
//  Animats
//  June, 2019
//
#ifndef PATHEXECUTE                                         // include guard, like C/C++
#define PATHEXECUTE
#include "npc/assert.lsl"                                   // assert
#include "npc/mazedefs.lsl"

//
//  Globals
//
list gInputSegments = [];                                   // strided list - segment number, segment points
float gMaxTurnRate = 0.2;                                   // (radians/sec) max turn rate
float gMaxSpeed = 2.0;                                      // (meters/sec) max speed



//
//  pathexeinit -- set up path execute parameters
//
pathexeinit(float speed, float turnrate)
{
    pathexestop();                                          // stop any operation in progress
    gMaxSpeed = speed;
    gMaxTurnRate = turnrate;
    
}

//
//  pathexecutedeliver -- incoming path segment
//
//  Starts motion if necessary
//
pathexedeliver(list pts, integer pathid, integer segmentid)
{
    //  ***MORE***
}

//
//  pathexeextrapoints -- add extra points to path on long segments.
//
//  This is to allow smooth speed and direction changes.
//
list pathexeextrapoints(list pts, float distfromend)
{
    list newpts = [];
    integer i;
    integer length = llGetListLength(pts);   
    vector p0 = llList2Vector(pts,0);
    newpts += [p0];                                 // always the initial point     
    for (i=1; i<length; i++)
    {   vector p1 = llList2Vector(pts,i);           // this pt
        //  Do we need to add any intermediate points?
        vector dv = p1-p0;
        vector dir = llVecNorm(dv);
        float dlen = llVecMag(dv);                          
        if (dlen >= 3*distfromend)                  // long enough to need 2 intermediate points
        {   newpts += [p0+dir*distfromend, p1-dir*distfromend]; }
        else if (dlen >= 2*distfromend)
        {   newpts += [(p0+p1)*0.5); }              // add the midpoint 
        newpts += [p1];
        p0 = p1;                                    // for next cycle
    }
    return(newpts);
}

//
//  pathexebuildkfm  -- build keyframe motion list from points
//
list pathexebuildkfm(vector startpos, rotation startrot, list pts)
{
    list kfmdata = [];      // [pos, rot, time ... ]
    integer i;
    integer length = llGetListLength(pts);
    vector pos = startpos;
    vector rot = startrot;
    //  Vectors off the end are ZERO_VECTOR. Code relies on this.
    for (i=1; i<length; i++)                    // skip 1, we are already there.
    {   vector pprev = llList2Vector(pts,i-1);  // previous point
        vector p0 = llList2Vector(pts,i);       // point we are going to
        vector p1 = llList2Vector(pts,i+1);
        kfmdata += pathexecalckfm(pos, rot, pprev, p0, p1);
        pos += llList2Rotatation(kfmdata,-3);   // update pos in world coords
        rot *= llList2Rotation(kfmdata,-2);     // update rot in world coords      
        pprev = p0;
    }
    return(kfmdata);                            // list ready for KFM
}

//
//  pathexecalckfm -- calc the keyframe parameters for one point
//
//  ***NEED TO PREVENT ROTATION ERROR ACCUMULATION***
//
list pathexecakckfm(vector pos, rotation rot, vector pprev, vector p0, vector p1)
{
    vector rp = p0 - pos;                       // p0 in relative coords - advances us to p0
    //  Rotation is to the average direction of the previous and next sections in the XY plane.
    vector invec = pprev-p0;                    // incoming direction
    vector outvec = p1-p0;                      // outgoing direction
    float outveclen = llVecMag(outvec);         // distance of this move
    vector invecnorm = llVecNorm(<invec.x, invec.y, 0>);
    vector outvecnorm = llVecNorm(<outvec.x,outvec.y,0>);
    if (p1 == ZERO_VECTOR) { outvecnorm = invecnorm; } // last section, no turn
    vector dir = llVecNorm(invecnorm+outvecnorm);// next direction
    rotation rr = llRotBetween(invecnorm, dir); // relative rotation
    //  Time computation. Speed is limited by rotation rate.
    float angle = llFAbs(llAngleBetween(ZERO_ROTATION, rr));    // how much rotation is this?
    float rsecs = angle / gMaxTurnRate;         // minimum time for this move per rotation limit
    float rt = outveclen / gMaxSpeed;           // minimum time for this move per speed limit
    if (rsecs > rt) { rt = rsecs; }             // choose longer time
    return([rp, rr, rt]);                       // [rel pos, rel rot, rel time]
} 

//
//  pathexemovementend -- movement has finished, feed in next section if any
//
pathexemovementend()
{
}

//
//  pathexetimer  -- timer event, check progress
//
pathexetimer()
{
}

//
//  pathexestop -- trouble, stop and abort keyframe motion
//
pathexestop(integer status)
{
}

#endif // PATHEXECUTE
