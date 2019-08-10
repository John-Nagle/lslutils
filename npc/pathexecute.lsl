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
//  pathexeinit -- set up path execute parameters
//
pathexeinit(float slowspeed, float fastspeed, float turnrate)
{
    //  ***MORE***
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
//  pathexeextrapoints -- add points to path on long segments.
//
//  This is to allow speed and direction changes.
//
list pathexeextrapoints(list pts)
{
    //  ***MORE***
    return(pts);
}

//
//  pathexebuildkfm  -- build keyframe motion list from points
//
list pathexebuildkfm(vector startpos, rotation startrot, list pts)
{
    return([]);     // ***MORE***
}

//
//  pathexecalckfm -- calc the keyframe parameters for one point
//
list pathexecakckfm(vector pos, rotation rot, vector pprev, vector p0, vector p1, vector p2)
{
    return([]);     // ***MORE***
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
