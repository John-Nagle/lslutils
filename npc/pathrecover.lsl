//
//  pathrecover.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  Gets the NPC back to a safe position.
//
//  Communicates with pathmove.lsl, which got too big.
//
//  Animats
//  October, 2019
//
//  License: GPLv3
//
///
#include "npc/assert.lsl"                                   // assert
#include "npc/patherrors.lsl"
#include "npc/mazedefs.lsl"
#include "npc/pathbuildutils.lsl"
#include "npc/pathmovecall.lsl"
//
//  Constants
//

//
//  Globals
//
integer gPathrecoverId = 0;                                    // current path ID

//
//  pathcheckforwalkable  -- is there a walkable below here?
//
//  Returns 0 or error status
//
//
integer pathcheckforwalkable()
{   vector pos = llGetPos();                                // we are here
    vector fullheight = <0,0,gPathMoveHeight>;              // add this for casts from middle of character
    vector halfheight = fullheight*0.5;
    vector p = pos-halfheight;                              // position on ground
    vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
    if (obstacleraycastvert(p+fullheight,p-mazedepthmargin) >= 0)  
    {   return(0); }                                        // no problem   
    //  Trouble, there is no walkable here
    return(PATHEXEWALKABLEFAIL);                            // fail for now, may recover later
}

//
//  pathrecoverwalkable  -- get back onto walkable surface if possible
//
//  Returns 0 or error status
//
integer pathrecoverwalkable(list pts)
{   vector pos = llGetPos();                                // we are here
    //  Trouble, there is no walkable here
    //  Attempt recovery. Try to find a previous good location that's currently open and move there.
    integer i = llGetListLength(pts);                       // for stored good points, most recent first
    pathMsg(PATH_MSG_WARN,"No walkable below after move to " + (string)pos + ". Recovery points available: " + (string)i);
    while (i-- > 1)                                         // for newest (len-1) to oldest (0)
    {   vector recoverpos = llList2Vector(pts,i);           // try to recover to here
        vector prevrecoverpos = llList2Vector(pts,i-1);
        vector halfheight = <0,0,gPathHeight*0.5>;
        if (pathcheckcelloccupied(prevrecoverpos, recoverpos, TRUE, FALSE) >= 0.0)
        {   
            llSleep(0.5);                                   // allow time for stop to take effect
            llSetPos(recoverpos + halfheight);              // forced move to previous good position
            llSleep(0.5);                                   // give time to settle
            pathMsg(PATH_MSG_WARN,"Recovered by moving to " + (string) recoverpos);
            return(PATHEXEWALKABLEFIXED);
        }
    }
    pathMsg(PATH_MSG_ERROR,"Unable to recover from lack of walkable below " + (string)pos + " by recovering to any of " + llDumpList2String(pts,",")); 
    return(PATHEXEWALKABLEFAIL);
}


//
//  pathrecoverrequest  -- JSON from path move
//
//  We get a list of possible recovery points.
//
pathrecoverrequest(string jsn, key hitobj) 
{   pathMsg(PATH_MSG_WARN,"Path recover request: " + jsn);
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type  
    if (requesttype == "recover")                    // recover to known good position, requested by pathprep
    {   //  Go back to some previous good point.
        integer pathid = (integer)llJsonGetValue(jsn, ["pathid"]);
        list ptsstr = llJson2List(llJsonGetValue(jsn, ["recoverpoints"])); // points, as list of strings
        list pts = [];                                  // points as list of vectors
        integer i;
        for (i=0; i<llGetListLength(ptsstr); i++) { pts += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors
        jsn = "";                                       // release memory
        integer status = pathrecoverwalkable(pts);      // force to a walkable position
        pathrecoverreply(pathid, status, hitobj);
        ////pathdonereply(status, hitobj, pathid);          // report move completion
    } else {
        pathMsg(PATH_MSG_ERROR,"Bad msg: " + jsn);
    }
}

//
//  The main program of the recover task.
//
default
{
    state_entry()
    {   pathinitutils();                                        // init library
    }


    link_message(integer status, integer num, string jsn, key id )
    {   if (num == LINKMSGRECOVERREQUEST)               // recovery request
        {   pathrecoverrequest(jsn,id); 
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                      // initialize params
    }
    
}
