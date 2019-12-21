//
//  pathavatartrackcall.lsl  
//  
//  Service task for pathfinding system users.
//  Reports nearby avatars and keeps track of which ones 
//  have been served.
//
//  Caller side include
//
//  Split from patrollerdemo for space reasons.
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#ifndef PATHAVATARTRACKCALLLSL
#define PATHAVATARTRACKCALLLSL

#include "debugmsg.lsl"

//
//  Link message types for avatar track.
//
#define PATHAVATARTRACKREQUEST  301             // we send this
#define PATHAVATARTRACKREPLY    302             // we receive this

//
//  Reply to msg from avatar tracking.
//
//  We ship the debug message level across so the tracking task knows what to print.
//
#define pathavatartrackreply(id, action) \
    llMessageLinked(LINK_THIS, PATHAVATARTRACKREPLY, llList2Json(JSON_OBJECT,["reply","trackavi","id",(id),"action",(action)]),"") 
    
//  Request message format is JSON with message num PATHAVATARTRACKREQUEST, format:
//
//  {"request":"trackavi", "id": KEY }
//
//  The using program has to parse that.
//
#endif // PATHAVATARTRACKCALLLSL

