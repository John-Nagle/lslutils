//
//  pathmovecall.lsl  -- caller side interface to pathscan
//
#define LINKMSGSCANREQUEST  1010                            // message types
#define LINKMSGSCANREPLY    1011
//
//  Path move start -- sends link message to move task
//
//  This is a macro to avoid an extra recopy of "pts", which can be large.
//  The "execute" task is tight on space.
//
//  "move" used to be called "scan", before that task picked up the KFM work.
//
#define pathscanstart(pts, width, height, target, speed, turnspeed, pathid, msglev) {  \
    llMessageLinked(LINK_THIS, LINKMSGSCANREQUEST, \
    llList2Json(JSON_OBJECT,["request","startscan","width", (width), "height", (height), "target", (target),\
    "speed", (speed), "turnspeed", turnspeed,\
    "pathid",(pathid), "msglev",(msglev),\
    "points", llList2Json(JSON_ARRAY,(pts))]),""); }  
//
//  Path scan stop -- sends link message to scan task
//
#define pathscanstop() {  \
    llMessageLinked(LINK_THIS, LINKMSGSCANREQUEST, \
    llList2Json(JSON_OBJECT,["request","stopscan"]),""); }  

