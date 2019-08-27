//
//  pathscancall.lsl  -- caller side interface to pathscan
//
#define LINKMSGSCANREQUEST  1010                            // message types
#define LINKMSGSCANREPLY    1011
//
//  Path scan start -- sends link message to scan task
//
//  This is a macro to avoid an extra recopy of "pts", which can be large.
//  The "execute" task is tight on space.
//
#define pathscanstart(pts, width, height, pathid, msglev) {  \
    llMessageLinked(LINK_THIS, LINKMSGSCANREQUEST, \
    llList2Json(JSON_OBJECT,["request","startscan","width", (width), "height", (height), "pathid",(pathid), "msglev",(msglev),\
    "points", llList2Json(JSON_ARRAY,(pts))]),""); }  
//
//  Path scan stop -- sends link message to scan task
//
#define pathscanstop() {  \
    llMessageLinked(LINK_THIS, LINKMSGSCANREQUEST, \
    llList2Json(JSON_OBJECT,["request","stopscan"]),""); }  

