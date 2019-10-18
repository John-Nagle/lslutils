//
//  pathmovecall.lsl  -- caller side interface to pathmove
//
#define LINKMSGMOVEREQUEST  1010                            // message types
#define LINKMSGMOVEREPLY    1011
//
//  Path move start -- sends link message to move task
//
//  This is a macro to avoid an extra recopy of "pts", which can be large.
//  The "execute" task is tight on space.
//
#define pathmovestart(pts, width, height, target, speed, turnspeed, pathid, msglev) {  \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT,["request","startmove","width", (width), "height", (height), "target", (target),\
    "speed", (speed), "turnspeed", turnspeed,\
    "pathid",(pathid), "msglev",(msglev),\
    "points", llList2Json(JSON_ARRAY,(pts))]),""); }  
//
//  Path move stop -- sends link message to move task
//
#define pathmovestop() {  \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT,["request","stopmove"]),""); }  
//
//  Path move recover -- sends link message to recover to good position
//
#define pathmoverecover(pathid) \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT, ["request", "recover", "pathid", (pathid)]),"");
