//
//  pathmovecall.lsl  -- caller side interface to pathmove
//
#define LINKMSGMOVEREQUEST  1010                            // message types
#define LINKMSGMOVEREPLY    1011
#define LINKMSGRECOVERREQUEST 1012                         // passed to pathrecover
#define LINKMSGRECOVERREPLY 1013                          // returned from pathrecover
//
//  Path move start -- sends link message to move task
//
//  This is a macro to avoid an extra recopy of "pts", which can be large.
//  The "execute" task is tight on space.
//
#define pathmovestart(pts, width, height, chartype, target, speed, turnspeed, pathid, msglev) {  \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT,["request","startmove","width", (width), "height", (height), "chartype", (chartype), "target", (target),\
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
//  Path move recover -- sends link message to move task to recover to good position
//
#define pathmoverecover(pathid, width, height, chartype) \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT, ["request", "recover", "pathid", (pathid), "width", (width), "height", (height), "chartype", (chartype)]),"");
    
//
//  Messages to and from recover task
//

#define pathrecoverreply(pathid, status, hitobj) \
    llMessageLinked(LINK_THIS, LINKMSGMOVEREQUEST, \
    llList2Json(JSON_OBJECT, ["reply", "recover", "pathid", (pathid), "status",status]),hitobj);
