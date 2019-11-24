//
//   pathbuildutils.lsl -- components of a path building system
//
//   Part of a system for doing pathfinding in Second Life
//
//   Animats
//   June, 2019
//
#ifndef PATHBUILDUTILSLSL                                   // include guard, like C/C++
#define PATHBUILDUTILSLSL
#include "npc/assert.lsl"                                   // assert
#include "npc/pathmazedefs.lsl"
#include "debugmsg.lsl"
//
//  Constants
//
integer CASTRAYRETRIES = 10;                                // retry up to 10 times
float CASTRAYRETRYDELAY = 0.200;                            // if a problem, retry slowly
float GROUNDCLEARANCE = 0.20;                               // (m) avoid false ground collisions
float PATHCHECKTOL = 0.20;                                  // (m) allow 20cm collinearity error
float PATHPOINTONSEGDIST = 0.10;                            // (m) allow point up to 10cm off line when checking for what seg contains a point
float PATHSTATICTOL = 0.30;                                 // (m) allow extra space on either side of path 
float PATHSINMAXWALKABLEANGLE = 0.4226;                     // sine of (90-65) degrees. 
float MAZEBELOWGNDTOL = 2.0;                                // (m) huge tolerance to allow for huge differences between pathfinding mesh and ground
#define REGION_SIZE (256.0)                                 // (m) Ought to be an LSL call         

list PATHCASTRAYOPTS = [RC_REJECT_TYPES,RC_REJECT_LAND, RC_MAX_HITS,2, RC_DATA_FLAGS,RC_GET_ROOT_KEY]; // 2 hits, because we can hit ourself and must ignore that.
////list PATHCASTRAYOPTSOBS = [RC_MAX_HITS,2];                  // 2 hits, because we can hit ourselves and must ignore that
////list PATHCASTRAYOPTSOBS = [RC_MAX_HITS,2, RC_DATA_FLAGS,RC_GET_NORMAL|RC_GET_ROOT_KEY];   // 2 hits, plus we need the normal
list PATHCASTRAYOPTSOBS;                                    // filled in later because we need to do an OR




#ifndef INFINITY                                            // should be an LSL builtin
#define INFINITY ((float)"inf")                             // is there a better way?
#endif // INFINITY
#ifndef NAN
#define NAN ((float)"nan")
#endif // 

//  Debug interface
#define pathMsg debugMsg
//
//  Error levels
#define PATH_MSG_ERROR DEBUG_MSG_ERROR                              // fatal - debug popups, restarts.
#define PATH_MSG_WARN DEBUG_MSG_WARN
#define PATH_MSG_INFO DEBUG_MSG_INFO
#define PATH_MSG_DEBUG DEBUG_MSG_DEBUG

#define gPathMsgLevel gDebugMsgLevel                            // ***TEMP*** until renaming
//  
//
//  Initial globals needed by all scripts. Set once and never changed
//
float gPathWidth = 0.0;                                     // these are overrridden at init
float gPathHeight = 0.0;                                    // invalid
integer gPathChartype = -1;                                 // invalid
key gPathSelfObject;                                        // key of root prim
integer gPathConstantsInitialized = FALSE;                   // constants initalized
key gPathOwner;                                                 // owner of object
key gPathGroup;                                                 // group of object

//
//  pathinitparams -- parse message which has the initial params all scripts need
//
//  All scripts should call this if message num PATHPARAMSINIT received.
//
pathinitparams(string jsn)
{   assert(gPathConstantsInitialized);                      // startup init done
    if (llJsonGetValue(jsn,["request"]) != "pathparams") { return; } // not for us
    gPathWidth = (float)llJsonGetValue(jsn,["width"]);        // gPathWidth
    gPathHeight = (float)llJsonGetValue(jsn,["height"]);      // height
    gPathChartype = (integer)llJsonGetValue(jsn,["chartype"]);// character type
    gPathMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);  // get message level
    assert(gPathWidth > 0 && gPathHeight > 0);              // sanity check
}
//
//  Debug marker generation
//
#ifdef MARKERS
string MARKERLINE = "Path marker, rounded (TEMP)";
integer LINKMSGMARKER = 1001;                               // for sending to marker service

//  Colors (RGBA)
rotation TRANSGREEN = <0,0.35,0,0.5>;                          // translucent green
rotation TRANSRED   = <1,0,0,0.5>;
rotation TRANSYELLOW = <1,1,0,0.5>;



//
//  Create a marker with the requested parameters
//
//  Rez now, wait for message, send details to marker.
//  We can't send enough data during the rez. We have to use a message for the params.
//
//
//  placesegmentmarker - mark one segment of a path
//
//  "Color" is RGBA.
//
placesegmentmarker(string markername, vector p0, vector p1, float width, rotation rgba, float thickness)
{
    //  Draw marker for segment
    vector midpoint = (p0+p1)*0.5;          // midpoint
    float length = llVecMag(p1-p0);         // how long
    
    vector color = <rgba.x, rgba.y, rgba.z>;
    float alpha = rgba.s;
    vector scale = <length,width,thickness>;    // size of marker
    list params = [ "pos", midpoint, "rot", rotperpenonground(p0,p1), "scale", scale, "color", color, "alpha", alpha];
    llMessageLinked(LINK_THIS, LINKMSGMARKER, llList2Json(JSON_OBJECT,params), markername);   // ask marker service to place a marker.   
}
#endif // MARKERS
//
//  Globals
//
key   gPathLastObstacle = NULL_KEY;                         // last obstacle that stopped us

#define pathGetRoot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))   // get root key of object.

//
//  pathinitutils -- initialize constants if necessary. Call in default state entry.
//
pathinitutils()
{   if (!gPathConstantsInitialized)              // if constants not initialized
    {   PATHCASTRAYOPTSOBS = [RC_MAX_HITS,2, RC_DATA_FLAGS,RC_GET_NORMAL|RC_GET_ROOT_KEY];   // 2 hits, plus we need the normal
        gPathOwner = llGetOwner();                  // owner of animesh
        list groupdetails = llGetObjectDetails(llGetKey(), [OBJECT_GROUP]); // my group
        gPathGroup = llList2Key(groupdetails,0);    // group of animesh
        gPathSelfObject = pathGetRoot(llGetKey());              // our own key, for later
        ////llOwnerSay(llGetScriptName() + "initialized. Root key: " + (string)gPathSelfObject); // ***TEMP***
        gPathConstantsInitialized = TRUE;        // initialize
    }
}
//
//  pathneedmem -- need at least this much free memory. Return TRUE if tight on memory.
//
integer pathneedmem(integer minfree)
{
    integer freemem = llGetFreeMemory();                // free memory left
    if (freemem < minfree)                              // tight, but a GC might help.
    {   pathMsg(PATH_MSG_WARN, "Possibly low on memory. Free mem: " + (string)freemem + ". Forcing GC.");
        integer memlimit = llGetMemoryLimit();          // how much are we allowed?
        llSetMemoryLimit(memlimit-1);                   // reduce by 1 to force GC
        llSetMemoryLimit(memlimit);                     // set it back
        freemem = llGetFreeMemory();                    // get free memory left after GC, hopefully larger.
    }
    if (freemem < minfree)                              // if still too little memory
    {   pathMsg(PATH_MSG_WARN, "Low on memory after GC. Free mem: " + (string)freemem);
        return(TRUE);
    }
    return(FALSE);                                      // no problem
}           


//  True if v between a and b within tolerance tol.
#define pathbetween(v,a,b,tol) ((((v)>(a)-tol) && ((v)<(b)+tol)) || (((v)<(a)+tol) && ((v)>(b)-tol)))
//
//  pathpointinsegment - is p between p0 and p1?
//
//  Tests for opposing directions to p0 and p1.
//  Allow big error in Z direction, 2 meters, because we may be comparing avatar mid-position and a ground point
//
integer pathpointinsegment(vector p, vector p0, vector p1)
{   if (!pathbetween(p.z, p0.z, p1.z, 2.0)) { return(FALSE); }    // way out of the Z range, more than any reasonable half-height
    p.z = 0.0; p0.z = 0.0; p1.z = 0.0;                  // ignore Z axis
    if (llVecMag(p-p0) < 0.001 || llVecMag(p-p1) < 0.001) { return(TRUE); }  // close enough to endpoint to use, avoid divide by zero
    float enddot = (p-p0)*(p1-p0);                                      // dot product of p0->p and p0-p1
    float alldot = (p1-p0)*(p1-p0);                                     // dot with self
    if (enddot < 0 || enddot > alldot) { return(FALSE); }                // outside endpoints
    return(distpointtoline(p,p0,p1) < PATHPOINTONSEGDIST);              // point must be near infinite line
}
 
//
//  distpointtoline -- point to line distance, infinite line
//
//  Formula from geomalgorithms.com
//
float distpointtoline(vector p, vector p0, vector p1)
{
     vector v = p1 - p0;                // the line
     vector w = p - p0;                 
     float c1 = w*v;
     float c2 = v*v;                    // length squared
     vector pb;
     if (c2 > 0.00001)                  // nonzero line length
     {  float b = c1 / c2;              // would divide by 0 for null line
        pb = p0 + b * v;                // closest point on line to p
     } else {
        pb = p0;                        // zero length line case
     }
     return (llVecMag(p - pb));             // return distance
}

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
//  RotBetween - rotation between two vectors, with proper normalization
//
//  UNSOUND FOR TOTALLY OPPOSED VECTORS
//  USE ONLY FOR DEBUG MARKERS
//
#define RotBetween(p0, p1) \
    (NormRot(llRotBetween(llVecNorm((p0)),llVecNorm((p1)))))
     
//
//  RotFromXAxis  --  rotation from X axis in XY plane.
//
//  Used to set up coordinate system for maze solving
//
#define RotFromXAxis(dv) llAxes2Rot(llVecNorm(dv),<0,0,1>%llVecNorm(dv),<0,0,1>)

//
//  pathvecmagxy -- vector magnitude, XY plane only.
//
float pathvecmagxy(vector v) { return(llVecMag(<v.x, v.y, 0>));}            // vector magnitude, XY plane only

//
//  pathvaliddest - is target valid (same sim and same parcel ownership?)
//
key gPathAlllowedParcel;                                // last parcel checked, as cache

integer pathvaliddest(vector pos) 
{
    if (pos.x <= 0 || pos.x >= REGION_SIZE || pos.y <= 0 || pos.y >= REGION_SIZE) { return(FALSE); }
    list theredata = llGetParcelDetails(pos, [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP, PARCEL_DETAILS_ID]);
    key thereid = llList2Key(theredata,2);              // ID of the parcel
    if (thereid == gPathAlllowedParcel) { return(TRUE); }   // checked this one already, good to go
    list heredata = llGetParcelDetails(llGetPos(), [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP]);
    integer thereflags = llGetParcelFlags(pos);         // flags for dest parcel
    key thereowner = llList2Key(theredata,0);
    key theregroup = llList2Key(theredata,1);
    if ((llList2Key(heredata,0) != thereowner) // dest parcel must have same ownership
    && (llList2Key(heredata,1) != theregroup))
    {   return(FALSE); } // different group and owner at dest parcel
    //  Check for no-script area
    if ((thereflags & PARCEL_FLAG_ALLOW_SCRIPTS) == 0)            // if scripts off for almost everybody
    {   if (gPathOwner != thereowner)         // different owner, check group
        {   if (((thereflags & PARCEL_FLAG_ALLOW_GROUP_SCRIPTS) == 0) || (gPathGroup != theregroup))
            { return(FALSE); }                                  // no-script area, would die
        }
    }                                                           
    //  Can we enter the destination parcel?
    if ((thereflags & PARCEL_FLAG_ALLOW_ALL_OBJECT_ENTRY) == 0) 
    {   if (gPathOwner != thereowner) 
        {   if (((thereflags & PARCEL_FLAG_ALLOW_GROUP_OBJECT_ENTRY) == 0) || (gPathGroup != theregroup))
            { return(FALSE); }
        }
    }
    gPathAlllowedParcel = thereid;                              // this parcel is OK
    return(TRUE);  // OK to enter destination parcel
}
//
//  pathdistalongseg -- distance along segment
//  
//  Returns INFINITY if outside tolerance.
//  So this is actually a capsule check.
//
float pathdistalongseg(vector p, vector p0, vector p1, float tol)
{   vector line = p1-p0;                // the line
    vector linedir = llVecNorm(line);   // the line's direction
    vector v = p - p0;                  // segment start to p.
    float d = v*linedir;                // distance along line from p0
    vector ponline = p0 + linedir * d;  // closest point on line
    float disttoline = llVecMag(p-ponline); // distance to line
    if (disttoline > tol) { return(INFINITY); }   // too far from line
    if (d < -tol || d > llVecMag(line) + tol) { return(INFINITY); } // too far outside endpoints
    return(d);                          // return distance along segment, which can be <0
}
//  
//  checkcollinear -- are points on list collinear?
//
integer checkcollinear(list pts)
{   integer length = llGetListLength(pts);
    if (length < 3) { return(TRUE); }    // collinear unless at least 3
    integer i;
    vector p0 = llList2Vector(pts,0);
    vector p1 = llList2Vector(pts,-1);  // last
    for (i=1; i<length-1; i++)          // points other than endpoints
    {   float dist = distpointtoline(llList2Vector(pts,i), p0, p1);    // dist from line between endpoints
        if (dist > PATHCHECKTOL) { return(FALSE); } // tolerance 2cm
    }
    return(TRUE);                       // collinear   
}
//
//  pathstraighten -- straighten a path if possible.
//
//  Works in 3D world, not maze cell space.
//
list pathstraighten(list pts, float probespacing)
{   integer n = 0;
    assert(llGetListLength(pts) > 0);                           // must have at least one point
    vector firstpt = llList2Vector(pts,0);                      // only for checking
    vector lastpt = llList2Vector(pts,-1);                      // get last point
    //   Advance through route. On each iteration, either the route gets shorter, or n gets
    //   larger, so this should always terminate.
    while (n < llGetListLength(pts)-3)                          // advancing through route
    {   vector  p0 = llList2Vector(pts,n);                      // get next three points
        vector  p1 = llList2Vector(pts,n+1);                    // get next three points
        vector  p2 = llList2Vector(pts,n+2);                    // get next three points
        //  Try to take a short cut, bypassing p1
        if (obstaclecheckpath(p0, p2, probespacing))            // short cut unobstructed?
        {   pts = llListReplaceList(pts,[],n+1,n+1);            // success, we can delete p2
        } else {                                                // can't delete, so advance
            n = n + 1;
        }
    }
    assert(llList2Vector(pts,0) == firstpt);                    // must preserve first point
    assert(llList2Vector(pts,-1) == lastpt);                    // must preserve last point
    return(pts);                                                // shortened route
}
//
//  pathnearestpointonnavmesh -- get nearest point on navmesh
//
//  Returns ZERO_VECTOR if fail.
//  Expensive, for trouble recovery only.
//
vector pathnearestpointonnavmesh(vector p)
{
    list navmeshinfo = llGetClosestNavPoint(p,[GCNP_STATIC,TRUE,GCNP_RADIUS,5.0]);   // find reasonable point on navmesh
    if (llGetListLength(navmeshinfo) > 0)                   // if found some point on navmesh
    {   return(llList2Vector(navmeshinfo,0));   }           // we can still compute a distance
    pathMsg(PATH_MSG_WARN,"No navmesh near " + (string)p);  // can't even find the navmesh
    return(ZERO_VECTOR);                                    // fails                
}

//
//  pathdistance -- distance measured along a static path
//
//  Used to check if progress is being made.
//
//  Returns < 0 if fail
//
float pathdistance(vector startpos, vector endpos, float gPathWidth, integer gPathChartype)
{
    vector scale = llGetScale();
    vector startposorig = startpos;                         // 
    vector endposorig = endpos;
    //  Try to find position using pathfindwalkable
    //  Find walkable under avatar. Look straight down. Startpos must be on ground.
    startpos.z = pathfindwalkable(startpos, scale.z*0.5, scale.z);
    endpos.z = pathfindwalkable(endpos, scale.z*0.5, scale.z);    // find walkable below dest - big tolerance
    list path;
    integer status = 1;
    if (startpos.z >= 0 && endpos.z >= 0)               // if find walkable worked
    {   path = llGetStaticPath(startpos, endpos, gPathWidth*0.5, [CHARACTER_TYPE,gPathChartype]);
        status = llList2Integer(path,-1);               // status is last value
    }
    if (status != 0)                                    // trouble finding navmesh, use backup plan
    {   pathMsg(PATH_MSG_WARN, "Trouble finding navmesh, trying backup approach.");
        startpos = pathnearestpointonnavmesh(startposorig); // find nearest point on navmesh
        if (startpos == ZERO_VECTOR) { return(-1.0); }
        endpos = pathnearestpointonnavmesh(endposorig);     // for both ends
        if (endpos == ZERO_VECTOR) { return(-1.0); }
        path = llGetStaticPath(startpos, endpos, gPathWidth*0.5, [CHARACTER_TYPE,gPathChartype]);
        status = llList2Integer(path,-1);                   // status is last value
        if (status != 0) 
        {   pathMsg(PATH_MSG_WARN, "Static path error: " + (string)status +  (string)startpos + " to " + (string)endpos + 
                " orig startpos: " + (string)startposorig);
            return(-1.0);                                       // fails
        }
    }
    integer i;
    integer pntcnt = llGetListLength(path) - 1;             // don't count endpoint
    float pathlength = 0.0;                                 // add up path length
    for (i=0; i<pntcnt-1; i++)                              // for all segments
    {   pathlength += llVecMag(llList2Vector(path,i+1)-llList2Vector(path,i)); }
    return(pathlength);                                     // return positive pathlength
}
//
//  pathfindwalkable -- find walkable below point
//
//  Looks straight down.
//  Returns ZERO_VECTOR if fail.
//  Other objects in the way cause the value from the static path to be used.
//
float pathfindwalkable(vector startpos, float abovetol, float belowtol)
{   //  Look downward twice the height, because of seat mispositioning issues
    assert(gPathConstantsInitialized);                              // make sure pathinitutils has been done
    list castresult = castray(startpos+<0.0,0.0,abovetol>,startpos-<0.0,0.0,belowtol>,PATHCASTRAYOPTSOBS);                // cast downwards, must hit walkable
    return(pathcastfoundproblem(castresult, TRUE, FALSE));           // if any non-walkable hits, -1, otherwise ground Z
}

//
//  pathptstowalkable -- move path points down to walkable surface
//
//  Used on the output of llGetStaticPath, which can be off by up to 0.35m.
//
list pathptstowalkable(list path, float gPathHeight)
{   list pts = [];
    integer length = llGetListLength(path);
    integer i;
    for (i=0; i<length; i++)                                    // for all points
    {   vector p = llList2Vector(path,i);
        float newz = pathfindwalkable(p,gPathHeight*0.5,MAZEBELOWGNDTOL);             // look down and find the ground
        if (newz < 0)                                           // can't find walkable surface
        {   pathMsg(PATH_MSG_WARN, "Can't find walkable below pnt " + (string)p);   // where's the ground or floor?
        } else {
            p.z = newz;                                         // use more accurate Z
        }
        pts += p;                                               // add this point
    }
    return(pts);                                                // points on floor
}
//
//  rotperpenonground  -- rotation to get line on ground perpendicular to vector
//
//  Used to construct the end lines of path segments.
//
rotation rotperpenonground(vector p0, vector p1)
{
    vector dv = p1-p0;                                      // direction for line
    vector dvflat = <dv.x, dv.y, 0>;
    rotation azimuthrot = RotFromXAxis(dvflat);             // numerically sound
    rotation elevrot = RotBetween(dvflat, dv);              // elevation
    return(azimuthrot * elevrot);                           // apply in proper order
}
//
//  mazecellto3d  -- convert maze cell to 3D coords
//
//  mazecellsize is the size of a cell in the XY plane, not the 3D plane
//  mazepos is the position of cell (0,0);
//  mazerot is the rotation of the maze plane and must be in the XY plane
//
//  Used in multiple scripts.
//
vector mazecellto3d(integer x, integer y, float mazecellsize, vector mazepos, rotation mazerot)
{
    vector vflat = <x*mazecellsize,y*mazecellsize,0.0>;   // vector to cell in XY plane
    return(vflat*mazerot + mazepos);
}

//
//  castray -- llCastRay with retry
//
//  llCastRay can fail under overload. This is rare, but we have to retry.
//
list castray(vector p0, vector p1, list params)
{
    integer tries = CASTRAYRETRIES;                         // number of retries
    list castresult = [];
    while (tries-- > 0)
    {   
        castresult = llCastRay(p0, p1, params);             // try cast ray
        DEBUGPRINT1("Cast ray: p0: " + (string)p0 + "  p1: " + (string)p1 + " result: " + llDumpList2String(castresult,","));  
        if (llList2Integer(castresult,-1) >= 0)             // if good status
        {   return(castresult); }                           // non-error, return
        pathMsg(PATH_MSG_WARN,"Cast delayed: " + (string) llList2Integer(castresult,-1)); 
        llSleep(CASTRAYRETRYDELAY);                         // error, wait and retry
    }
    //  Too many retries, give up
    return(castresult);                                     // return final failure
}
//
//  castbeam -- do multiple llCastRay operations to check if way is clear for movement
//
//  Returns distance, or INFINITY if no obstacle found, or negative of status code if error.
//  Ignores hits with walkable objects, so this is for horizontal scans only.
//
//  Minimum value of probecnt is 2. ***NOT SURE ABOUT THIS***
//
//  castparams must yield a data format of [rootid, hitpt, ... status]. No normal.
//
float castbeam(vector p0, vector p1, float probespacing, integer wantnearest, list castparams)
{   float yoffset;                                          // cast ray offset, Y dir in coords of vector
    float zoffset;                                          // cast ray offset, Z dir in coords of vector
    float nearestdist = INFINITY;                           // closest hit
    gPathLastObstacle = NULL_KEY;                           // no recent obstacle

    
    ////DEBUGPRINT1("p0: " + (string)p0 + "  p1: " + (string)p1 + " probespacing: " + (string) probespacing);  // ***TEMP***
    integer probecount = (integer)((gPathHeight-GROUNDCLEARANCE)/probespacing); // number of probes
    if (probecount < 1) { probecount = 1; }                 // minimum is one probe
    probespacing = (gPathHeight-GROUNDCLEARANCE)/probecount;     // adjust to match gPathHeight
    if (probespacing < 0.10) { return(-4); }                // Bad call
    vector dir = llVecNorm(p1-p0);                          // direction of raycast 
    vector endoffsetdir = <0,0,1> % dir;                    // offset for horizontal part of scan. Cross product of dir and Z axis.
    ////DEBUGPRINT1("End offset dir: " + (string)endoffsetdir);  // ***TEMP***
    //  Always do 3 scans across gPathWidth - left edge, middle, right edge.
    for (yoffset = -gPathWidth * 0.5; yoffset <= gPathWidth * 0.5 + 0.001; yoffset += (gPathWidth*0.5))
    {   for (zoffset = GROUNDCLEARANCE; zoffset <= gPathHeight  + 0.001; zoffset += probespacing)
        {   ////DEBUGPRINT1("p0: " + (string)p0 + "  p1: " + (string)p1 + "  zoffset: " + (string)zoffset); // ***TEMP***
            vector yadjust = yoffset*endoffsetdir;          // offset for scan crosswise to path
            list castresult = castray(<p0.x, p0.y, p0.z+zoffset>+yadjust, <p1.x, p1.y, p1.z + zoffset>+yadjust, castparams);
            integer status = llList2Integer(castresult, -1);// status is last element in list
            if (status < 0)
            {   pathMsg(PATH_MSG_WARN,"Cast beam status: " + (string)status);
                return((integer)status);                    // fails       
            }
            if (status > 0) 
            {   integer i;
                for (i=0; i<2*status; i+= 2)                        // for all hits
                {   gPathLastObstacle = llList2Key(castresult, i+0);       // get object hit
                    if (gPathLastObstacle != gPathSelfObject)          // ignore hits with self
                    {   vector hitpt = llList2Vector(castresult, i+1); // get point of hit
                        ////list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
                        ////integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
                        ////if (pathfindingtype != OPT_WALKABLE)        // if it's not a walkable
                        {   float dist = (hitpt-p0) * dir;          // distance to hit
                            if (dist < 0) { dist = 0; }             // can potentially be small and negative, from geometry. Treat as zero.
                            if (!wantnearest)                       // if don't need nearest
                            {   return(dist); }                     // just return first
                            if (dist < nearestdist)                 // save closest hit point
                            {   nearestdist = dist; }
                        }
                    }
                }
            }          
        }
    }
    return(nearestdist);   // no obstacles
}
//
//  obstaclecheckpath  -- is path obstacle-free?
//
//  Does both a ray check and a llGetStaticPath check.
//
integer obstaclecheckpath(vector p0, vector p1, float probespacing)
{
    list path = llGetStaticPath(p0,p1,(gPathWidth+PATHSTATICTOL)*0.5, [CHARACTER_TYPE, gPathChartype]);
    integer status = llList2Integer(path,-1);                   // last item is status
    path = llList2List(path,0,-2);                              // remove last item
    if (status != 0 || (llGetListLength(path) > 2 && !checkcollinear(path)))
    {   pathMsg(PATH_MSG_INFO,"Path static check found static obstacle between " + (string)p0 + " to " + (string)p1 + ": " + llDumpList2String(path,","));
        return(FALSE);
    }
    //  Don't test against land, because the static path check did that already.
    float disttohit = castbeam(p0, p1, probespacing, FALSE, PATHCASTRAYOPTS);
    if (disttohit != INFINITY)
    {   DEBUGPRINT1("Obstacle check path from " + (string)p0 + " " + (string)p1 + " hit at " + (string)(p0 + llVecNorm(p1-p0)*disttohit));
        return(FALSE);
    }
    return(TRUE);                                               // success
}
//
//  mazecheckcelloccupied and obstaclecheckcelloccupied -- is there an obstacle in this cell?
//
//  Maze check checks for a valid static path first. Obstacle check does not.
//  Don't need to recheck the static path on ordinary paths.
//
//  Temporary until caller converted. No static check.
#define obstaclecheckcelloccupied(p0, p1,  dobackcorners) \
   (pathcheckcelloccupied(p0, p1, dobackcorners, FALSE) < 0.0)

//  Temporary until caller converted. Does static check
#define mazecheckcelloccupied(p0,p1,dobackcorners) \
    (pathcheckcelloccupied((p0),(p1),(dobackcorners),TRUE))
    

//
//  pathcheckcelloccupied  -- is there an obstacle in this cell?
//
//  Common version for maze solver and planner.
//
//  Checks a cell centered on p1. Assumes the cell centered on p0 is clear. Alignment of cell is p1-p0.
//
//  This does some ray casts straight down, and some horizontally. Follows the maze plane for "horizontal"
//  casts.
//
//  We don't have to check the trailing edge of the cell, because we are coming from a clear cell at p0,
//  so that was already checked.
//
//  No static path check, but it has to hit a walkable.
//
//  p0 and p1 must be one gPathWidth apart. They are positions at ground level. Z values will be different on slopes.
//  
//  New Z-aware version.
//  Returns Z value of ground at P1, or -1 if occupied.
//  At entry, p0 must have a correct Z value, but p1's Z value is obtained by a ray cast downward.
//
float pathcheckcelloccupied(vector p0, vector p1, integer dobackcorners, integer dostaticcheck)
{
    vector dv = p1-p0;                                      // direction, unnormalized
    dv.z = 0;                                               // Z not meaningful yet.
    vector dir = llVecNorm(dv);                             // forward direction, XY plane
    vector fullheight = <0,0,gPathHeight>;                       // add this to top of vertical ray cast
    vector halfheight = <0,0,gPathHeight*0.5>;                   // add this for casts from middle of character
    vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
    pathMsg(PATH_MSG_INFO, "Cell occupied check: p0: " + (string) p0 + " p1: " + (string)p1);
    vector crossdir = dir % <0,0,1>;                        // horizontal from ahead point
    vector fwdoffset = dir*(gPathWidth*0.5);                     // distance to look ahead, to end of this cell
    vector sideoffset = crossdir*(gPathWidth*0.5);               // distance to the side of the cell
    //  Initial basic downward cast. This gets us our Z reference for P1
    p1.z = p0.z;                                            // we have no Z value for P1 yet. Start with the one from P0.
    //  Look over a wide enough range of Z values to catch all walkable slopes.
    float zp1 = obstacleraycastvert(p1 + fullheight, p1-mazedepthmargin-<0,0,1>*(gPathWidth*(1.0+PATHSINMAXWALKABLEANGLE)));   // probe center of cell, looking down
    if (zp1 < 0) { return(-1.0); }                          // fails
    p1.z = zp1;                                             // we can now set p1's proper Z height
    //  Do static check if requested.
    if (dostaticcheck)
    {   //  Do a static path check for this short path between two maze cells
        list path = llGetStaticPath(p0,p1,gPathWidth*0.5, [CHARACTER_TYPE, gPathChartype]);
        integer status = llList2Integer(path,-1);           // last item is status
        path = llList2List(path,0,-2);                      // remove last item
        if (status != 0 || (llGetListLength(path) > 2 && !checkcollinear(path)))
        {   ////pathMsg(PATH_MSG_INFO,"Maze path static check found static obstacle between " + (string)p0 + " to " + (string)p1 + ": " + llDumpList2String(path,","));
            return(-1.0);
        }
    }
    //  End static check    
    vector pa = p1 + sideoffset;                            // one edge at ground level
    vector pb = p1 - sideoffset;                            // other edge at ground level
    vector pc = p1 + fwdoffset;                             // ahead at ground level
    vector pd = p1 - fwdoffset;                             // "behind" point
    ////pathMsg(PATH_MSG_INFO, "pa/pb/pc/pd: " + (string)pa + (string)pb+(string)pc+(string)pd);    // ***TEMP***
    //  Horizontal casts.
    //  Horizontal checks in forward direction to catch tall obstacles or thin ones.
    //  First cast from front edge of cell p0 to front edge of cell p1, at half height. The most important ray cast.
    if (obstacleraycasthoriz(p0+halfheight+fwdoffset, pc + halfheight)) { return(-1.0); }// Horizontal cast at mid height, any hit is bad
    if (obstacleraycasthoriz(p0+fullheight+fwdoffset, pc + fullheight)) { return(-1.0); }// Same at full height to check for low clearances
    if (obstacleraycasthoriz(p0+halfheight+fwdoffset+sideoffset, pa + halfheight)) { return(-1.0); }// Horizontal cast at mid height, any hit is bad
    if (obstacleraycasthoriz(p0+halfheight+fwdoffset-sideoffset, pb + halfheight)) { return(-1.0); }// Horizontal cast at mid height, any hit is bad
    //  Crosswise horizontal check.
    if (obstacleraycasthoriz(pa+halfheight,pb+halfheight)) { return(-1.0); }   // Horizontal cast crosswize at mid height, any hit is bad
    //  Downward ray casts only.  Must hit a walkable.
    //  Center of cell is clear and walkable. Now check upwards at front and side.
    //  The idea is to check at points that are on a circle of diameter "gPathWidth"
    if (obstacleraycastvert(pa+fullheight,pa-mazedepthmargin) < 0) { return(-1.0); }   
    if (obstacleraycastvert(pb+fullheight,pb-mazedepthmargin) < 0) { return(-1.0); } // cast downwards, must hit walkable
    if (obstacleraycastvert(pc+fullheight,pc-mazedepthmargin) < 0) { return(-1.0); } // cast downwards, must hit walkable
    if (obstacleraycastvert(pd+fullheight,pc-mazedepthmargin) < 0) { return(-1.0); } // cast at steep angle, must hit walkable
    if (!dobackcorners) 
    {   DEBUGPRINT1("Cell at " + (string)p1 + " empty.");           
        return(zp1);                                            // return Z
    }
    //  Need to do all four corners of the square. Used when testing and not coming from a known good place.
    if (obstacleraycastvert(pd+fullheight,pd-mazedepthmargin) < 0) { return(-1.0); }; // cast downwards for trailing point
    return(zp1);                                                // success, no obstacle, return Z value for ground below P1.
}
//
//  pathcheckcellz -- get Z height of ground below base of cell
//
//  Same result for that as pathcheckcelloccupied, but fewer checks.
//  Used when we need the Z height for a second time.
//
float pathcheckcellz(vector p0, vector p1)
{
    vector fullheight = <0,0,gPathHeight>;                       // add this to top of vertical ray cast
    vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
    //  Initial basic downward cast. This gets us our Z reference for P1
    p1.z = p0.z;                                            // we have no Z value for P1 yet. Start with the one from P0.
    //  Look over a wide enough range of Z values to catch all walkable slopes.
    return(obstacleraycastvert(p1 + fullheight, p1-mazedepthmargin-<0,0,1>*(gPathWidth*(1.0+PATHSINMAXWALKABLEANGLE))));   // probe center of cell, looking down
}



//
//  obstacleraycasthoriz -- test for horizontal ray casts.  Must find open space.  Returns TRUE if obstacle.
//
integer obstacleraycasthoriz(vector p0, vector p1)
{   
    list castresult = castray(p0,p1,PATHCASTRAYOPTSOBS);        // Horizontal cast at full height, any hit is bad
    float result = pathcastfoundproblem(castresult, FALSE, FALSE);     // if any hits at all, other than self, fail
    if (result < 0) { pathMsg(PATH_MSG_INFO, "Horiz raycast fail: "+ (string)p0 + " " +  (string)p1); } // ***TEMP***
    if (result < INFINITY) { return(TRUE); };                   // obstacle
    //  And in other direction, in case start is inside an object
    castresult = castray(p1,p0,PATHCASTRAYOPTSOBS);        // Horizontal cast at full height, any hit is bad
    result = pathcastfoundproblem(castresult, FALSE, FALSE);     // if any hits at all, other than self, fail
    if (result < 0) { pathMsg(PATH_MSG_INFO, "Horiz raycast fail: "+ (string)p0 + " " +  (string)p1); } // ***TEMP***
    return(result < INFINITY);                                  // TRUE if obstacle
}

//
//  obstacleraycastvert -- test for downward ray casts. Must find a walkable.  Returns -1 if problem, Z height otherwise.
//
//
float obstacleraycastvert(vector p0, vector p1)
{
    list castresult = castray(p0,p1,PATHCASTRAYOPTSOBS);        // cast downwards, must hit walkable
    float result = pathcastfoundproblem(castresult, TRUE, FALSE);      // if any non-walkable hits, fail
    if (result < 0) { pathMsg(PATH_MSG_INFO, "Vert raycast fail: "+ (string)p0 + " " +  (string)p1 + " cast result: " + llDumpList2String(castresult,",")); } // ***TEMP***
    return(result);
}
//
//  pathcastfoundproblem  -- analyze result of llCastRay. Input must have [pos, key, normal ... ]
//
//  Returns -1.0 for fails. INFINITY for no hit. Z height of object hit for walkables.
//
//  A walkable must be roughly horizontal, below PATHSINMAXWALKABLEANGLE. This includes the ground.
//
float pathcastfoundproblem(list castresult, integer needwalkable, integer ignoresolids)
{
    integer status = llList2Integer(castresult, -1);        // status is last element in list
    if (status == 0) 
    {   if (needwalkable) { return(-1.0); }                 // needed to find ground
        return(INFINITY);                                   // hit nothing, infinite distance
    }
    if (status < 0)  { return(-1.0); }                      // problem, fails
    //  Hit something. Must analyze.
    //  Hit ourself, ignore. 
    //  Hit land or walkable, return TRUE.
    //  Hit nothing but ignorables, return nohitval  
    integer i;
    for (i=0; i<3*status; i+=3)                             // check objects hit. Check two, because the first one might be ourself
    {
        key hitobj = llList2Key(castresult, i+0);           // get object hit
        if (hitobj != gPathSelfObject)                      // if hit something other than self.
        {   vector hitpt = llList2Vector(castresult, i+1);  // get point of hit
            if (hitobj != NULL_KEY)                         // if not ground, must check pathfinding type
            {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
                integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
                if (pathfindingtype != OPT_WALKABLE)                    // if it's not a walkable
                {   ////pathMsg(PATH_MSG_DEBUG,"Hit non-walkable " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) + " at " + (string)(hitpt));
                    if (!ignoresolids)                      // unless ignoring non-walkable solids
                    {   return(-1.0);  }                    // hit non-walkable, obstructed
                }
            }
            //  Hit a walkable. Check normal angle
            vector hitnormal = llList2Vector(castresult, i+2);  // normal to surface
            if ((hitnormal * <0,0,1.0>) < PATHSINMAXWALKABLEANGLE) { return (-1.0); } // too steep for a walkable
            return(hitpt.z);                                // otherwise Z value of walkable.
        }
    }
    if (needwalkable) { return(-1.0); }                     // needed a walkable and didn't find one.
    return(INFINITY);                                       // hit nothing of interest   
}
//
//  pathanalyzecastresult  -- analyze the result of a cast ray probe
//
//  Uses short form cast results.
//
//  Returns: 
//  []              No problem
//  [KEY,VECTOR]    Hit obstacle
//  [INTEGER]       Error status
//
list pathanalyzecastresult(list castresult, integer needwalkable)
{   integer status = llList2Integer(castresult, -1);        // status is last element in list
    if (status == 0) 
    {   if (needwalkable) { return([PATHEXENOTWALKABLE]); } // need ground support and don't have it
        return([]);                                         // OK
    }
    if (status < 0)  
    {   return([MAZESTATUSCASTFAIL]); }                     // problem, fails
    //  Hit something. Must analyze.
    //  Hit ourself, ignore. 
    //  Hit land or walkable, ignore.
    //  Otherwise, report.
    integer i;
    for (i=0; i<2*status; i+=2)                             // check objects hit. Check two, because the first one might be ourself
    {
        key hitobj = llList2Key(castresult, i+0);           // get object hit
        if (hitobj == NULL_KEY) { return([]); }             // land is walkable, so, OK
        if (hitobj != gPathSelfObject)                      // if hit something other than self.
        {   vector hitpt = llList2Vector(castresult, i+1);            // get point of hit
            list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)            // if it's not a walkable
            {   return([hitobj, hitpt]);                    // obstacle
            }
            return([]);                                     // we hit a walkable - good.
        }
    }
    if (needwalkable) { return([PATHEXENOTWALKABLE]); }     // need ground support and don't have it
    return([]);                                             // OK
}


//
//  pathcalccellmovedist 
//
//  How far to move in dir to get an integral number of cells between points.
//  We need to move the point by a small distance in dir to achieve this.
//  Movement must always increase prevdist
//
//  Goal: |endpt - (pnt+dir*movedist)| mod cellsize == 0
//
//  with all the vectors in the XY plane only.
//
//  Derivation:
//
//    Number of cells needed, rounding up from what we have
//
//      unitcells = ceil(|pnt-endpt| / cellsize)
//
//    So we want this relationship:
//
//      |endpt - (pnt+dir*movedist)| = unitcells * cellsize  
//
//    Solve for movedist.
//
//      Let cdist = unitcells*cellsize
//      Let cdistsq = cdist^2
//      Let dv = endpt - pnt
//      Let dvsq = dv*dv
//
//    so
//
//      |dv - dir*movedist| = cdist 
//
//    or
//
//      |dv - dir*movedist|^2 = cdist^2
//
//    Expand length of vector
//      
//      (dv - dir*movedist)*(dv - dir*movedist) = cdistsq
//
//      dv*dv - 2*dir*movedist*dv + dir*dir*movedist*movedist = cdistsq
//
//      (dir*dir) * movedist^2 + movedist*(2*dir) + dv*dv - cdistsq = 0
//
//   Which is a quadratic.
//
//      a = dir*dir = 1
//      b = 2*dir*dv
//      c = dvsq - cdistsq
//
//      
//  This is just geometry.
//  It's really finding a point which is on an XY plane circle centered at endpt and an
//  integral number of widths from it, and also on the line from pnt along vector dist.
//
//
float pathcalccellmovedist(vector pnt, vector dir3d, vector endpt, float cellsize, float prevdist3d)
{
    if (endpt == ZERO_VECTOR) { return(prevdist3d+cellsize); }   // no endpt constraint, simple solution
    vector endptflat = <endpt.x, endpt.y, 0.0>;     // project endpoint into XY plane
    vector pntflat = <pnt.x,pnt.y,0.0>;             // project pnt into XY plane
    vector dirflat = llVecNorm(<dir3d.x, dir3d.y,0.0>);    // project direction into XY plane
    float scale2d = dirflat*dir3d;                  // scale 3D lengths in dir down to 2D
    float prevdistflat = prevdist3d * scale2d;      // XY plane length of prevdist
    vector p0 = pntflat+(dirflat*(prevdistflat+0.01));  // starting point, just past prevdist to force progress
    assert(llFabs(p0.z) < 0.001);                   // p0 is in XY plane
    float p0toendptflat = llVecMag(p0-endptflat);   // approx dist to center
    integer unitcells = llCeil(p0toendptflat/cellsize);     // number of cells desireed between endpoints
    float cdist = unitcells*cellsize;               // desired distance to endpt
    float cdistsq = cdist*cdist;                    // distance squared
     //  Wanted: |endptflat - (pntflat+dirflat*movedist)| = cdist 
    vector dv = endptflat - pntflat;                // start point to center  
    float dvsq = dv*dv;                             // distance squared
    float a = 1;                                    // quadratic solution
    float b = 2*(dv*dirflat);
    float c = dvsq - cdistsq;
    float numersq = b*b-4*a*c;                       // term under radical in quadratic equation
    if (numersq < 0.0) { return(NAN); }              // Error
    float numer = llSqrt(numersq);                   // must be nonnegative
    float movedistflat = (-b - numer) / (2*a);       // the smaller quadatic solution.
#ifdef OBSOLETE
    DEBUGPRINT1("path cell move calc.  llFabs(llVecMag((endptflat - (pntflat+dirflat*(-movedistflat))() : " 
        + (string) llFabs(llVecMag((endptflat - (pntflat+dirflat*(-movedistflat)))))
        + " unit cells: " + (string)unitcells + " cell size: " + (string)cellsize + " pntflat: " + (string)pntflat + " endpflat: "
        + (string)endptflat +  " p0: " + (string)p0 + " dirflat: " + (string)dirflat + " movedistflat: "  
        + (string)movedistflat);
#endif // OBSOLETE
    assert(llFabs(a*movedistflat*movedistflat + b*movedistflat + c) < 0.001);   // quadratic equation check
    movedistflat = -movedistflat;                   // ***NOT SURE ABOUT THIS***
    if (movedistflat < 0) { return(NAN); }
    assert(llFabs(llVecMag(endptflat - (pntflat + dirflat*movedistflat)) - unitcells*cellsize) < 0.01); // math check
    assert(movedistflat > prevdistflat);            // must increase dist  
    float movedist3d = movedistflat / scale2d;      // scale up for 3D
    return(movedist3d);                             // move this far along segment in 3D 
}
//
//
//  pathfindunobstructed -- find an unobstructed point near a path segment end.
//
//  Returns [pnt,ix], where ix is the point index previous, in the direction of scan, to the
//  scan, of the point found.
//
list pathfindunobstructed(list pts, integer ix, integer fwd)
{
    assert(fwd == 1 || fwd == -1);
    DEBUGPRINT1("Looking for unobstructed point on segment #" + (string)ix + " fwd " + (string)fwd); 
    if (ix < 0 || ix+fwd<0) { return([ZERO_VECTOR,-1]);} 
    integer length = llGetListLength(pts);
    vector p0 = llList2Vector(pts,ix);      // index of previous point
    vector p1 = llList2Vector(pts,ix + fwd);// index of next point
    vector dir = llVecNorm(p1-p0);          // move dir
    float distalongseg = 0.001;                  // just enough to get into segment
    // distance along segment starting at ix.
    while (TRUE)                                // until return
    {   
        p0 = llList2Vector(pts,ix);             // index of previous point
        p1 = llList2Vector(pts,ix + fwd);       // index of next point
        dir = llVecNorm(p1-p0);                 // move dir
        vector pos = p0 + llVecNorm(p1-p0) * distalongseg; // next point to try
        float vlen = llVecMag(p1-p0);           // length of vector
        if (distalongseg > vlen)                // if hit end of segment
        {   ix = ix + fwd;                      // advance one seg in desired dir
            if (ix + fwd >= length || ix + fwd < 0) // end of entire path without find
            {   DEBUGPRINT1("Fail: no clear point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd);  
                return([ZERO_VECTOR,-1]);       // hit end of path without find, fails
            }
            distalongseg = 0.001;               // just enough into new seg

        } else {                                // did not hit end of segment
            DEBUGPRINT1("Trying point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd);  
            if (!obstaclecheckcelloccupied(p0, pos, TRUE))
            {   DEBUGPRINT1("Found clear point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd); 
                return([pos,ix]);               // found an open spot
            }
            distalongseg += gPathWidth;              // advance one gPathWidth
        }
    }
    //  Unreachable
    return([ZERO_VECTOR,-1]);                   // no way
}
//
//  pathclean - remove very short segments from path. llGetStaticPath has a problem with this.
//
list pathclean(list path)
{   
    integer len = llGetListLength(path);                    // number of points on path
    if (len < 2)  { return([]); }                           // avoid single point problems
    if (len < 3)  { return(path); }                         // too short to clean
    vector prevpt = llList2Vector(path,0);                  // previous point
    vector lastpt = llList2Vector(path,-1);                 // last point
    if (llVecMag(prevpt-lastpt) < MINSEGMENTLENGTH)         // if too short for a valid move 
    {   return([]); }
    list newpath = [prevpt];                                // which is the first output point
    integer i;
    for (i=1; i<len; i++)                                   // for all points after first
    {   vector pt = llList2Vector(path,i);                  // get next point
        float dist = llVecMag(pt - prevpt);                 // segment length
        if (dist > MINSEGMENTLENGTH)                        // if long enough to keep
        {   vector nextpt = llList2Vector(path,i+1);        // will go off end and be zero
            if (distpointtoline(prevpt, pt, nextpt) > PATHCHECKTOL)  // skip if collinear
            {
                newpath += pt;                              // add new point
                prevpt = pt;
            }
        }
    }
    //  If the last point got optimized out for being too close to
    //  the previous point, replace it with the last point.
    //  The first and last points must always be preserved.
    if (lastpt != llList2Vector(newpath,-1))
    {   newpath = llListReplaceList(newpath,[lastpt],-1,-1); }
    if (llGetListLength(newpath) < 2) { return([]); }       // return empty for two identical points
    return(newpath);                                        // cleaned up path
}
//
//  pathtrimmedstaticpath -- trim off end of path by specified distance
//
//  Used to make Pursue stop before hitting the target
//
//  Width may be wider than gPathWidth here.
//
list pathtrimmedstaticpath(vector startpos, vector endpos, float stopshort, float width)
{   list path = llGetStaticPath(startpos, endpos, width*0.5, [CHARACTER_TYPE,gPathChartype]);    // plan a static path
    integer status = llList2Integer(path,-1);               // static path status
    if (stopshort <= 0) return(path);                       // no need to trim
    if (status != 0) { return([status]); }                  // fails
    path = llList2List(path,0,-2);                          // path without status
    while (llGetListLength(path) > 1)                       // while at least two points
    {   vector p0 = llList2Vector(path,-1);                 // last two points, reverse order
        vector p1 = llList2Vector(path,-2);
        ////DEBUGPRINT1("Stopshort test, at " + (string)p0);    // ***TEMP***
        vector dv = p1-p0;
        float dvmag = llVecMag(dv);
        if (dvmag > 0.001 && dvmag - stopshort > 0.001)     // trim is in this segment
        {   vector p = p0 + llVecNorm(dv) * stopshort;      //
            ////pathMsg(PATH_MSG_WARN,"Short stop path trimmed from " + (string)endpos + " to " + (string)p); // ***TEMP***
            return(llListReplaceList(path,[p, 0],-1,-1));   // replace last point, add status             
        }
        path = llList2List(path,0,-2);                      // drop last point
        stopshort -= dvmag;                                 // decrease trim dist by last segment
    }
    return([0]);                                            // we're so close we're there. No points, good status.
}

//
//  pathdonereply -- pathfinding is done, tell requesting script
//
pathdonereply(integer status, key hitobj, integer pathid)
{   
    llMessageLinked(LINK_THIS, PATHPLANREPLY, llList2Json(JSON_OBJECT,["status",status,"pathid",pathid]), hitobj);  // status to client
}

#endif // PATHBUILDUTILSLSL

