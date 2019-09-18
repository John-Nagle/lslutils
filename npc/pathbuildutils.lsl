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
#include "npc/mazedefs.lsl"
//
//  Constants
//
integer CASTRAYRETRIES = 10;                                // retry up to 10 times
float CASTRAYRETRYDELAY = 0.200;                            // if a problem, retry slowly
float GROUNDCLEARANCE = 0.05;                               // (m) avoid false ground collisions
float PATHCHECKTOL = 0.02;                                  // (m) allow 2cm collinearity error
float PATHPOINTONSEGDIST = 0.10;                            // (m) allow point up to 10cm off line when checking for what seg contains a point
float PATHSTATICTOL = 0.10;                                 // (m) allow extra space on either side of path 

#define PATHZTOL (0.35)                                     // (m) allow this much error from llGetStaticPath

list PATHCASTRAYOPTS = [RC_REJECT_TYPES,RC_REJECT_LAND, RC_MAX_HITS,2]; // 2 hits, because we can hit ourself and must ignore that.
list PATHCASTRAYOPTSOBS = [RC_MAX_HITS,2];                  // 2 hits, because we can hit ourselves and must ignore that

#ifndef INFINITY                                            // should be an LSL builtin
#define INFINITY ((float)"inf")                             // is there a better way?
#endif // INFINITY
#ifndef NAN
#define NAN ((float)"nan")
#endif // 


//
//  Error levels
integer PATH_MSG_ERROR = 0;
integer PATH_MSG_WARN = 1;
integer PATH_MSG_INFO = 2;
integer PATH_MSG_DEBUG = 3;
//  

//
integer PATH_MIN_IM_INTERVAL = 3600;                            // seconds between IMs. Do not overdo.
//                                  // Unused

//
//  Global
//
integer gPathMsgLevel = 0;                                      // debug logging off by default. Set this to change level
integer gPathLastIMTime = 0;                                    // last instant message sent. Do this rarely.

//
//  pathMsg  -- call for pathfinding problems
//
#define pathMsg(level, msg) { if (level <= gPathMsgLevel) { pathMsgFn((level),(msg)); }} // as macro, to prevent building message that will never print

pathMsgFn(integer level, string msg)                            // print debug message
{   if (level > gPathMsgLevel) { return; }                      // ignore if suppressed
    string s = llGetScriptName() + ": " + msg;                  // add name of script
    llOwnerSay(s);                                              // message
    if (level <= PATH_MSG_ERROR) 
    {   llSay(DEBUG_CHANNEL, s);                                // serious, pop up the red message
        integer now = llGetUnixTime();
        if (now - gPathLastIMTime > PATH_MIN_IM_INTERVAL)       // do this very infrequently
        {   llInstantMessage(llGetOwner(), llGetObjectName() + " in trouble at " + llGetRegionName() + " " + (string)llGetPos() + ": " + s);     // send IM to owner
            gPathLastIMTime = now;
        } 
    }                            
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
float gPathWidth = 0.5;                                     // dimensions of character
float gPathHeight = 1.8;                                    // defaults, overridden in JSON
key   gPathSelfObject = NULL_KEY;                           // my own key

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
////{   return(llVecMag(p-p0) < 0.001 || llVecMag(p-p1) < 0.001 || ((llVecNorm(p-p0)) * llVecNorm(p-p1)) < 0.999); }
{   if (!pathbetween(p.z, p0.z, p1.z, 2.0)) { return(FALSE); }    // way out of the Z range, more than any reasonable half-height
    p.z = 0.0; p0.z = 0.0; p1.z = 0.0;                  // ignore Z axis
    if (llVecMag(p-p0) < 0.001 || llVecMag(p-p1) < 0.001) { return(TRUE); }  // close enough to endpoint to use, avoid divide by zero
    float enddot = (p-p0)*(p1-p0);                                      // dot product of p0->p and p0-p1
    float alldot = (p1-p0)*(p1-p0);                                     // dot with self
    if (enddot < 0 || enddot > alldot) { return(FALSE); }                // outside endpoints
    ////if ((llVecNorm(p-p0) * llVecNorm(p-p1)) > 0) { return(FALSE); } // normals in same direction means outside endpoints
    return(distpointtoline(p,p0,p1) < PATHPOINTONSEGDIST);              // point must be near infinite line
    ////return(llVecMag(p-p0) < 0.001 || llVecMag(p-p1) < 0.001 || ((llVecNorm(p-p0)) * llVecNorm(p-p1)) < -0.99); // opposed normals
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
list pathstraighten(list pts, float width, float height, float probespacing, integer chartype)
{   integer n = 0;
    //   Advance through route. On each iteration, either the route gets shorter, or n gets
    //   larger, so this should always terminate.
    while (n < llGetListLength(pts)-3)                          // advancing through route
    {   vector  p0 = llList2Vector(pts,n);                      // get next three points
        vector  p1 = llList2Vector(pts,n+1);                    // get next three points
        vector  p2 = llList2Vector(pts,n+2);                    // get next three points
        //  Try to take a short cut, bypassing p1
        if (obstaclecheckpath(p0, p2, width, height,probespacing, chartype))
        {   pts = llListReplaceList(pts,[],n+1,n+1);            // success, we can delete p2
        } else {                                                // can't delete, so advance
            n = n + 1;
        }
    }
    return(pts);                                                // shortened route
}

//
//  pathdistance -- distance measured along a static path
//
//  Used to check if progress is being made.
//
//  Returns < 0 if fail
//
float pathdistance(vector startpos, vector endpos, float width, integer chartype)
{
    vector startscale = llGetScale();
    vector startposorig = startpos;                         // ***TEMP***
    startpos.z = (startpos.z - startscale.z*0.45);          // approx ground level for start point
#ifdef OBSOLETE // endpos is already at ground level. We did this already.
    //  Find walkable under avatar. Look straight down. Startpos must be on ground.
    vector endposorig = endpos;
    endpos = pathfindwalkable(endpos, startscale.z);        // find walkable below dest
    if (endpos == ZERO_VECTOR)
    {   pathMsg(PATH_MSG_WARN, "No walkable below path distance goal at " + (string)endposorig); 
        return(-1.0);
    }
#endif // OBSOLETE
    list path = llGetStaticPath(startpos, endpos, width*0.5, [CHARACTER_TYPE,chartype]);
    integer status = llList2Integer(path,-1);               // status is last value
    if (status != 0) 
    {   pathMsg(PATH_MSG_WARN, "Static path error: " + (string)status +  (string)startpos + " to " + (string)endpos + " orig startpos: " + (string)startposorig); 
        return(-1.0);                                       // fails
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
//
vector pathfindwalkable(vector startpos, float abovetol, float belowtol)
{   //  Look downward twice the height, because of seat mispositioning issues.
    ////list hits = llCastRay(startpos, startpos - <0,0,height*3>, 
    list hits = castray(startpos+<0,0,abovetol>, startpos-<0,0,belowtol>,      // look within allowed search range
            [RC_MAX_HITS,10, RC_REJECT_TYPES, RC_REJECT_PHYSICAL]); // go down up to 5 objs
    pathMsg(PATH_MSG_DEBUG,"Walkable hits looking down from " + (string)startpos + ": " + llDumpList2String(hits,",")); // ***TEMP***
    integer hitstatus = llList2Integer(hits,-1);        // < 0 is error
    if (hitstatus < 0)
    {   pathMsg(PATH_MSG_ERROR,"Error looking for walkable below " + (string)startpos + ", status " + (string)hitstatus); return(ZERO_VECTOR); }
    integer i;
    for (i=0; i<hitstatus*2; i = i + 2)                         // search hits
    {   key hitobj = llList2Key(hits,i);
        vector hitpt = llList2Vector(hits, i+1);
        if (hitobj == NULL_KEY) { return(hitpt); }               // found ground
        list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
        integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
        if (pathfindingtype == OPT_WALKABLE) { return(hitpt); } // found walkable
    }     
    return(ZERO_VECTOR);                                        // no find
}

//
//  pathptstowalkable -- move path points down to walkable surface
//
//  Used on the output of llGetStaticPath, which can be off by up to 0.35m.
//
list pathptstowalkable(list path)
{   list pts = [];
    integer length = llGetListLength(path);
    integer i;
    for (i=0; i<length; i++)                                    // for all points
    {   vector p = llList2Vector(path,i);
        vector pfloor = pathfindwalkable(p, PATHZTOL, PATHZTOL);// look near the 
        if (pfloor == ZERO_VECTOR)                              // can't find walkable surface
        {   pathMsg(PATH_MSG_WARN, "Can't find walkable below pnt " + (string)p);   // where's the ground or floor?
            pfloor = p;                                         // use unfixed value
        }
        pts += pfloor;                                          // add this point
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
    vector dir = llVecNorm(p1-p0);                          // direction for line
    rotation azimuthrot = llRotBetween(<1,0,0>, llVecNorm(<dir.x, dir.y, 0>));
    rotation elevrot = llRotBetween(llVecNorm(<dir.x, dir.y, 0>), dir); // elevation
    return(NormRot(azimuthrot * elevrot));                  // apply in proper order
}

//
//  mazecellto3d  -- convert maze cell to 3D coords
//
//  mazecellsize is the size of a cell in the XY plane, not the 3D plane
//  mazepos is the position of cell (0,0);
//  mazerot is the rotation of the maze plane.
//
//  Used in multiple scripts.
//
vector mazecellto3d(integer x, integer y, float mazecellsize, vector mazepos, rotation mazerot)
{
    if (x == 0 && y == 0) { return(mazepos); }      // avoid divide by zero
    vector vflat = <x*mazecellsize,y*mazecellsize,0.0>;   // vector to cell in XY plane
    //  Calc X and Y in 2D space.
    vector azimuthvec = <1,0,0>*mazerot;           // rotation 
    azimuthvec = llVecNorm(<azimuthvec.x, azimuthvec.y,0.0>);   
    rotation azimuthrot = NormRot(llRotBetween(<1,0,0>,azimuthvec));
    vector p = vflat*azimuthrot;             // vector in XY plane
    //  Vflatrot has correct X and Y. Now we need Z.
    vector planenormal = <0,0,1>*mazerot;          // normal to rotated plane. Plane is through origin here.
    //  Distance from point to plane is p*planenormal.  We want that to be zero.
    //  We want p.z such that p*planenormal = 0;
    //  want p.x*planenormal.x + p.y * planenormal.y + p.z * planenormal.z = 0
    p.z = - (p.x*planenormal.x + p.y * planenormal.y)/planenormal.z;    // planenormal.z cannot be zero unless tilted plane is vertical
    DEBUGPRINT1("mazecellto3d: x: " + (string)x + " y: " + (string)y + " p: " + (string)p + " p+mazepos: " + (string)(p+mazepos));
    return(p + mazepos);
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
//  Minimum value of probecnt is 2.
//
float castbeam(vector p0, vector p1, float width, float height, float probespacing, integer wantnearest, list castparams)
{   float yoffset;                                          // cast ray offset, Y dir in coords of vector
    float zoffset;                                          // cast ray offset, Z dir in coords of vector
    float nearestdist = INFINITY;                           // closest hit
    key ownkey = llGetKey();                                // get our own key
    ////DEBUGPRINT1("p0: " + (string)p0 + "  p1: " + (string)p1 + " probespacing: " + (string) probespacing);  // ***TEMP***
    integer probecount = (integer)((height-GROUNDCLEARANCE)/probespacing); // number of probes
    if (probecount < 1) { probecount = 1; }                 // minimum is one probe
    probespacing = (height-GROUNDCLEARANCE)/probecount;     // adjust to match height
    if (probespacing < 0.10) { return(-4); }                // Bad call
    vector dir = llVecNorm(p1-p0);                          // direction of raycast 
    vector endoffsetdir = <0,1,0>*rotperpenonground(p0,p1);    // offset for horizontal part of scan
    ////DEBUGPRINT1("End offset dir: " + (string)endoffsetdir);  // ***TEMP***
    //  Always do 3 scans across width - left edge, middle, right edge.
    for (yoffset = -width * 0.5; yoffset <= width * 0.5 + 0.001; yoffset += (width*0.5))
    {   for (zoffset = GROUNDCLEARANCE; zoffset <= height  + 0.001; zoffset += probespacing)
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
                {   key hitobj = llList2Key(castresult, i+0);       // get object hit
                    if (hitobj != ownkey)                           // ignore hits with self
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
//  NEEDS WORK to be usable from the maze solver on irregular terrain.
//
integer obstaclecheckpath(vector p0, vector p1, float width, float height, float probespacing, integer chartype)
{
    list path = llGetStaticPath(p0,p1,width*0.5, [CHARACTER_TYPE, chartype]);
    integer status = llList2Integer(path,-1);                   // last item is status
    path = llList2List(path,0,-2);                              // remove last item
    if (status != 0 || (llGetListLength(path) > 2 && !checkcollinear(path)))
    {   pathMsg(PATH_MSG_INFO,"Path static check found static obstacle between " + (string)p0 + " to " + (string)p1 + ": " + llDumpList2String(path,","));
        return(FALSE);
    }
    //  Don't test against land, because the static path check did that already.
    float disttohit = castbeam(p0, p1, width, height, probespacing, FALSE, PATHCASTRAYOPTS);
    if (disttohit != INFINITY)
    {   DEBUGPRINT1("Obstacle check path from " + (string)p0 + " " + (string)p1 + " hit at " + (string)(p0 + llVecNorm(p1-p0)*disttohit));
        return(FALSE);
    }
    return(TRUE);                                               // success
}
//
//  obstaclecheckcelloccupied  -- is there an obstacle in this cell?
//
//  Checks a cell centered on p1. Assumes the cell centered on p0 is clear. Alignment of cell is p1-p2.
//
//  This works by doing one ray cast straight down, and two straight up. The downward cast must hit a
//  wallkable. The upward casts must hit nothing. This catches big objects sitting on the walkable
//  surface.
//
//  We don't have to check the trailing edge of the cell, because we are coming from a clear cell at p0,
//  so that was already cheked.
//
//  No static path check, but it has to hit a walkable.
//
//  p0 and p1 must be one width apart. They are positions at ground level. Z values will be different on slopes.
//  
//  ***NEEDS WORK***
//
integer obstaclecheckcelloccupied(vector p0, vector p1, float width, float height, integer chartype, integer dobackcorners)
{
    if (gPathSelfObject == NULL_KEY)
    {   gPathSelfObject = llGetKey(); }                     // our own key, for later
#ifdef OBSOLETE
    //  Do static path check. We have to, or we will go through static obstacles which are vertical parts of walkables.
    list path = llGetStaticPath(p0,p1,width*0.5, [CHARACTER_TYPE, chartype]);
    integer status = llList2Integer(path,-1);                   // last item is status
    path = llList2List(path,0,-2);                              // remove last item
    if (status != 0 || (llGetListLength(path) > 2 && !checkcollinear(path)))
    {   pathMsg(PATH_MSG_INFO,"Path static check found static obstacle between " + (string)p0 + " to " + (string)p1 + ": " + llDumpList2String(path,","));
        return(TRUE);                                       // obstacle found
    }
#endif // OBSOLETE
    //  Static check OK, do ray casts.  
    float MAZEBELOWGNDTOL = 0.40;                           // cast downwards to just below ground
    vector dv = p1-p0;                                      // direction, unnormalized
    vector dvnorm = llVecNorm(dv);                          // 3D direction, normalized.
    float mazedepthmargin = 0.5*width*llFabs(dvnorm.z)+MAZEBELOWGNDTOL;   // allow for sloped area, cast deeper
    dv.z = 0;
    vector dir = llVecNorm(dv);                             // forward direction, XY plane
    p0 = p1 - dir*(width*1.5);                              // start casts from far side of previous cell
    vector crossdir = dir % <0,0,1>;                        // horizontal from ahead point
    DEBUGPRINT1("Cell edge check: dir = " + (string)dir + " crossdir: " + (string)crossdir + " p0: " + (string) p0 + " p1: " + (string)p1);
    vector pa = p1 + (crossdir*(width*0.5));                // one edge at ground level
    vector pb = p1 - (crossdir*(width*0.5));                // other edge at ground level
    vector pc = p1 + (dir*(width*0.5));                     // ahead at ground level
    vector pd = p1 - (dir*(width*0.5));                     // "behind" point
#ifdef OBSOLETE
 
    DEBUGPRINT1("Cell occupied check: " + (string)(p1+<0,0,height>) + " " + (string) (p1-<0,0,mazedepthmargin>)); // ***TEMP***
    list castresult = castray(p1+<0,0,height>, p1-<0,0,mazedepthmargin>,PATHCASTRAYOPTSOBS);    // probe center of cell, looking down
    if (!mazecasthitonlywalkable(castresult, FALSE)) { return(TRUE); }  // must hit walkable   
    //  Horizontal checks in forward direction to catch tall obstacles or thin ones.
    //  ***THESE NEED TO CONSIDER EVEN WALKABLES AS OBSTACLES***
    castresult = castray(p0+<0,0,height*0.5>,p1+dir*(width*0.5)+<0,0,height*0.5>,PATHCASTRAYOPTSOBS); // Horizontal cast at mid height, any non walkable hit is bad
    if (!mazecasthitonlywalkable(castresult, TRUE)) { return(TRUE); }  // if any non walkable hits, fail    
    castresult = castray(p0+<0,0,height*0.1>,p1+dir*(width*0.5)+<0,0,height*0.1>,PATHCASTRAYOPTSOBS); // Horizontal cast near ground level, any non walkable hit is bad
    if (!mazecasthitonlywalkable(castresult, TRUE)) { return(TRUE); }  // if any non walkable hits, fail    
    castresult = castray(p0+<0,0,height>,p1+dir*(width*0.5)+<0,0,height>,PATHCASTRAYOPTSOBS); // Horizontal cast at full height, any hit is bad
    if (!mazecasthitonlywalkable(castresult, TRUE)) { return(TRUE); }  // if any non walkable hits, fail    

    //  Crosswise horizontal check.
    castresult = castray(pa+<0,0,height*0.5>,pb+<0,0,height*0.5>,PATHCASTRAYOPTSOBS); // Horizontal cast, any hit is bad
    if (!mazecasthitonlywalkable(castresult, TRUE)) { return(TRUE); }  // if any non walkable hits, fail    
    //  Center of cell is clear and walkable. Now check upwards at front and side.
    //  The idea is to check at points that are on a circle of diameter "width"
    DEBUGPRINT1("Obstacle check if cell occupied. pa: " + (string)pa + " pb: " + (string)pb + " width: " + (string)width + " height: " + (string)height);     // ***TEMP***
    //  Downward ray casts only.  Must hit a walkable.   
    castresult = castray(pa+<0,0,height>,pa-<0,0,mazedepthmargin>,PATHCASTRAYOPTSOBS); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult, FALSE)) { return(TRUE); }// if any non-walkable hits, fail
    castresult = castray(pb+<0,0,height>,pb-<0,0,mazedepthmargin>,PATHCASTRAYOPTSOBS); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult, FALSE)) { return(TRUE); }    // if any non-walkable hits, fail
    castresult = castray(pc+<0,0,height>,pc-<0,0,mazedepthmargin>,PATHCASTRAYOPTSOBS); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult, FALSE)) { return(TRUE); }    // if any non-walkable hits, fail
    castresult = castray(pd+<0,0,height>,pc-<0,0,mazedepthmargin>,PATHCASTRAYOPTSOBS); // cast at steep angle, must hit walkable
    if (!mazecasthitonlywalkable(castresult, FALSE)) { return(TRUE); }    // if any non-walkable hits, fail
    if (!dobackcorners) 
    {   DEBUGPRINT1("Cell at " + (string)p1 + " empty.");           
        return(FALSE); 
    }
    //  Need to do all four corners of the square. Used when testing and not coming from a known good place.
    castresult = castray(pd+<0,0,height>,pd-<0,0,MAZEBELOWGNDTOL>,PATHCASTRAYOPTSOBS); // cast upwards, no land check
    if (!mazecasthitonlywalkable(castresult, TRUE)) { return(TRUE); }    // if any non-walkable hits, fail
    DEBUGPRINT1("Cell at " + (string)p1 + " empty."); 
#endif // OBSOLETE
    //  Initial basic downward cast.
    if (obstacleraycast1(p1+<0,0,height>, p1-<0,0,mazedepthmargin>)) { return(TRUE); }    // probe center of cell, looking down
    //  Horizontal casts.
    //  Horizontal checks in forward direction to catch tall obstacles or thin ones.
    //  ***THESE NEED TO CONSIDER EVEN WALKABLES AS OBSTACLES***
    //  ***WRONG??? - Supposed to be far side of next cell? p0 and p1 are same z value.  ****
    //  ***TEST POINTS MAY BE WRONG FOR SLOPES***
    if (obstacleraycast0(p0+<0,0,height*0.5>,p1+dir*(width*0.5)+<0,0,height*0.5>)) { return(TRUE); }// Horizontal cast at mid height, any non walkable hit is bad
#ifdef TEMPTURNOFF
    if (obstacleraycast0(p0+<0,0,height*0.1>,p1+dir*(width*0.5)+<0,0,height*0.1>)) { return(TRUE); }// Horizontal cast near ground level, any non walkable hit is bad
    if (obstacleraycast0(p0+<0,0,height>,p1+dir*(width*0.5)+<0,0,height>)) { return(TRUE); }   // Horizontal cast at full height, any hit is bad

    //  Crosswise horizontal check.
    if (obstacleraycast0(pa+<0,0,height*0.5>,pb+<0,0,height*0.5>)) { return(TRUE); }   // Horizontal cast, any hit is bad
#endif // TEMPTURNOFF
    //  Downward ray casts only.  Must hit a walkable.
    //  Center of cell is clear and walkable. Now check upwards at front and side.
    //  The idea is to check at points that are on a circle of diameter "width"
    if (obstacleraycast1(pa+<0,0,height>,pa-<0,0,mazedepthmargin>)) { return(TRUE); }   
    if (obstacleraycast1(pb+<0,0,height>,pb-<0,0,mazedepthmargin>)) { return(TRUE); } // cast downwards, must hit walkable
    if (obstacleraycast1(pc+<0,0,height>,pc-<0,0,mazedepthmargin>)) { return(TRUE); } // cast downwards, must hit walkable
    if (obstacleraycast1(pd+<0,0,height>,pc-<0,0,mazedepthmargin>)) { return(TRUE); } // cast at steep angle, must hit walkable
    if (!dobackcorners) 
    {   DEBUGPRINT1("Cell at " + (string)p1 + " empty.");           
        return(FALSE); 
    }
    //  Need to do all four corners of the square. Used when testing and not coming from a known good place.
    if (obstacleraycast1(pd+<0,0,height>,pd-<0,0,MAZEBELOWGNDTOL>)) { return(TRUE); }; // cast upwards, no land check 
    return(FALSE);                                               // success, no obstacle
}

//
//  obstacleraycast0 -- test for horizontal ray casts.  Must find open space.  Returns TRUE if obstacle.
//
//  ////***NEEDS WORK*** walkable is not acceptable.
//
integer obstacleraycast0(vector p0, vector p1)
{
    list castresult = castray(p0,p1,PATHCASTRAYOPTSOBS);        // Horizontal cast at full height, any hit is bad
    return(pathcastfoundproblem(castresult, FALSE, TRUE));      // if any hits at all, other than self, fail
}

//
//  obstacleraycast1 -- test for downward ray casts. Must find a walkable.  Returns TRUE if obstacle.
//
integer obstacleraycast1(vector p0, vector p1)
{
    list castresult = castray(p0,p1,PATHCASTRAYOPTSOBS);        // cast downwards, must hit walkable
    return(pathcastfoundproblem(castresult, TRUE, FALSE));      // if any non-walkable hits, fail
}
//
//  pathcastfoundproblem  -- true if cast ray hit a walkable, only. Used for downward casts
//
integer pathcastfoundproblem(list castresult, integer nohitval, integer walkableval)
{
    integer status = llList2Integer(castresult, -1);        // status is last element in list
#ifdef OBSOLETE
    if (status < 0)
    {   pathMsg(PATH_MSG_WARN,"Cast ray error status: " + (string)status);
        return(FALSE);                                      // fails, unlikely       
    }
#endif // OBSOLETE
    if (status == 0) { return(nohitval); }                  // hit nothing, use no hit value
    if (status < 0)  { return(TRUE); }                     // problem, fails
    //  Hit something. Must analyze.
    //  Hit ourself, ignore. 
    //  Hit land or walkable, return TRUE.
    //  Hit nothing but ignorables, return nohitval  
    integer i;
    for (i=0; i<2*status; i+=2)                             // check objects hit. Check two, because the first one might be ourself
    {
        key hitobj = llList2Key(castresult, i+0);           // get object hit
        if (hitobj == NULL_KEY) { return(walkableval); }           // land is walkable, so, OK
        if (hitobj != gPathSelfObject)                      // if hit something other than self.
        {   vector hitpt = llList2Vector(castresult, i+1);            // get point of hit
            list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // if it's not a walkable
            {   ////pathMsg(PATH_MSG_DEBUG,"Hit non-walkable " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) + " at " + (string)(hitpt));
                return(TRUE);                              // hit non-walkable, obstructed
            } 
            return(walkableval);                            // we hit a walkable - good, maybe
        }
    }
    return(nohitval);                                       // hit nothing of interest   
}
//
//  pathanalyzecastresult  -- analyze the result of a cast ray probe
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
list pathfindunobstructed(list pts, integer ix, integer fwd, float width, float height, integer chartype)
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
            if (!obstaclecheckcelloccupied(p0, pos, width, height, chartype, TRUE))
            {   DEBUGPRINT1("Found clear point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd); 
                return([pos,ix]);               // found an open spot
            }
            distalongseg += width;              // advance one width
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
    if (len == 0)  { return([]); }                          // empty list
    vector prevpt = llList2Vector(path,0);                  // previous point
    list newpath = [];                                      // which is the first output point
    integer i;
    for (i=1; i<len; i++)                                   // for all points after first
    {   vector pt = llList2Vector(path,i);                  // get next point
        float dist = llVecMag(pt - prevpt);                 // segment length
        if (dist > MINSEGMENTLENGTH)                        // if long enough to keep
        {   newpath += prevpt;
            prevpt = pt;
        }
    }
    newpath += llList2Vector(path,-1);                      // always include final point
    return(newpath);                                        // cleaned up path
}
//
//  pathtrimmedstaticpath -- trim off end of path by specified distance
//
//  Used to make Pursue stop before hitting the target
//
list pathtrimmedstaticpath(vector startpos, vector endpos, float stopshort, float width, integer chartype)
{   list path = llGetStaticPath(startpos, endpos, width*0.5, [CHARACTER_TYPE,chartype]);    // plan a static path
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

