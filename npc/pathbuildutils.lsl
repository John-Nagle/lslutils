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
//
//  Constants
//
integer CASTRAYRETRIES = 10;                                // retry up to 10 times
float CASTRAYRETRYDELAY = 0.200;                            // if a problem, retry slowly
float GROUNDCLEARANCE = 0.05;                               // avoid false ground collisions
float MAXAVOIDMOVE = 8.0;                                   // max distance to avoid obstacle
float PATHCHECKTOL = 0.02;                                  // allow 2cm collinearity error
#ifndef INFINITY                                            // should be an LSL builtin
#define INFINITY ((float)"inf")                             // is there a better way?
#endif // INFINITY

//
//  
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
        if (dist > PATHCHECKTOL) { return(FALSE); } // tolerance 1cm
    }
    return(TRUE);                       // collinear   
}
//
//  pathstraighten -- straighten a path if possible.
//
//  Works in 3D world, not maze cell space.
//
list pathstraighten(list pts, float width, float height, float probespacing, integer chartype)
{
    integer n = 0;
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
//  rotperpenonground  -- rotation to get line on ground perpendicular to vector
//
//  Used to construct the end lines of path segments.
//
rotation rotperpenonground(vector p0, vector p1)
{
    vector dir = llVecNorm(p1-p0);                          // direction for line
    rotation azimuthrot = llRotBetween(<1,0,0>, llVecNorm(<dir.x, dir.y, 0>));
    rotation elevrot = llRotBetween(llVecNorm(<dir.x, dir.y, 0>), dir); // elevation
    return(azimuthrot * elevrot);                           // apply in proper order
}
//
//  castray -- llCastRay with retry
//
//  llCastRay can fail under overload. This is rare, but we have to retry.
//
list castray(vector p0, vector p1, list params)
{
    ////DEBUGPRINT1("Cast ray: " + (string)p0 + " " + (string)p1);   // ***TEMP***
    integer tries = CASTRAYRETRIES;                         // number of retries
    list castresult = [];
    while (tries-- > 0)
    {   DEBUGPRINT1("Cast ray: p0: " + (string)p0 + "  p1: " + (string)p1);  // ***TEMP*** 
        castresult = llCastRay(p0, p1, params);             // try cast ray
        DEBUGPRINT1("Cast result: " + llDumpList2String(castresult,","));    // ***TEMP***
        if (llList2Integer(castresult,-1) >= 0)             // if good status
        {   return(castresult); }                           // non-error, return
        DEBUGPRINT1("Cast delayed: " + (string) llList2Integer(castresult,-1));  // ***TEMP***
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
            {   DEBUGPRINT1("Cast ray status: " + (string)status);
                return((integer)status);                    // fails       
            }
            if (status > 0) { 
                vector hitpt = llList2Vector(castresult, 1); // get point of hit
                key hitobj = llList2Key(castresult, 0);     // get object hit
                list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
                integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
                if (pathfindingtype != OPT_WALKABLE)        // if it's not a walkable
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
    list path = llGetStaticPath(p0,p1,width*0.5, [CHARACTER_TYPE, CHARTYPE]);
    integer status = llList2Integer(path,-1);                   // last item is status
    path = llList2List(path,0,-2);                              // remove last item
    if (status != 0 || llGetListLength(path) > 2 && !checkcollinear(path))
    {   DEBUGPRINT1("Path static check failed for " + (string)p0 + " to " + (string)p1 + ": " + llDumpList2String(path,","));
        return(FALSE);
    }
    //  Don't test against land, because the static path check did that already.
    float disttohit = castbeam(p0, p1, width, height, probespacing, FALSE, [RC_REJECT_TYPES,RC_REJECT_LAND]);
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
//  p0 and p1 must be one width apart. 
//
//  
//
integer obstaclecheckcelloccupied(vector p0, vector p1, float width, float height, integer dobackcorners)
{
    float MAZEBELOWGNDTOL = 0.20;                           // cast downwards to just below ground
    vector dv = p1-p0;                                      // direction, unnormalized
    float mazedepthmargin = llFabs(dv.z)+MAZEBELOWGNDTOL;   // allow for sloped area, cast deeper
    dv.z = 0;
    vector dir = llVecNorm(dv);                             // forward direction, XY plane
    p0 = p1 - dir*(width*1.5);                              // start casts from far side of previous cell
    vector crossdir = dir % <0,0,1>;                        // horizontal from ahead point
    DEBUGPRINT1("Cell edge check: dir = " + (string)dir + " crossdir: " + (string)crossdir + " p0: " + (string) p0 + " p1: " + (string)p1);
    vector pa = p1 + (crossdir*(width*0.5));                // one edge at ground level
    vector pb = p1 - (crossdir*(width*0.5));                // other edge at ground level
    vector pc = p1 + (dir*(width*0.5));                     // ahead at ground level
    vector pd = p1 - (dir*(width*0.5));                     // "behind" point 
    DEBUGPRINT1("Cell occupied check: " + (string)(p1+<0,0,height>) + " " + (string) (p1-<0,0,mazedepthmargin>)); // ***TEMP***
    list castresult = castray(p1+<0,0,height>, p1-<0,0,mazedepthmargin>,[]);    // probe center of cell, looking down
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); } // cell is occupied
    //  Horizontal checks in forward direction to catch tall obstacles or thin ones.
    castresult = castray(p0+<0,0,height*0.5>,p1+dir*(width*0.5)+<0,0,height*0.5>,[]); // Horizontal cast at mid height, any hit is bad
    if (mazecasthitanything(castresult)) { return(TRUE); }  // if any hits, fail
    castresult = castray(p0+<0,0,mazedepthmargin>,p1+dir*(width*0.5)+<0,0,GROUNDCLEARANCE>,[]); // Horizontal cast at ground level, any hit is bad
    if (mazecasthitanything(castresult)) { return(TRUE); }  // if any hits, fail
    castresult = castray(p0+<0,0,height>,p1+dir*(width*0.5)+<0,0,height>,[]); // Horizontal cast at mid height, any hit is bad
    if (mazecasthitanything(castresult)) { return(TRUE); }  // if any hits, fail

    //  Crosswise horizontal check.
    castresult = castray(pa+<0,0,height*0.5>,pb+<0,0,height*0.5>,[]); // Horizontal cast, any hit is bad
    if (mazecasthitanything(castresult)) { return(TRUE); }  // if any hits, fail    
    //  Center of cell is clear and walkable. Now check upwards at front and side.
    //  The idea is to check at points that are on a circle of diameter "width"
    DEBUGPRINT1("Obstacle check if cell occupied. pa: " + (string)pa + " pb: " + (string)pb + " width: " + (string)width + " height: " + (string)height);     // ***TEMP***
    //  Downward ray casts only.  Must hit a walkable.   
    castresult = castray(pa+<0,0,height>,pa-<0,0,mazedepthmargin>,[]); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); }// if any non-walkable hits, fail
    castresult = castray(pb+<0,0,height>,pb-<0,0,mazedepthmargin>,[]); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); }    // if any non-walkable hits, fail
    castresult = castray(pc+<0,0,height>,pc-<0,0,mazedepthmargin>,[]); // cast downwards, must hit walkable
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); }    // if any non-walkable hits, fail
    castresult = castray(pd+<0,0,height>,pc-<0,0,mazedepthmargin>,[]); // cast at steep angle, must hit walkable
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); }    // if any non-walkable hits, fail
    if (!dobackcorners) 
    {   DEBUGPRINT1("Cell at " + (string)p1 + " empty.");           
        return(FALSE); 
    }
    //  Need to do all four corners of the square. Used when testing and not coming from a known good place.
    castresult = castray(pd+<0,0,height>,pd-<0,0,MAZEBELOWGNDTOL>,[]); // cast upwards, no land check
    if (!mazecasthitonlywalkable(castresult)) { return(TRUE); }    // if any non-walkable hits, fail
    DEBUGPRINT1("Cell at " + (string)p1 + " empty.");           
    return(FALSE);                                               // success, no obstacle
}
//
//  mazecasthitonlywalkable  -- true if cast ray hit a walkable, only. Used for downward casts
//
integer mazecasthitonlywalkable(list castresult)
{
    integer status = llList2Integer(castresult, -1);        // status is last element in list
    if (status < 0)
    {   DEBUGPRINT1("Cast ray error status: " + (string)status);
        return(FALSE);                                      // fails, unlikely       
    }
    if (status != 1) { return(FALSE); }                     // hit nothing, fails
    vector hitpt = llList2Vector(castresult, 1);            // get point of hit
    key hitobj = llList2Key(castresult, 0);                 // get object hit
    if (hitobj == NULL_KEY) { return(TRUE); }               // null key is land, that's OK
    list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
    integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
    if (pathfindingtype != OPT_WALKABLE)                    // if it's not a walkable
    {   DEBUGPRINT1("Hit non-walkable " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) + " at " + (string)(hitpt));                // ***TEMP***
        return(FALSE);                                      // hit non-walkable, obstructed
    }
    return(TRUE);                                           // hit only a walkable - good.    
}
//
//  mazecasthitanything  -- true if cast ray hit a walkable, only. Used for downward casts
//
integer mazecasthitanything(list castresult)
{
    integer status = llList2Integer(castresult, -1);        // status is last element in list
    if (status == 0) { return(FALSE); }                     // hit nothing, good
    return(TRUE);                                           // hit something, bad    
}
//
//  pathfindunobstructed -- find an unobstructed point near a path segment end.
//
//  Returns [pnt,ix], where ix is the point index previous, in the direction of scan, to the
//  scan, of the point found.
//
//  This is inefficient but seldom used.
//
list pathfindunobstructed(list pts, integer ix, integer fwd, float width, float height)
{
    ////assert(fwd == 1 || fwd == -1);
    DEBUGPRINT1("Looking for unobstructed point on segment #" + (string)ix + " fwd " + (string)fwd);  
    integer length = llGetListLength(pts);
    float distalongseg = width;                   // distance along segment starting at ix.
    while (TRUE)                                // until return
    {   integer ix2 = ix + fwd;                 // next index in desired direction
        vector p0 = llList2Vector(pts,ix);      // index of previous point
        vector p1 = llList2Vector(pts,ix2);     // index of next point
        vector pos = p0 + llVecNorm(p1-p0) * distalongseg; // next point to try
        float vlen = llVecMag(p1-p0);           // length of vector
        if (distalongseg > vlen)                // if hit end of segment
        {   ix = ix + fwd;                      // advance one seg in desired dir
            if (ix + fwd >= length || ix + fwd < 0) // end of entire path without find
            {   DEBUGPRINT1("Fail: no unobstructed point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd);  
                return([ZERO_VECTOR,-1]);          // hit end of path without find, fails
            }
            distalongseg = width;                     // start working next segment
        } else {
            DEBUGPRINT1("Trying point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd);  
            if (!obstaclecheckcelloccupied(p0, pos, width, height, TRUE))
            {   DEBUGPRINT("Found unobstructed point on segment #" + (string)ix + " at " + (string)pos + " fwd " + (string)fwd); 
                return([pos,ix]);                  // found an open spot
            }
            distalongseg += width;              // advance to next spot to try
        }
    }
    //  Unreachable
    return([ZERO_VECTOR,-1]);                   // no way
}
//
//  pathendpointadjust -- make sure the endpoint of each path segment is clear.
//
//  The next phase of processing requires this. We can only do a maze solve if
//  we have clear start and end points.
//
//  As a special case, it's OK if we can't reach the endpoint. We will shorten the last
//  segment if necessary. That's so we can send multiple NPCs to the same destination
//  and have them end up as a crowd, rather than all but one failing.
//
list pathendpointadjust(list pts, float width, float height)
{
    integer n;
    if (llGetListLength(pts) == 0) { return(pts); } // empty
    for (n=1; n<llGetListLength(pts); n++)          // check all points except start
    {   vector pos = llList2Vector(pts,n);
        vector prevpos = llList2Vector(pts,n-1);                                // previous point
        if (obstaclecheckcelloccupied(prevpos, pos, width, height, TRUE))       // if obstacle at endpoint
        {   DEBUGPRINT1("Path segment endpoint #" + (string)n + " obstructed at " + (string)pos);   // ***TEMP***
            //  We are going to have to move an endpoint.
            //  Find unobstructed points in the previous and next segments.
            //  Replace obstructed point with those two points.
            list revresult = pathfindunobstructed(pts,n,-1, width, height);     // search backwards
            vector revpt = llList2Vector(revresult,0);                          // new clear point in reverse dir
            integer revix = llList2Integer(revresult,1);                        // index of point after find pt
            if (n == llGetListLength(pts)-1)                                                  // if last point, special case
            {   if (revix <= 0)
                {   DEBUGPRINT1("Cannot find unobstructed end point anywhere near " + (string)pos); // ***TEMP***
                    return([]);                                                     // fails
                }
                DEBUGPRINT1("Replaced endpoint " + (string)pos + " with " + (string)revpt);    // ***TEMP***
                pts = llListReplaceList(pts,[revpt],n,n);                       // replace endpoint
                return(pts);                                                    // result
            
            }
            list fwdresult = pathfindunobstructed(pts,n, 1, width, height);     // search forwards
            vector fwdpt = llList2Vector(fwdresult,0);                          // new clear point in forward dir
            integer fwdix = llList2Integer(fwdresult,1);                        // index of point before find pt.
            if (fwdix <= 0 || revix <= 0)
            {   DEBUGPRINT1("Cannot find unobstructed points anywhere near " + (string)pos); // ***TEMP***
                return([]);                                                     // fails
            }
            //  We now have two unobstructed points, fwdpt and revpt. Those will be connected, and will replace
            //  the points between them. 
            //  Drop points from n through fwdix, and replace with new fwd point. 
            //  If fwdix is off the end, the new point just replaces it.
            DEBUGPRINT1("At pt #" + (string)n + ", replacing points [" + (string)revix + ".." + (string)fwdix + "] at " + (string)pos + " with " + (string)revpt + " and " + (string)fwdpt);    // ***TEMP***
            DEBUGPRINT1("List before update: " + llDumpList2String(pts,", "));    
            pts = llListReplaceList(pts, [revpt, fwdpt], revix, fwdix);             // replace points revix through fwdix inclusive
            ////pts = llListReplaceList(pts, [fwdpt], n, fwdix);                    // replace point ahead
            ////pts = llListReplaceList(pts, [revpt], revix-1, n-1);                // replace point behind - CHECK THIS
            DEBUGPRINT1("List after update: " + llDumpList2String(pts,", ")); 
            //  ***NEED TO UPDATE N***   

        }   
    }
    return(pts);
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
    {   vector pt = llList2Vector(path,i);                  // get next pont
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
//  pathcheckobstacles -- check a path for obstacles.
//
//  The points on the incoming path should be unobstructed.
//
//  Output is a strided list of the form [pnt, blocked, pnt, blocked ...]
//
//  ***NOTYET***
//  ***PROBABLY SHOULD TAKE STRIDED LIST AS INPUT WITH KNOWN OBSTRUCTIONS LISTED***
//
list pathcheckobstacles(list pts)
{   
    vector initialpos = llGetPos();                         // starting position
    list pathPoints = [];
    integer i;
    integer len = llGetListLength(pts);
    vector prevpos = ZERO_VECTOR;                           // other end of 
    for (i=0; i< len; i++)
    {   vector pos = llList2Vector(pts, i);                 // nth point
        //  Place line
        if (prevpos != ZERO_VECTOR)                         // if not initial point
        {
            //  Check segment for obstacles
            float fulllength = llVecMag(pos-prevpos);               // full segment length
            vector dir = llVecNorm(pos-prevpos);                    // direction of segment
            float hitdist = castbeam(prevpos, pos, CHARRADIUS*2, CHARHEIGHT, TESTSPACING, TRUE,
                    [RC_REJECT_TYPES,RC_REJECT_LAND]);
            if (hitdist < 0)
            {   llSay(DEBUG_CHANNEL,"ERROR: Cast beam error " + (string)hitdist);
                return([]);                                          // failure
            }
            if (hitdist != INFINITY)                                    // if meaningful hit distance
            {   llOwnerSay("Hit obstacle at " + (string) (prevpos + dir * hitdist)); 
                //  Need to subdivide vector into 3 vectors - 2 clear ones, and one obstructed one in the middle.
                float prevposdist = hitdist;                            // begin to first hit
                //  Search from other end. Other endpoint should be unobstructed.
                //  That was fixed at a previous step.
                //  Is reverse scan start point obstructed?
                //  So we are raycasting from open space.
                float posdist = castbeam(pos, prevpos, CHARRADIUS*2, CHARHEIGHT, TESTSPACING, TRUE,
                    [RC_REJECT_TYPES,RC_REJECT_LAND]);
                //  Check for strange cases. Cell occupied and horizontal beam cast disagree. 
                //  If not obstructed both ways, assume clear. Probably some tiny bump.
                if (posdist != INFINITY)                        // obstacle both ways
                {    //  Back off, looking for open space
                    integer noopenspace = TRUE;                 // no open space found yet
                    vector interpt0 = dir * prevposdist + prevpos;          // first intermediate point
                    while (noopenspace && prevposdist > 0)
                    {   interpt0 = dir * prevposdist + prevpos; 
                        // Check for an open cell. Search backwards from hit point.
                        if (obstaclecheckcelloccupied(prevpos, interpt0, CHARRADIUS*2, CHARHEIGHT, TRUE))
                        {   prevposdist = prevposdist - CHARRADIUS; // no open cell, back off
                            if (prevposdist < 0) { prevposdist = 0; }
                        } else { 
                            noopenspace = FALSE;                    // found a good open cell
                        }
                    }
                    if (noopenspace)                                // if could not find open space
                    {   llOwnerSay("Can't find open space between " + (string)prevpos +  " and " + (string)pos); 
                        return([]);                              // fails
                    }
                    //  Now do a similar backoff on the far side of the obstacle                
                    vector interpt1 = -dir * posdist + pos; 
                    noopenspace = TRUE;                             // no open space found yet
                    while (noopenspace && posdist > 0)
                    {   interpt1 = -dir * posdist + pos;            // search forwards from hit pt
                        // Check for an open cell.
                        if (obstaclecheckcelloccupied(prevpos, interpt1, CHARRADIUS*2, CHARHEIGHT, TRUE))
                        {   llOwnerSay("No open cell at " + (string)interpt1); // ***TEMP***
                            posdist = posdist - CHARRADIUS;         // no open cell, back off
                            if (posdist < 0) { posdist = 0; }
                        } else { 
                            noopenspace = FALSE;                    // found a good open cell
                        }
                    }
                    if (noopenspace)                                // if could not find open space
                    {   llOwnerSay("Can't find open space in reverse between " + (string)pos +  " and " + (string)prevpos); 
                        return([]);                              // fails
                    }
                    if (fulllength - (prevposdist + posdist) < 0.005)           // if nothing in the middle
                    {   llOwnerSay("Segment from " + (string)prevpos + " to " + (string)pos + " with obstruction at " +
                            (string)(prevpos + dir * hitdist) + "has inconsistent hits when tested from both ends.");
                        //  2 segment case, with second one bad
                        pathPoints += [prevpos, TRUE, interpt0, FALSE];
                    } else {
                       // 3 segment case, with obstruction in middle
                        pathPoints += [prevpos, TRUE, interpt0, FALSE, interpt1, TRUE];
                    }
                } else {                                                        // no hit in reverse dir
                    pathPoints += [prevpos, TRUE];                              // assume no hit, space clear. May be overoptimistic                  
                }
            } else {                                                            // no obstacle
                //  Draw marker for segment
                pathPoints += [prevpos, TRUE];                              // assume no hit                   
            }
        }
        //  On to next point
        prevpos = pos;
    }
    pathPoints += [llList2Vector(pts,-1), llList2Integer(pathPoints,-1)];         // use status from last segment as dummy.
    return(pathPoints);
}
#endif // PATHBUILDUTILSLSL

