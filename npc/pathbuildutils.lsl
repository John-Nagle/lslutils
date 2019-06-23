//
//   pathbuildutils.lsl -- components of a path building system
//
//   Part of a system for doing pathfinding in Second Life
//
//   Animats
//   June, 2019
//
//
//  Constants
//
integer CASTRAYRETRIES = 10;                                // retry up to 10 times
float CASTRAYRETRYDELAY = 0.200;                            // if a problem, retry slowly
#define INFINITY ((float)"inf")                             // is there a better way?
float GROUNDCLEARANCE = 0.05;                               // avoid false ground collisions

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
    {   castresult = llCastRay(p0, p1, params);             // try cast ray
        if (llList2Integer(castresult,-1) >= 0)             // if good status
        {   return(castresult); }                           // non-error, return
        llSleep(CASTRAYRETRYDELAY);                         // error, wait and retry
    }
    //  Too many retries, give up
    return(castresult);                                     // return final failure
}
//
//  castbeam -- do multiple llCastRay operations to check if way is clear for movement
//
//  ***TEMP*** no Y scan.
//
//  Returns distance, or INFINITY if no obstacle found, or negative of status code if error.
//  Ignores hits with walkable objects, so this is for horizontal scans only.
//
//  Minimum value of probecnt is 2.
//
float castbeam(vector p0, vector p1, float width, float height, float probecnt, integer wantnearest)
{
    float zoffset;
    float nearestdist = INFINITY;
    if (probecnt < 2) { return(-4); }                       // Bad call
    ////for (yoffset = -width * 0.5; yoffset <= width * 0.5 + 0.001; yoffset += width/subdivide)
    vector dir = llVecNorm(p1-p0);                          // direction of raycast                     
    {   for (zoffset = GROUNDCLEARANCE; zoffset <= height  + 0.001; zoffset += (height - GROUNDCLEARANCE)/(probecnt-1))
        {   ////llOwnerSay("p0: " + (string)p0 + "  p1: " + (string)p1 + "  zoffset: " + (string)zoffset); // ***TEMP***
            list castresult = castray(<p0.x, p0.y, p0.z+zoffset>, <p1.x, p1.y, p1.z + zoffset>, []);
            integer status = llList2Integer(castresult, -1);// status is last element in list
            if (status < 0)
            {   llOwnerSay("Cast ray status: " + (string)status);
                return((integer)status);                    // fails       
            }
            if (status > 0) { 
                vector hitpt = llList2Vector(castresult, 1); // get point of hit
                key hitobj = llList2Key(castresult, 0);     // get object hit
                list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
                integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
                if (pathfindingtype != OPT_WALKABLE)        // if it's not a walkable
                {   float dist = (hitpt-p0) * dir;          // distance to hit
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

