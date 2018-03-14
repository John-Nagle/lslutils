//
//  Vehicle utilities
//
//  Animats
//  March 2019
//
//  Utility functions
float min(float a, float b)
{   if (a < b) { return(a); } else { return(b); }}
float max(float a, float b)
{   if (a > b) { return(a); } else { return(b); }}
float abs(float a)
{   if (a > 0) { return(a); } else { return(-a); }}

//
//  nearestcorner -- nearest region corner to position
//
//  Will need revision for systems with multiple region sizes
//
vector nearestcorner(vector pos)
{   vector corner = <0,0,0>;
    if (pos.x*2.0 > REGION_SIZE) corner.x = REGION_SIZE;
    if (pos.y*2.0 > REGION_SIZE) corner.y = REGION_SIZE;
    return(corner);
}

integer outsideregion(vector pos)                       // TRUE if outside region
{   return(pos.x < 0.0 || pos.x > REGION_SIZE || pos.y < 0.0 || pos.y > REGION_SIZE); }

//
//  regioncrossingseglength -- length of segment between next two region crossings in dir
//
//  Ignores Z direction.  
//  There is a diagram of this in regioncrossing.md.
//
float regioncrossingseglength(vector pos, vector dir)
{
    vector corner = nearestcorner(pos);             // nearest region corner
    vector dp = pos - corner;                       // work in coord system where corner is <0,0>
    float xcept = HUGE;                             // X intercept
    float ycept = HUGE;                             // Y intercept
    if (abs(dir.x) > TINY)                          // avoid divide by zero
    {   ycept = dp.y + dir.y * (-dp.x / dir.x); }   // Y intercept
    if (abs(dir.y) > TINY)
    {   xcept = dp.x + dir.x * (-dp.y / dir.y); }   // X intercept
    return(llSqrt(xcept*xcept + ycept*ycept));      // return segment distance   
}

//
//  regioncrossingprobe  --  returns length of segment between closest two next region crossings
//
//  The test is made along two test vectors some angle apart
//  Returns HUGE if not meaningful.
//  ***UNUSED*** at present
//
float regioncrossingprobe(vector pos, vector dir, rotation testangle1, rotation testangle2)
{
    vector v1 = dir * testangle1;                   // need two test vectors
    vector v2 = dir * testangle2;                   // 
    return(min(regioncrossingseglength(pos, v1), regioncrossingseglength(pos, v2))); // closest hit
}


//  Where will a 2D XY vector from p in direction v hit a box of [0..boxsize, 0..boxsize]?
//  Returns distance. Hit point is pos + distance*norm(dir)
float line_box_intersection(vector pos, vector dir, float boxsize)
{   dir.z = 0;
    pos.z = 0;
    dir = llVecNorm(dir);
    //  p = pos + dir * dist;               p is a point on the edge
    float dist1 = HUGE;                     // distance to X bound
    float dist2 = HUGE;                     // distance to Y bound
    //  Check Y axis edges 
    //  p1.x = pos.x + dir.x * dist
    if (dir.x < -TINY)                      // will hit on 0 edge, p1.x = 0
    {   dist1 = -pos.x/dir.x; }             // distance to x=0 edge        
    else if (dir.x > TINY)                // will hit on boxsize edge, p1.x = boxsize
    {   dist1 = (-(pos.x - boxsize))/dir.x; }   // boxsize = pos.x + dir.x * dist1
    //  Check X axis edges
    if (dir.y < -TINY)
    {   dist2 = -pos.y/dir.y; }
    else if (dir.y > TINY)
    {   dist2 = (-(pos.y - boxsize))/dir.y; }
    if (dist1 < dist2) { return(dist1);}
    return(dist2);                  // return minimum distance       
}
//
//  slowforregioncross -- do we need to slow down for a region crossing?
//
float maxspeedforregioncross(vector pos, vector vel)
{  
    float maxspeed = HUGE;                              // assume no speed limit
    vel.z = 0;                                          // XY plane only
    float speed = llVecMag(vel);                        // how fast are we going
    if (speed < TINY) { return(HUGE); }                 // slow enough that not a problem
    float disttosimcross = line_box_intersection(pos, vel, REGION_SIZE);    // dist to sim crossing
    float timetoedge = disttosimcross / speed;                    // speed to edge
    ////if (((TimerTick % 10) == 0) && (timetoedge < 1.5)) // ***TEMP DEBUG***
    ////{   llOwnerSay((string)disttosimcross + "m to region boundary at " + (string)pos + "  " + (string)speed + "m/sec."); } // ***TEMP***
    if (timetoedge < TIMER_INTERVAL*3.0)                // if very close to sim crossing and time to brake
    {   float rcseglength = regioncrossingseglength(pos, vel); // distance between next two sim crosses
        //  slow to prevent too-fast double region cross
        maxspeed = max(rcseglength / MIN_SAFE_DOUBLE_REGION_CROSS_TIME, MIN_BRAKE_SPEED); 
        ////if (maxspeed < speed)
        ////{   llOwnerSay("Region crossing braking: segment length: " + (string)rcseglength + 
        ////   "  speed limit: " + (string)maxspeed);  }// ***TEMP***
    }
    return(maxspeed);
}
//  posasstring -- get current position as string
string posasstring(string region, vector pos)
{   
    return(region
        + " (" + (string)((integer)pos.x)
        + "," + (string)((integer)pos.y)
        + "," + (string)((integer)pos.z) + ")");
}
//
//  ifnotseated  - true if avatar position is valid
//
//  Check for everything which can go wrong with the vehicle/avatar
//  relationship.
//
integer ifnotseated(key avatar)
{   integer trouble = FALSE;
    //  Check for avatar out of position
    if (avatar != NULL_KEY) 
    {   vector vehpos = llGetPos();
        list avatarinfo = llGetObjectDetails(avatar, 
                [OBJECT_POS, OBJECT_ROOT]);
        vector avatarpos = llList2Vector(avatarinfo,0);
        key avatarroot = llList2Key(avatarinfo,1);
        float avatardist = llVecMag(avatarpos - vehpos);
        if (avatardist > AVATAR_MAX_DIST_TO_BIKE)
            {   trouble = TRUE;
                llOwnerSay("Avatar out of position: " + 
                    (string)avatardist + "m from bike.");
            }
        integer agentinfo = llGetAgentInfo(avatar);
        if (agentinfo & (AGENT_SITTING | AGENT_ON_OBJECT) !=
            (AGENT_SITTING | AGENT_ON_OBJECT))
        {   trouble = TRUE;
            llOwnerSay("Avatar not fully seated.");
        }
        //  Check back link from avatar to root prim.
        if (avatarroot == NULL_KEY)
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is null.");
        }
        else if (avatarroot == avatar)
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is to avatar itself.");
        }
        else if (avatarroot != llGetKey())
        {   trouble = TRUE;
            llOwnerSay("Avatar link to root is wrong."); 
            llOwnerSay("Avatar link to root: " + 
                (string) avatarroot + "  Veh. root: " +
                (string) llGetKey());
        }
    } else {                    // unseated
        trouble = TRUE;
        llOwnerSay("Avatar not on sit target.");
    }
    return trouble;
} 
