//
//  bhvavoid.lsl -- avoid incoming threats
//
//  For now, moving vehicles only
//
//  Very close to stack overflow here.
//
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/bhv/bhvcall.lsl"
#include "npc/pathbuildutils.lsl"

integer ACTION_IDLE = 0;                    // not doing anything
integer ACTION_AVOIDING = 1;                // trying to avoid something
integer ACTION_DWELL = 2;                   // dwell time after an avoid


//  Configuration

#define PRIORITY_AVOID  PRIORITY_URGENT     // higher than most behaviors

#define DETECTION_RADIUS 50.0               // look for moving vehicles this range
#define AVOID_DIST 4.0                      // (m) move this far each try to get out of the way
#define REACT_DIST 3.0                      // (m) this close and get away from it
#define SENSE_DIST 10.0                     // (m) this close and poll fast
#define IDLE_SENSOR_PERIOD 2.0              // (s) sensor poll rate when no nearby targets
#define MIN_SENSOR_PERIOD 0.2               // (s) sensor poll rate max
#define CLOSE_SENSOR_PERIOD 0.5             // (s) sensor poll rate when close to threat

#ifndef CHARACTER_SPEED                     // overrideable
#define CHARACTER_SPEED  2.5                // (m/sec) speed
#endif // CHARACTER_SPEED
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
#define CHARACTER_RUN_SPEED 5.5             // (m/sec) top speed run
string IDLE_ANIM = "stand";                 // holding after evasion
#define IDLE_POLL 10.0                      // for stall timer tick if used
#define SAFE_IDLE_TIME 20.0                 // idle this long, stop polling fast
#define DWELL_TIME 20.0                     // keep control in avoid until this time runs out

//
//  Avoid directions - try escaping in these directions.               
//
//  All must be vectors of length 1.
//  If we have to use the last one, trouble.
//

#define DIRTRIES 5                          // number of directions to try when avoiding

#define MAX_AVOID_TIME 6.0                  // (s) trigger avoid when ETA less than this
#define MAX_AVOID_TRIES 10                  // (cnt) after this many tries to avoid, give up
#define PROTECTIVE_HEIGHT 0.8               // (m) obstacle height that will stop an theat
////#define MAX_STATIONARY_TIME 15.0            // (s) we are stationary if did not move for this long

#define getroot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))  


//  Global variables
integer gAction = ACTION_IDLE;
list gThreatList = [];                      // list of incoming threats
integer gAvoidIndex = 0;                    // which avoid path we are taking
vector gAvoidStart = ZERO_VECTOR;           // where we started avoiding
float gSensorPeriod = 0.0;                  // Current sensor poll rate
integer gLastThreatTime = 0;                // time last threat seen
integer gNextAvoid = 0;                     // next entry in AVOIDDIRS to use
integer gAvoidTries = 0;                    // no avoid tries yet
vector gLastSelfPos;                        // last position of NPC, for move check
integer gLastSelfMovetime;                  // last time NPC moved
integer gLastAvoidtime;                     // last time avoid action taken

//
//  setsensortime -- set current sensor poll rate
//
setsensortime(float secs)
{   if (secs < MIN_SENSOR_PERIOD) { secs = MIN_SENSOR_PERIOD; } // apply minimum
    if (secs > IDLE_SENSOR_PERIOD) { secs = IDLE_SENSOR_PERIOD; } // apply max
    if (secs == gSensorPeriod) { return; }    // no change
    debugMsg(DEBUG_MSG_WARN,"Sensor period now " + (string)secs);   // ***TEMP*** as warn
    llSensorRepeat("", "", AGENT, DETECTION_RADIUS, PI, secs);
    gSensorPeriod = secs;                     // save for later
}

//
//  invehicle -- true if avatar in active vehicle
//
integer invehicle(key id)
{   key rootid = getroot(id);                                   // avi if not setting, else sit root
    if (id == rootid) { return(FALSE); }                        // not sitting
    list objinfo = llGetObjectDetails(rootid, [OBJECT_PHYSICS]);  // get physics status of sitting object
    if (llGetListLength(objinfo) < 1) { return(FALSE); }        // failed to get velocity
    return(llList2Integer(objinfo,0));                          // in vehicle if sitting and physics on
}

//
//  findclosestpoint -- find closest point on threat object bounding box.
//
//  Simple version - closest point is always a corner. 
//  Not going to implement GJK algorithm in LSL for this.
//
vector findclosestpoint(key id, vector pos, list bb)
{   vector bblower = llList2Vector(bb,0);       // these define an axis aligned box with 8 corners.
    vector bbupper = llList2Vector(bb,1);
    //  Find the corner closest to pos.
    vector closestpt = bblower;
    if (llFabs(bbupper.x - pos.x) < llFabs(bblower.x - pos.x)) { closestpt.x = bbupper.x; }
    if (llFabs(bbupper.y - pos.y) < llFabs(bblower.y - pos.y)) { closestpt.y = bbupper.y; }
    if (llFabs(bbupper.z - pos.z) < llFabs(bblower.x - pos.z)) { closestpt.z = bbupper.z; }
    return(closestpt);
}
//
//  evadelims  -- get distance limits for evasion, left and right side of velocity vector
//
//  This projects the bounding box of the threat along the threat's movement vector,
//  and returns the distance between the target and the bounding box on each side.
//  
//  Left is negative, right is positive.
//
//  If rightlim < -(gBhvWidth + safemargin), collision not possible, threat will pass on right
//  if leftlim > (gBhvWidth + safemargin), collision not possible, threat will pass on left
//  If a collision is possible, evade in the direction that has the smallest
//  magnitude. That's the shortest distance to escape.
//
//  Returns [leftlim, rightlim]
//
list evadelims(vector targetpos, vector threatpos, vector threatvel, list bbworld)
{   float leftlim = INFINITY;
    float rightlim = -INFINITY;
    integer ix;
    integer iy;
    for (ix=0; ix<2; ix++)                                              // try all bounding box points
    {   for (iy=0; iy<2; iy++)
        {   vector vx = llList2Vector(bbworld,ix);                      // both X values
            vector vy = llList2Vector(bbworld,iy);                      // both Y values
            vector bbpnt = <vx.x,vy.y,0>;                               // a bounding box point
            float signeddistfromline = distpointtolinexysigned(targetpos, bbpnt, bbpnt+threatvel); // dist between target and projected threat
            ////llOwnerSay("evade dist " + (string)signeddistfromline + " from " + (string)targetpos + " vs. " + (string)bbpnt + " along " + (string) threatvel);// ***TEMP***
            if (signeddistfromline < leftlim) { leftlim = signeddistfromline; } // update bounds of projected threat
            if (signeddistfromline > rightlim) { rightlim = signeddistfromline; }
        }
    }
    //  Z check so as not to evade low flying aircraft, overpasses, other floors, etc.
    {
        vector bblo = llList2Vector(bbworld,0);
        vector bbhi = llList2Vector(bbworld,1);
        float HEIGHTSAFETYFACTOR = 2.0;
        if ((targetpos.z - gBhvHeight*HEIGHTSAFETYFACTOR > bbhi.z) || (targetpos.z + gBhvHeight*HEIGHTSAFETYFACTOR < bblo.z))
        {   debugMsg(DEBUG_MSG_WARN,"No threat, big altitude difference");
            return([INFINITY,-INFINITY]);
        }       
    }
    ////llOwnerSay("evade lims: " + llDumpList2String([leftlim,rightlim],",")); // ***TEMP***
    return([leftlim, rightlim]);   
}
//
//  evadedir -- pick direction to evade to, or ZERO_VECTOR if fail
//
vector evadedir(vector targetpos, vector threatpos, vector threatvel, list bbworld)
{   threatvel.z = 0;                                // assume move in XY plane
    list lims = evadelims(targetpos, threatpos, threatvel, bbworld);    // find evasion limits
    float leftlim = llList2Float(lims,0);           // get left and right lim from result
    float rightlim = llList2Float(lims,1);
    if (leftlim == INFINITY && rightlim == -INFINITY) { return(ZERO_VECTOR); } // altitude diff, no evasion needed
    float safedist = gBhvWidth*0.5 + AVOID_DIST;
    if (leftlim > safedist || rightlim < -safedist) // if edge of bounding box outside collision zone
    {   debugMsg(DEBUG_MSG_WARN,"No collision predicted.");
        return(ZERO_VECTOR);
    }
    //  OK, collision possible. Which way do we evade?  Shortest move that will get us safe.
    //  So, don't cross in front of approaching vehicle.
    float evade = leftlim - safedist;               // assume evade to left
    if (-leftlim > rightlim)                        // if shorter escape to right ????? ***NOT SURE ABOUT THIS***
    {   evade = rightlim + safedist; }              // evade to right
    if (evade == 0.0) { return(ZERO_VECTOR); }      // no need to evade
    threatvel.z = 0;
    if (llVecMag(threatvel) < 0.001) { return(ZERO_VECTOR); } // not moving, and avoid null unit vector
    threatvel = llVecNorm(threatvel);               // vel, normalized 
    vector crossdir = threatvel % <0,0,1>;          // crosswise dir
    ////llOwnerSay("evade: " + (string)(crossdir*evade) + " threatvel: " + (string)threatvel);   // ***TEMP***
    return(crossdir*evade);                         // evade this way by this much   
}

//
//  avoidthreat -- avoid incoming object
//
//  Get direction of incoming object and move out of its way.
//  Must work for vehicles which are longer than they are wide.
//
//  Returns 0 if no threat, 1 if threat being avoided, -1 if no escape
//
integer avoidthreat(key id)
{
    list threat = llGetObjectDetails(id,[OBJECT_POS, OBJECT_ROT, OBJECT_VELOCITY, OBJECT_NAME]); // data about incoming threat
    if (llGetListLength(threat) < 4) { return(0); } // no object, no threat
    vector threatpos = llList2Vector(threat,0);     // position
    rotation threatrot = llList2Rot(threat,1);      // rotation
    vector threatvel = llList2Vector(threat,2);     // velocity
    vector targetpos = llGetRootPosition();         // our position
    ////integer stationary = (llVecMag(targetpos - gLastSelfPos) < 0.01) 
    ////    && (llGetUnixTime() > gLastSelfMovetime + MAX_STATIONARY_TIME); // true if NPC did not move recently
    list bb = pathGetBoundingBoxWorld(id);          // find bounding box in world coords
    vector p = findclosestpoint(id, targetpos, bb); // find closest point on bounding box of object to pos
    vector vectothreat = p - targetpos;             // direction to threat
    vectothreat.z = 0;                              // ignore height diff here, caught elsewhere.
    float disttothreat = llVecMag(vectothreat);     // distance to threat
    ////llOwnerSay("Avoid, Vec to threat: " + (string)vectothreat + " threatvel: " + (string)threatvel ); // ***TEMP***
    float approachrate = 1.0;                       // assume moderate approach rate
    float eta = 1.0;                                // assume threat is here
    if (disttothreat > 0.001)                       // if positive distance to threat
    {   approachrate = - threatvel * llVecNorm(vectothreat); // approach rate of threat
        if (approachrate <= 0.01 && disttothreat > REACT_DIST) 
        {   debugMsg(DEBUG_MSG_NOTE,"Not a threat. Approach rate " + (string)approachrate 
                + " Dist " + (string)disttothreat); // 
            return(0);                              // not a threat
        }
        if (disttothreat > REACT_DIST && approachrate > 0.01)  // if not too close, use approach rate for calc
        {   eta = disttothreat / approachrate;  }   // time until collision                             // force evade regardless of speed
        else                                        // very close
        {
            vectothreat = threatpos - targetpos;    // use direction to center of threat
            threatvel = -llVecNorm(vectothreat);    // fake velocity vector
        }
    } else {                                        // threat is upon us
        //  We are within the bounding box of the threat. Get out of here now.
        vectothreat = threatpos - targetpos;        // use direction to center of threat
        if (llVecMag(vectothreat) > 0.001)
        {    threatvel = -llVecNorm(vectothreat);   // fake velocity vector
        } else {
            threatvel = <1,0,0>*threatrot;          // we're inside the object. Go sideways. Might help
        }           
        debugMsg(DEBUG_MSG_WARN,"Inside bounding box of threat. Get out of here.");                                            
    }
    vectothreat.z = 0;                              // in XY plane
    debugMsg(DEBUG_MSG_WARN,"Inbound threat " + llList2String(threat,3) + " at " + (string)p + " vec to threat " + (string)vectothreat 
        + " ETA " + (string)eta + "s  approach rate: " + (string)approachrate);
    if (eta > MAX_AVOID_TIME) { return(0); }        // not approaching fast enough to need avoidance
    if (testprotectiveobstacle(targetpos, threatpos, id)) { return(0); } // protective obstacle between threat and target - no need to run
    float disttopath =  distpointtoline(targetpos, p, p + threatvel); // distance from line threat is traveling
    if (disttopath - gBhvWidth*0.5 > AVOID_DIST && disttothreat > REACT_DIST) { return(0); }    // will pass by at safe distance
    //  Check which way to avoid
    vector evadevec = evadedir(targetpos, threatpos, threatvel, bb); // get which way to best evade
    if (evadevec == ZERO_VECTOR) { return(0); }     // no need to avoid
    //  Definite threat - need to avoid.
    //  Decide which direction to run.
    //  Prefer to run crosswise to direction of threat, but will 
    //  consider running away at 45 degrees or straight ahead
    ////float disttorun = llVecMag(evadevec);           // how far to evade
    float disttorun = gBhvWidth*0.5 + AVOID_DIST;   // always evade by a fixed distance. Will evade again if necessary
    integer i;
    vector threatvelnorm = llVecNorm(threatvel);    // direction threat is traveling
    vector evadevecnorm = llVecNorm(evadevec);      // ideal direction to evade
    for (i = 0; i < DIRTRIES; i++)                  // try several directions
    {   float fract = (float)((i+gNextAvoid) % DIRTRIES) / (float)(DIRTRIES-1); // fraction for weighting
        float evadeangle = fract*PI;                     // fraction of a half circle
        vector avoiddir = llCos(evadeangle)*evadevecnorm + llSin(evadeangle)*threatvelnorm; // Try across half circle starting at avoiddir
        vector escapepnt = testavoiddir(targetpos,avoiddir*disttorun); // see if avoid dir is valid
        if (escapepnt != ZERO_VECTOR)               // can escape this way
        {   ////***REEXAMINE THIS CHECK*** - removed to make evade ahead of vehicle work
            ////if (distpointtoline(escapepnt, p, p + threatvel) > disttopath) // if improves over doing nothing
            {   debugMsg(DEBUG_MSG_WARN,"Incoming threat - escaping by moving " + (string)disttorun + "m from " + (string)targetpos + " to " + (string)escapepnt); // heading for here
                debugMsg(DEBUG_MSG_WARN,"avoiddir: " + (string)avoiddir + " fract: " + (string)fract); // heading for here
                gAction = ACTION_AVOIDING;              // now avoiding
                gNextAvoid = (gNextAvoid+1) % DIRTRIES;    // advance starting point cyclically
                bhvNavigateTo(llGetRegionCorner(),escapepnt,0.0,CHARACTER_RUN_SPEED);
                gLastAvoidtime = llGetUnixTime();       // time of last avoid activity
                return(1);                              // evading threat
            }
        }
    }
    debugMsg(DEBUG_MSG_WARN,"Unable to escape incoming threat.");
    bhvSay("STOP!");
    ////llPlaySound(SCREAM_SOUND,1.0);                  // scream, that's all we can do
    return(-1);                                     // incoming threat and cannot avoid
}

float pathMin(float a, float b) { if (a<b) { return(a); } else { return(b); }} // usual min and max, avoiding name clashes
float pathMax(float a, float b) { if (a>b) { return(a); } else { return(b); }}
//
//  pathGetBoundingBoxWorld -- get object bounding box in world coordinates
//
//  LSL gives us the bounding box in object coordinates. We have to rotate it
//  and get the limits.  One vertex at a time.
//
list pathGetBoundingBoxWorld(key id)
{
    list info = llGetObjectDetails(id, [OBJECT_POS, OBJECT_ROT]) + llGetBoundingBox(id);
    vector pos = llList2Vector(info, 0);            // position in world coords
    rotation rot = llList2Rot(info, 1);             // rotation in world coords
    integer ix; 
    integer iy;
    integer iz;
    //  Convert each corner of the original bounding box to world coordinates.
    vector mincorner;                               // bounding box in world coords
    vector maxcorner;
    integer first = TRUE;                           // first time through
    for (ix=0; ix<2; ix++)                          // do all vertices of bounding box
    {   vector vx = llList2Vector(info, ix+2);
        for (iy=0; iy<2; iy++)
        {   vector vy = llList2Vector(info, iy+2);
            for (iz=0; iz<2; iz++)
            {   vector vz = llList2Vector(info, iz+2);
                vector pt = <vx.x, vy.y, vz.z>;     // one corner of the bounding box in obj coords
                vector ptworld = pt * rot + pos;    // in world coords
                if (first)
                {   mincorner = ptworld;
                    maxcorner = ptworld;
                    first = FALSE;
                }  else {
                    mincorner = <pathMin(mincorner.x,ptworld.x), pathMin(mincorner.y, ptworld.y), pathMin(mincorner.z, ptworld.z)>;
                    maxcorner = <pathMax(maxcorner.x,ptworld.x), pathMax(maxcorner.y, ptworld.y), pathMax(maxcorner.z, ptworld.z)>; 
                }
            }
        }
    }
    return([mincorner, maxcorner]);                 // min and max corners, in world coordinates
}
//
//  testavoiddir -- test avoid dir for reachability
//
//  Quick test for an escape direction.
//  This is a cheap and conservative check, not that good on irregular terrain.
//
//  Test for walkable above/below pos + avoidvec.
//  Find ground height there.
//  Cast ray to ground height + height*0.5
//
//  Pos is center of NPC.
//
vector testavoiddir(vector pos, vector avoidvec)
{
    vector p = pos + avoidvec;
    float z = obstacleraycastvert(p+<0,0,gBhvHeight>,p-<0,0,gBhvHeight>);
    if (z < 0) 
    {   debugMsg(DEBUG_MSG_WARN,"No walkable below avoid dest at " + (string)p);
        return(ZERO_VECTOR);                    // no walkable here
    }
    p.z = z + gBhvHeight*0.5;                   // ground level destination 
    if (obstacleraycasthoriz(pos,p)) 
    {   debugMsg(DEBUG_MSG_WARN,"Obstacle blocks avoid move from " + (string)(pos) + " to " + (string)(p));
        return(ZERO_VECTOR); 
    }
    return(p);                                  // success, OK to try going there
}

//
//  testprotectiveobstacle -- is there a protective obstacle between target and threat?
//
//  A walkable or a static obstacle is protective.
//  One cast ray at waist height.
//  Targetpos and threatpos must be at waist level.
//
integer testprotectiveobstacle(vector targetpos, vector threatpos, key threatid)
{
    //  If there is anything between target and threat, don't run away.
    //  This includes physical objects and avatars, which don't provide much protection.
    //  Realistic, though.
    targetpos = targetpos + <0,0,PROTECTIVE_HEIGHT - gBhvHeight*0.5>;   // an obstacle at this height is protective 
    list castresult = castray(targetpos,threatpos,PATHCASTRAYOPTSOBS);        // Horizontal cast at full height, any hit is bad
    integer status = llList2Integer(castresult, -1);        // status is last element in list
    if (status <= 0) { return(FALSE); }                     // nothing in between, threat real
    //  Hit something. Must analyze.
    //  Hit ourself, ignore.
    //  Hit threat, ignore. 
    integer i;
    for (i=0; i<3*status; i+=3)                             // check objects hit. Check two, because the first one might be ourself
    {
        key hitobj = llList2Key(castresult, i+0);           // get object hit
        if (hitobj != gPathSelfObject && hitobj != threatid)// if hit something other than self or threat
        {   return(TRUE); }                                 // found protective object
    }
    return(FALSE);  
}

//
//  evaluatethreat -- is incoming object a threat?  Quick check.
//
//  This check is made before taking control of the NPC.
//
//  Get direction of incoming object and move out of its way.
//  Must work for vehicles which are longer than they are wide.
//
//  Returns TRUE if a threat.
//
integer evalthreat(key id)
{
    list threat = llGetObjectDetails(id,[OBJECT_POS, OBJECT_ROT, OBJECT_VELOCITY, OBJECT_NAME]); // data about incoming threat
    if (llGetListLength(threat) < 3) { return(FALSE); } // no object, no threat
    vector threatpos = llList2Vector(threat,0);     // position
    rotation threatrot = llList2Rot(threat,1);      // rotation
    vector threatvel = llList2Vector(threat,2);     // velocity
    vector targetpos = llGetRootPosition();         // our position
    ////integer stationary = (llVecMag(targetpos - gLastSelfPos) < 0.01) 
    ////    && (llGetUnixTime() > gLastSelfMovetime + MAX_STATIONARY_TIME); // true if NPC (not threat) did not move recently
    list bb = pathGetBoundingBoxWorld(id);          // find bounding box in world coords
    vector p = findclosestpoint(id, targetpos, bb); // find closest point on bounding box of object to pos
    vector vectothreat = p - targetpos;                 // direction to threat
    float disttothreat = llVecMag(vectothreat);     // distance to threat
    float approachrate = - threatvel * llVecNorm(vectothreat); // approach rate of threat
    //  Do we need to even consider this threat?
    //  If not approaching and threat not close, ignore.
    //  Previous version allowed approaching stationary vehicles, but now we avoid them when the engine is running.
    //  The idea is that the NPC can closely approach a non-moving vehicle but can't sit there and block it.
    if (approachrate <= 0.01 && disttothreat > REACT_DIST) 
    {   debugMsg(DEBUG_MSG_NOTE,"No threat. Approach rate " + (string)approachrate 
            + " Dist " + (string)disttothreat); // 
        //  If close, speed up sensor polling.
        if (disttothreat < SENSE_DIST)
        {   gLastThreatTime = llGetUnixTime();      // reset time for sensor slowdown
            if (CLOSE_SENSOR_PERIOD < gSensorPeriod)  { setsensortime(CLOSE_SENSOR_PERIOD); } 
        }          
        return(FALSE);                              // not a threat
    }       
    float eta = 1.0;                                // assume 1 sec to collision
    if (llFabs(approachrate) > 0.001)               // if moving, compute a real ETA
    {   eta = disttothreat / approachrate;  }       // time until collision
    debugMsg(DEBUG_MSG_WARN,"Inbound threat " + llList2String(threat,3) + " at " + (string)p + " ETA " + (string)eta + "s.");
    gLastThreatTime = llGetUnixTime();              // record last threat time to decide when things are quiet
    if (eta > MAX_AVOID_TIME)                       // not a threat now, but step up polling
    {   float fastpoll = eta*0.1;                   // to 10% of ETA
        if (disttothreat < AVOID_DIST) { fastpoll = CLOSE_SENSOR_PERIOD; } // if nearby threat, keep polling fast regardless of speed
        if (fastpoll < gSensorPeriod) { setsensortime(fastpoll); } // poll faster to follow inbound threat
        gLastThreatTime = llGetUnixTime();          // reset time for sensor slowdown
        return(FALSE);                              // not approaching fast enough to need avoidance
    }
    //  Real threat - need to try to avoid.
    return(TRUE);
}

//
//  checkdwelltime -- check if in dwell time, holding the avoid behavior in control to avoid going back toward threat
//
checkdwelltime()
{   if (gAction == ACTION_DWELL)                    // if in dwell timeout
    {   if (gLastAvoidtime + DWELL_TIME < llGetUnixTime())       // if dwell time has expired
        {   gAction == ACTION_IDLE;                 // back to idle mode
            debugMsg(DEBUG_MSG_WARN,"Post-avoid dwell done, back to normal.");
            bhvSetPriority(PRIORITY_OFF);           // turn self off
        }
    }
} 
//
//  doneavoid -- done with all requests, give control back to scheduler
//
doneavoid()
{   debugMsg(DEBUG_MSG_WARN,"Avoiding done.");      // All done with avoids for now
    assert(gAction == ACTION_IDLE || gAction == ACTION_DWELL);  // must not have something running
    gThreatList = [];                               // clear to-do list.
    checkdwelltime();                               // release control of NPC if dwell timer has expired.
}

//
//  bhvDoRequestDone -- pathfinding is done. Analyze the result and start the next action.
//
bhvDoRequestDone(integer status, key hitobj)
{   debugMsg(DEBUG_MSG_WARN, "Avoid update: " + (string) status + " obstacle: " + llKey2Name(hitobj));
    if (gAction == ACTION_IDLE || gAction == ACTION_DWELL) // why did we even get a completion?
    {   restartavoid();                         // see if anything needs doing
        return;
    }
    assert(gAction == ACTION_AVOIDING);         // we must be avoiding
    gLastAvoidtime = llGetUnixTime();       // time of last avoid activity
    gAction = ACTION_IDLE;                      // and now idle, no path operation in progress   
    if (status == PATHERRMAZEOK)                // success
    {   
        debugMsg(DEBUG_MSG_NOTE,"Avoid move reached safe place, rechecking threat.");
        startavoid();                           // avoid some other threat if any
        return;
    }
    //  Obstructed while escaping
    //  If blocked by something, deal with it.
    debugMsg(DEBUG_MSG_WARN,"Avoid unsuccessful, status " + (string)status + " at " + (string)llGetRootPosition());
    gAvoidTries++;
    if (gAvoidTries >= MAX_AVOID_TRIES)
    {   debugMsg(DEBUG_MSG_ERROR,"Unable to avoid threat after many tries at " + (string)llGetRootPosition()); // Either broken or being griefed            
        doneavoid();                            // turn us off
        return;
    }                                                              
    restartavoid();                             // otherwise try again
}

//
//  startavoid -- new threat has appeared, start avoiding it
//
startavoid()
{
    gAvoidTries = 0;                                                // no retries yet
    gNextAvoid = 0;                                                 // start from beginning
    restartavoid();                                                 // and look for something to avoid
}
//
//  restartavoid -- start avoidance of threat
//
restartavoid()
{   assert(gAction == ACTION_IDLE || gAction == ACTION_DWELL);      // must be idle when called
    debugMsg(DEBUG_MSG_WARN,(string)llGetListLength(gThreatList) + " threats queued"); // ***TEMP***
    while (llGetListLength(gThreatList) > 0)                        // while threats to handle
    {   key id = llList2Key(gThreatList,0);                         // get oldest threat
        integer threatstatus = avoidthreat(id);                     // try to avoid threat
        if (threatstatus == 0) 
        {   gThreatList = llDeleteSubList(gThreatList,0,0);         // first threat no longer a threat, delete from to-do list
        } else if (threatstatus == 1) {
            assert(gAction == ACTION_AVOIDING);                     // should be avoiding now
            return;                                                 // doing something about the threat already
        } else {                                                    // big trouble, no escape
            debugMsg(DEBUG_MSG_ERROR,"Unable to avoid threat by any path from " + (string)llGetRootPosition()); // Either broken or being griefed
            gAction = ACTION_DWELL; 
            doneavoid();                                            // give up 
            return;  
        }             
    }
    //  No threat, give up control, but may need dwell time
    gAction = ACTION_DWELL;                                          // now idle
    doneavoid();                                                    // done, give up control   
}
//
//  requestavoid -- request to avoid something, from avatar monitoring task.
//
requestavoid(key id)
{   if (llListFindList(gThreatList,[id]) >= 0) { return; }          // we have this one
    if (!evalthreat(id)) { return; }                                // not a threat, ignore
    debugMsg(DEBUG_MSG_WARN,"New threat: " + llKey2Name(id));       // note new threat
    gThreatList += id;                                              // add to threat list
    bhvSetPriority(PRIORITY_AVOID);                                 // we need control of the NPC immediately
}

//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    gAction = ACTION_IDLE;                                          // whatever we were doing is cancelled
    start_anim(IDLE_ANIM);                                          // use idle animation
    llSetTimerEvent(IDLE_POLL);                                     // check for dwell time
    startavoid();                                                   // do we need to avoid something? Probably.
}
//
//  bhvDoStop -- this behavior no longer has control
//
bhvDoStop()
{
    gAction = ACTION_IDLE;                                          // we've been preempted. No patrol now.
    llSetTimerEvent(0);                                             // don't need a timer
}

//
//  start_anim -- start indicated idle animation.
//
//  Single anim only; we don't need multiple here.
//
start_anim(string anim)
{
    bhvAnimate([anim]);                             // new API
}

//
//  startup - initialization
//
startup()
{   pathinitutils();                                // init library
    gAction = ACTION_IDLE;
    gThreatList = [];                               // no threats yet
    //  Set up connection to scheduler
    bhvInit();                                      // set up scheduler system
}

//
// bhvRegistered -- scheduler is ready to run us.
//
bhvRegistered()                                                     // tell controlling script to go
{
    //  We don't ask to run yet; we will ask for control of the NPC when there's a threat.
    //  Simple avatar sensor - replace with something more efficient.
    setsensortime(IDLE_SENSOR_PERIOD);                                // start scanning for threats
}

//
//  bhvDoCollisionStart -- a collision has occured while this behavor has control.
//
//  We don't actually have to do anything about the collision.
//  The path system will stop for this, and pathstart will retry.
//  Only avatars and physical objects generate collisions, because we're keyframe motion.
//
bhvDoCollisionStart(key hitobj)
{       
    list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
    integer pathfindingtype = llList2Integer(details,0);        // get pathfinding type
    debugMsg(DEBUG_MSG_WARN, "Collided with " + llList2String(details,1));
    if (pathfindingtype == OPT_AVATAR)                          // apologize if hit an avatar
    {   bhvSay("Outta my way!"); }                              // I'm about to be hit by a vehicle!
    else 
    {   bhvSay("OW!"); }
    setsensortime(MIN_SENSOR_PERIOD);                           // start fast polling for evasion
}

default
{
    on_rez(integer start_param)
    {
        llResetScript();
    }
 
    state_entry()
    {
        startup();
    }

    timer()                                                 // timer tick
    {   bhvTick();                                          // timers in path library get updated
        checkdwelltime();                                   // if in dwell time, check for done
    }
    
    sensor(integer num_detected)                            // nearby agents
    {   if (!gBhvRegistered)
        {   
            return;                                         // not initialized yet
        }
        vector ourpos = llGetRootPosition();                // where we are
        if (llVecMag(ourpos - gLastSelfPos) > 0.01)         // if we moved
        {   gLastSelfPos = ourpos;                          // our new pos
            gLastSelfMovetime = llGetUnixTime();            // when we were last there
        }
        integer i;
        for (i=0; i<num_detected; i++)                      // for all avatars in range
        {   key id = llDetectedKey(i);                      // avatar id
            if (invehicle(id))                              // if in a vehicle
            {   key rootid = getroot(id);                   // vehicle key
                requestavoid(rootid);                       // consider avoiding it
            }
        }
        if (llGetUnixTime() - gLastThreatTime > SAFE_IDLE_TIME) // if nothing going on
        {    setsensortime(IDLE_SENSOR_PERIOD);             // no action lately, return to slow polling
        }

    }
    
    no_sensor()                                             // no targets, return to idle poll rate
    {   vector ourpos = llGetRootPosition();                // where we are
        if (llVecMag(ourpos - gLastSelfPos) > 0.01)         // if we moved
        {   gLastSelfPos = ourpos;                          // our new pos
            gLastSelfMovetime = llGetUnixTime();            // when we were last there
        }
        setsensortime(IDLE_SENSOR_PERIOD);                   // slow poll
    }

    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   ////debugMsg(DEBUG_MSG_INFO, jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            bhvSchedMessage(num,jsn);                       // message from scheduler
            return;
        }
        if (num == DEBUG_MSGLEV_BROADCAST)                  // set message level from broadcast
        {   debugMsgLevelSet(jsn); return; }
    }

}


