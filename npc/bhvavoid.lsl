//
//  bhvavoid.lsl -- avoid incoming threats
//
//  For now, moving vehicles only
//
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/bhvcall.lsl"
#include "npc/pathbuildutils.lsl"

integer ACTION_IDLE = 0;
integer ACTION_AVOIDING = 1;                // trying to avoid something
integer ACTION_CANNOTAVOID = 2;             // unable to avoid


//  Configuration

#define PRIORITY_AVOID  PRIORITY_URGENT     // higher than most behaviors

#define DETECTION_RADIUS 50.0               // look for moving vehicles this range
#define AVOID_DIST 4.0                      // (m) move this far each try to get out of the way
#define IDLE_SENSOR_PERIOD 2.0              // (s) sensor poll rate when no nearby targets
#define MIN_SENSOR_PERIOD 0.2               // (s) sensor poll rate max

#ifndef CHARACTER_SPEED                     // overrideable
#define CHARACTER_SPEED  2.5                // (m/sec) speed
#endif // CHARACTER_SPEED
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
#define CHARACTER_RUN_SPEED 5.5             // (m/sec) top speed run
////string IDLE_ANIM = "stand 2";            // idle or chatting         
////string STAND_ANIM = "stand 2";           // just when stopped
///string WAITING_ANIM = "stand arms folded";  // during planning delays
string WAITING_ANIM = "SEmotion-bento13";   // arms folded during planning delays
string IDLE_ANIM = "SEmotion-bento18";      // arms folded during planning delays
string STAND_ANIM = "SEmotion-bento18";     // just when stopped
string SCREAM_SOUND = "???";                // sound of a scream
#define IDLE_POLL 10.0                      // for stall timer tick if used
#define SAFE_IDLE_TIME 15.0                 // idle this long, stop polling fast

//
//  Avoid directions - try escaping in these directions.               
//
//  All must be vectors of length 1.
//  If we have to use the last one, trouble.
//
#define HALFSQRT2 0.7071
list AVOIDDIRS = [<0,1,0>,<0,-1,0>,<HALFSQRT2,HALFSQRT2,0>,<-HALFSQRT2,HALFSQRT2,0>,<1,0,0>];

#define MAX_AVOID_TIME 6.0                  // (s) trigger avoid when ETA less than this

#define getroot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))  


//  Global variables
integer gAction = ACTION_IDLE;
list gThreatList = [];                      // list of incoming threats
integer gAvoidIndex = 0;                    // which avoid path we are taking
vector gAvoidStart = ZERO_VECTOR;           // where we started avoiding
float gWidth = 0.5;                         // dimensions of character
float gHeight = 2.0;
float gSensorPeriod = 0.0;                  // Current sensor poll rate
integer gLastThreatTime = 0;                // time last threat seen
#ifdef OBSOLETE
//
//  boxpoints -- construct box around set of points and vector
//
//  Output is center and scale of box. Rotation per dir.
//
//  ***WRONG*** needs work
//
vector boxpoints(vector pos, vector dir, list pts)
{   vector lobounds = -<BIG,BIG,0>;
    vector hibounds = <BIG,BIG,0>;
    vector crossdir = dir % <0,0,1>;        // cross direction
    integer i;
    for (i=0; i<llGetListLength(pts); i++)
    { 
        vector pt = llList2Vector(pts,i);   // next point
        float x = (pt-pos)*dir;             // distance to pt from pos along dir
        lobounds.x = min(lobounds.x, min(x,-x));
        hibounds.x = max(hibounds.x, max(x,-x);
        float y = (pt-pos)*crossdir;
        lobounds.y = min(lobounds.y, min(y,-y));
        hibounds.y = max(hibounds.y, max(y,-y);                 
    }
    //  We now have bounds relative to pos.
    vector center = (lobounds + hibounds)*0.5;  // center of enclosing box
    vector scale = <hibounds.x-lobounds.x,hibounds.y-lobounds.y,0>;
    
}
#endif // OBSOLETE
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
//  findclosestpoint -- find closest point on threat object bounding box.
//
//  Simple version - closest point is always a corner. 
//  Not going to implement GJK algorithm in LSL for this.
//
vector findclosestpoint(key id, vector pos)
{   list bb = pathGetBoundingBoxWorld(id);      // get bounding box in world coords
    vector bblower = llList2Vector(bb,0);       // these define an axis aligned box with 8 corners.
    vector bbupper = llList2Vector(bb,1);
    //  Find the corner closest to pos.
    vector closestpt = bblower;
    if (llFabs(bbupper.x - pos.x) < llFabs(bblower.x - pos.x)) { closestpt.x = bbupper.x; }
    if (llFabs(bbupper.y - pos.y) < llFabs(bblower.y - pos.y)) { closestpt.y = bbupper.y; }
    if (llFabs(bbupper.z - pos.z) < llFabs(bblower.x - pos.z)) { closestpt.z = bbupper.z; }
    return(closestpt);
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
vector testavoiddir(vector pos, vector avoidvec)
{
    vector p = pos + avoidvec;
    float z = obstacleraycastvert(p+<0,0,gHeight>,p-<0,0,gHeight>);
    if (z < 0) 
    {   debugMsg(DEBUG_MSG_WARN,"No walkable below avoid dest at " + (string)p);
        return(ZERO_VECTOR);                    // no walkable here
    }
    p.z = z;                                    // ground level destination 
    if (obstacleraycasthoriz(pos+<0,0,gHeight*0.5>,p+<0,0,gHeight*0.5>)) 
    {   debugMsg(DEBUG_MSG_WARN,"Obstacle blocks avoid move from " + (string)(pos+<0,0,gHeight*0.5>) + " to " + (string)(p+<0,0,gHeight*0.5>));
        return(ZERO_VECTOR); 
    }
    return(p);                                  // success, OK to try going there
}

//
//  testprotectiveobstacle -- is there a protective obstacle between target and threat?
//
//  A walkable or a static obstacle is protective.
//  One cast ray at waist height.
//  Targetpos and threatpos must be at ground level.
//
integer testprotectiveobstacle(vector targetpos, vector threatpos)
{
    //  ***MORE*** Not the right test. 
    ///if (obstacleraycasthoriz(targetpos+<0,0,gHeight*0.5>,threatpos+<0,0,gHeight*0.5>)) { return(TRUE); }
    return(FALSE);                              // ***TEMP***
}

//
//  evaluatethreat -- is incoming object a threat?
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
    vector pos = llGetRootPosition();               // our position
    vector p = findclosestpoint(id, pos);           // find closest point on bounding box of object to pos
    vector vectothreat = p - pos;                   // direction to threat
    float disttothreat = llVecMag(vectothreat);     // distance to threat
    float approachrate = - threatvel * llVecNorm(vectothreat); // approach rate of threat
    if (approachrate <= 0.01) { return(FALSE); }    // not approaching
    float eta = disttothreat / approachrate;        // time until collision
    debugMsg(DEBUG_MSG_WARN,"Inbound threat " + llList2String(threat,4) + " at " + (string)p + " ETA " + (string)eta + "s.");
    gLastThreatTime = llGetUnixTime();              // record last threat time to decide when things are quiet
    if (eta > MAX_AVOID_TIME)                       // not a threat now, but step up polling
    {   float fastpoll = eta*0.1;                   // to 10% of ETA
        if (fastpoll < gSensorPeriod) { setsensortime(fastpoll); } // poll faster to follow inbound threat
        return(FALSE);                              // not approaching fast enough to need avoidance
    }
    //  Definite threat - need to avoid.
    return(TRUE);
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
    if (llGetListLength(threat) < 3) { return(0); } // no object, no threat
    vector threatpos = llList2Vector(threat,0);     // position
    rotation threatrot = llList2Rot(threat,1);      // rotation
    vector threatvel = llList2Vector(threat,2);     // velocity
    vector pos = llGetRootPosition();               // our position
    vector p = findclosestpoint(id, pos);           // find closest point on bounding box of object to pos
    vector vectothreat = p - pos;                   // direction to threat
    float disttothreat = llVecMag(vectothreat);     // distance to threat
    float approachrate = - threatvel * llVecNorm(vectothreat); // approach rate of threat
    if (approachrate <= 0.01) { return(0); }        // not approaching
    float eta = disttothreat / approachrate;        // time until collision
    debugMsg(DEBUG_MSG_WARN,"Inbound threat " + llList2String(threat,4) + " at " + (string)p + " ETA " + (string)eta + "s.");
    if (eta > MAX_AVOID_TIME) { return(0); }        // not approaching fast enough to need avoidance
    //  Definite threat - need to avoid.
    vector dirfromthreat = llVecNorm(<-vectothreat.x,-vectothreat.y,0.0>); // away from threat
    rotation rotfromthreat = RotFromXAxis(dirfromthreat); // rotation from X axis
    ////vector crossdir = llVecNorm(threatvel) % <0,0,1>;  // horizontal direction to escape
    //  Decide which direction to run.
    //  Prefer to run crosswise to direction of threat, but will 
    //  consider running away and 45 degrees. So 5 possiblities to try.
    //  If nothing works, head in threat direction and go as far as possible. Scream.
    //  ***MORE***
    float disttorun = AVOID_DIST;                    // Run 4 meters to get out of the way of most vehicles 
    integer dirix;
    for (dirix = 0; dirix < llGetListLength(AVOIDDIRS); dirix++)
    {   vector avoiddir = llList2Vector(AVOIDDIRS,dirix)*rotfromthreat;   // direction to run, relative to dirfromthreat
        vector escapepnt = testavoiddir(pos,avoiddir*disttorun); 
        if (escapepnt != ZERO_VECTOR)               // can escape this way
        {   debugMsg(DEBUG_MSG_WARN,"Incoming threat - escaping to " + (string)escapepnt); // heading for here
            gAction = ACTION_AVOIDING;              // now avoiding
            bhvNavigateTo(ZERO_VECTOR,escapepnt,0.0,CHARACTER_RUN_SPEED);
            return(1);                              // evading threat
        }
    }
    debugMsg(DEBUG_MSG_WARN,"Unable to escape incoming threat.");
    bhvSay("STOP!");
    ////llPlaySound(SCREAM_SOUND,1.0);                  // scream, that's all we can do
    return(-1);                                     // incoming threat and cannot avoid
} 

//
//  bhvDoRequestDone -- pathfinding is done. Analyze the result and start the next action.
//
bhvDoRequestDone(integer status, key hitobj)
{   debugMsg(DEBUG_MSG_INFO, "Avoid update: : " + (string) status + " obstacle: " + llKey2Name(hitobj));
    if (gAction == ACTION_IDLE)                 // why did we even get a completion?
    {   bhvSetPriority(0);                      // turn us off
        return;
    }
    if (status == PATHERRMAZEOK)                    // success
    {   
        debugMsg(DEBUG_MSG_INFO,"Avoid reached safe place, rechecking threat.");
        gAction = ACTION_IDLE;                  // idle for now
        startavoid();                           // avoid top threat if any
        return;
    }
    //  Obstructed while escaping
    //  If blocked by something, deal with it.
    if (gAction == ACTION_AVOIDING)  
    {
        debugMsg(DEBUG_MSG_WARN,"Escape unsuccessful, status " + (string)status + " at " + (string)llGetRootPosition());
        //  ***NEED TO TRY ALTERNATE ESCAPE ROUTE***
        //  ***MORE***
    }                                                               // if going somewhere
    //  Default - errors we don't special case.
    gAction = ACTION_IDLE;            
    startavoid();                                                   // avoid top threat if any    
}

//
//  startavoid -- start avoidance of threat
//
startavoid()
{   if (gAction != ACTION_IDLE) { return; }                         // already dealing with a threat
    while (llGetListLength(gThreatList) > 0)                        // while threats to handle
    {   key id = llList2Key(gThreatList,0);                         // get oldest threat
        integer threatstatus = avoidthreat(id);                     // try to avoid threat
        if (threatstatus == 0) 
        {   gThreatList = llDeleteSubList(gThreatList,0,0);         // first threat no longer a threat, delete from to-do list
        } else if (threatstatus == 1) {
            return;                                                 // doing something about the threat already
        } else {                                                    // big trouble, no escape
            //  ***MORE***
        }       
    }
    //  No threat, give up control
    gAction = ACTION_IDLE;                                          // now idle
    bhvSetPriority(PRIORITY_OFF);                                   // done, give up control   
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
//  bhvDoCollisionStart -- a collision has occured.
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

    timer()                                         // timer tick
    {   bhvTick();                                  // timers in path library get updated
    }
    
    sensor(integer num_detected)                            // nearby agents
    {
        vector ourpos = llGetRootPosition();                // where we are
        integer i;
        for (i=0; i<num_detected; i++)
        {   vector vel = llDetectedVel(i);                  // velocity of threat
            vector pos = llDetectedPos(i);                  // position of threat
            float approachrate = -llVecNorm(pos-ourpos)*vel; // approach rate
            ////llOwnerSay("Avatar " + llDetectedName(i) + " at " + (string) pos + " vel " + (string)vel + " approachrate " + (string)approachrate); // ***TEMP***
            if (approachrate > 0.01)                        // if approaching us
            {   key id = llDetectedKey(i);                  // id of agent
                key rootid = getroot(id);                   // root, which will be different if vehicle
                if (id != rootid)                           // incoming vehicle
                {   ////llOwnerSay("Inbound vehicle " + llKey2Name(rootid) + " at " + (string) pos + " vel " + (string)vel + " approachrate " + (string)approachrate); // ***TEMP***
                    requestavoid(rootid);                   // consider avoiding it
                }
            }
        }
        if (llGetUnixTime() - gLastThreatTime > SAFE_IDLE_TIME) // if nothing going on
        {    setsensortime(IDLE_SENSOR_PERIOD);             // no action lately, return to slow polling
        }

    }
    
    no_sensor()                                             // no targets, return to idle poll rate
    {   setsensortime(IDLE_SENSOR_PERIOD);                    // slow poll
    }

    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   debugMsg(DEBUG_MSG_INFO, jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            bhvSchedMessage(num,jsn);                       // message from scheduler
            return;
        }
#ifdef NOTYET
        if (num == PATHAVATARAVOIDREQUEST)                  // if avatar tracker wants us to avoid something
        {   if (llJsonGetValue(jsn,["request"]) != "avoid") { return; } // not for us
            requestavoid((key)llJsonGetValue(jsn,["id"]));  // go pursue, if appropriate.
            return; 
        } 
#endif // NOTYET

    }

#ifdef DEBUGCHAN    
    listen(integer channel, string name, key id, string msg)
    {  
        if (channel == DEBUGCHAN)                               // if debug control
        {   bhvDebugCommand("Hello");   // ***TEMP***
            bhvDebugCommand(msg);
            return;
        }
    }
#endif // DEBUGCHAN   
}


