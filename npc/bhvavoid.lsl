//
//
//  bhvavoid.lsl -- avoid incoming threats
//
//  For now, moving vehicles only
//
//  Uses old-format patrol points file, lines of:
//
//      VECTOR,DWELL,HEADING
//  
//  Demo for Animats pathfinding system.
//
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/bhvcall.lsl"
#include "npc/mathutils.lsl"

integer ACTION_IDLE = 0;
integer ACTION_AVOIDING = 1;                // trying to avoid something
integer ACTION_CANNOTAVOID = 2;             // unable to avoid


//  Configuration

#define PRIORITYAVOID  4                    // higher than most behaviors

float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.7;            // (fract) Retry if random < this.
float TESTSPACING = 0.33;                   // (fract) Multiply height and width by this to get ray cast spacing

#ifndef CHARACTER_SPEED                     // overrideable
#define CHARACTER_SPEED  2.5                // (m/sec) speed
#endif // CHARACTER_SPEED
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
////string IDLE_ANIM = "stand 2";            // idle or chatting         
////string STAND_ANIM = "stand 2";           // just when stopped
///string WAITING_ANIM = "stand arms folded";  // during planning delays
string WAITING_ANIM = "SEmotion-bento13";   // arms folded during planning delays
string IDLE_ANIM = "SEmotion-bento18";      // arms folded during planning delays
string STAND_ANIM = "SEmotion-bento18";     // just when stopped
float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                  // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;            // must move at least this far before we recheck on approach
#ifndef VERBOSITY                           // define VERBOSITY to override
#define VERBOSITY DEBUG_MSG_ERROR            // verbose
#endif // VERBOSITY

//
//  Avoid directions - try escaping in these directions.               
//
//  All must be vectors of length 1.
//  If we have to use the last one, trouble.
//
#define HALFSQRT2 0.7071
list AVOIDDIRS [<0,1,0>,<0,-1,0>,<HALFSQRT2,HALFSQRT2,0>,<-HALFSQRT2,HALFSQRT2,0>,<0,1,0>];

integer PATH_STALL_TIME = 300;              // path stall time


//  Global variables
integer gAction = ACTION_IDLE;
key gThreatId = NULL_KEY;                   // incoming threat
float gWidth = 0.5;                         // dimensions of character
float gHeight = 2.0;

//
//  testavoiddir -- test avoid dir for reachability
//
//  Quick test for an escape direction
//
//  Test for walkable above/below pos + avoidvec.
//  Find ground height there.
//  Cast ray to ground height + height*0.5
//
vector testavoiddir(vector pos, vector avoidvec, float height)
{
    
}
//
//  avoidthreat -- avoid incoming object
//
//  Get direction of incoming object and move out of its way.
//  Must work for vehicles which are longer than they are wide.
//
//  Returns TRUE if no threat.
//
integer avoidthreat(key id, float width, float height)
{
    list threat = llGetObjectDetails(id,[OBJECT_POS, OBJECT_ROT, OBJECT_VELOCITY]); // data about incoming threat
    if (llGetListLength(threat) < 3) { return(FALSE); } // no object, no threat
    vector threatpos = llList2Vector(threat,0);     // position
    rotation threatrot = llList2Rotation(threat,1); // rotation
    vector threatvel = llList2Vector(threat,2);     // velocity
    vector pos = llGetRootPosition();               // our position
    vector dirtothreat = threatpos - pos;           // direction to threat
    vector bb = llGetBoundingBox(id);               // bounding box of threat
    vector p = findclosestpoint(bb,pos);            // find closest point on bounding box of object to pos
    vector vectothreat = p - pos;                   // direction to threat
    float disttothreat = llVecMag(vectothreat);     // distance to threat
    float approachrate = - threatvel * llVecNorm(disttothreat); // approach rate of threat
    if (approachrate <= 0.01) { return(FALSE); }    // not approaching
    float eta = disttothreat / approachrate;        // time until collsion
    if (eta > MAXAVOIDTIME) { return(FALSE); }      // not approaching fast enough to need avoidance
    //  Definite threat - need to avoid.
    vector crossdir = llVecNorm(threatvel) % <0,0,1>;  // horizontal direction to escape
    //  Decide which direction to run.
    //  Prefer to run crosswise to direction of threat, but will 
    //  consider running away and 45 degrees. So 5 possiblities to try.
    //  If nothing works, head in threat direction and go as far as possible. Scream.
    //  ***MORE***   
    integer dirix;
    for (dirix = 0; dirix < llGetListLength(AVOIDDIRS); i++)
    {   vector avoiddir = llList2Vector(AVOIDDIRS,i);   // direction to run
        vector escapepnt = testavoiddir(pos,avoiddir, height); 
        if (escapepnt != ZERO_VECTOR)               // can escape this way
        debugMsg(DEBUG_MSG_WARN,"Incoming threat - escaping to " + (string)escapepnt); // heading for here
        gAction = ACTION_AVOIDING;                  // now avoiding
        bvhNavigateTo(ZERO_VECTOR,escapepnt,CHARACTER_RUN_SPEED);
        return(TRUE);
    }
    debugMsg(DEBUG_MSG_WARN,"Unable to escape incoming threat.");
    llPlaySound(SCREAM);                            // scream, that's all we can do
    return(TRUE);                                   // need to rethink statuses here
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
    {   ////llOwnerSay("Pathfinding task completed.");
        if (gAction == ACTION_AVOIDING)
        {   integer avoiding = avoidthreat(gThreatId,gWidth, gHeight);
            if (avoiding) 
            {   debugMsg(DEBUG_MSG_WARN,"Still avoiding");
            } else {
                gAction = ACTION_IDLE;              // we are done here
                gThreatId = NULL_KEY;               // no threat
                bhvSetPriority(0);                  // turn us off
            }
            return;
        }
        
        
         //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        gAction = ACTION_IDLE;            
        bhvSetPriority(0);
        return;
    }
    //  Obstructed while escaping
    //  If blocked by something, deal with it.
    if (gAction == ACTION_AVOIDING)  
    {
        //  ***NEED TO TRY ALTERNATE ESCAPE ROUTE***
        //  ***MORE***
    }                                               // if going somewhere
    //  Default - errors we don't special case.
    {                      
        //  Failed, back to idle.
        gAction = ACTION_IDLE;            
        debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        start_anim(IDLE_ANIM);
        return;
    }
}

//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    gAction = ACTION_IDLE;                                          // whatever we were doing is cancelled
    start_anim(IDLE_ANIM);                                          // use idle animation
    llSetTimerEvent(IDLE_POLL);                                     // check for dwell time
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
//  face -- face in indicated direction
//
face(key target, integer nextstate)                 // turn to face avi
{
    vector dir = vec_to_target(target);             // direction to face
    float heading = llAtan2(dir.x,dir.y);           // direction to face as heading (0=north)
    pathTurn(heading);                              // turn to face avi
    gAction = nextstate;                            // on completion, greet
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
//  start_platrol -- start patrolling if allowed to do so.
//
start_patrol()
{   //  Start patrolling if nothing else to do
    if (gAction == ACTION_IDLE && gPatrolEnabled && 
        llGetTime() > gDwell)      
    {   llResetTime(); 
        //  Pick a random patrol point different from the last one.
        integer newpnt;
        integer bound = llGetListLength(gPatrolPoints); // want 0..bound-1
        if (bound < 1)
        {   llSay(DEBUG_CHANNEL,"No patrol points, cannot patrol."); return; }
        do { newpnt = rand_int(bound);  }
        while (newpnt == gNextPatrolPoint);
        gNextPatrolPoint = newpnt;
        gPatrolDestination = llList2Vector(gPatrolPoints, gNextPatrolPoint);
        gDwell = llList2Float(gPatrolPointDwell, gNextPatrolPoint);
        gFaceDir = llList2Float(gPatrolPointDir, gNextPatrolPoint);
        restart_patrol();
    }  
}
//
//  restart_patrol -- start patrol to previously selected point.
//
restart_patrol()
{
    debugMsg(DEBUG_MSG_WARN,"Patrol to " + (string)gPatrolDestination);
    start_anim(WAITING_ANIM);                       // applies only when stalled during movement
    bhvNavigateTo(ZERO_VECTOR,gPatrolDestination,0,CHARACTER_SPEED);  // head for next pos
    gAction = ACTION_PATROL;                        // patrolling
}

//
//  startup - initialization
//
startup()
{
    gAction = ACTION_IDLE;
    //  Start loading patrol points.
    //  Get our name
    gName = llGetObjectName();
    integer spaceIndex = llSubStringIndex(gName, " ");
    if (spaceIndex >0)
    {   gName  = llGetSubString(gName, 0, spaceIndex - 1); }       // first name of character
    //  Set up connection to scheduler
    bhvInit();                              // set up scheduler system
}

//
// bhvRegistered -- scheduler is ready to run us.
//
bhvRegistered()                                                     // tell controlling script to go
{
    bhvSetPriority(PRIORITYPATROL);                                 // now we can ask to run
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
    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   debugMsg(DEBUG_MSG_INFO, jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            bhvSchedMessage(num,jsn);                       // message from scheduler
            return;
        }
    }
    
    collision_start(integer num_detected)
    {   key hitobj = llDetectedKey(0);                       // first object hit
        list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
        integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
        debugMsg(DEBUG_MSG_WARN, "Collided with " + llList2String(details,1));
        if (pathfindingtype == OPT_AVATAR)                      // apologize if hit an avatar
        {   llSay(0,"Excuse me."); }
    }
   
}

#ifdef DEBUGCHAN    
    listen(integer channel, string name, key id, string msg)
    {  
        if (channel == DEBUGCHAN)                               // if debug control
        {   bvhDebugCommand(msg);
            return;
        }
    }
#endif // DEBUGCHAN   
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
