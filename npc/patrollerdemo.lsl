//
//  patrollerdemo.lsl  
//  
//  Demo for Animats pathfinding system.
//
//  Approaches any new avatar and says hello.
//  Otherwise moves between patrol points.
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/pathcall.lsl"

integer ACTION_IDLE = 0;         
integer ACTION_PURSUE = 1;
integer ACTION_FACE = 2;
integer ACTION_PATROL = 3;


//  Configuration
float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                      // get this close to talk
//  Character dimensions
float CHARACTER_WIDTH = 0.5;
float CHARACTER_HEIGHT = 2.2;
float CHARACTER_SPEED = 2.5;                // (m/sec) speed
float CHARACTER_TURNSPEED_DEG = 90.0;       // (deg/sec) turn rate
string IDLE_ANIM = "stand 2";               // idle or chatting         
string STAND_ANIM = "stand 2";              // just when stopped
string WAITING_ANIM = "impatient";          // during planning delays
float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                  // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;            // must move at least this far before we recheck on approach
#ifndef VERBOSITY                           // define VERBOSITY to override
#define VERBOSITY PATH_MSG_ERROR            // verbose
#endif // VERBOSITY

//  Configuration
string PATROL_NOTECARD = "Patrol points";   // read this notecard for patrol points

integer PATH_STALL_TIME = 300;              // path stall time


//  Global variables
integer gAction = ACTION_IDLE;
string gAnim;                   // current animation                       
key gTarget = NULL_KEY;         // current avatar to pursue
list gGreetedTargets = [];      // we have said hello
list gDeferredTargets = [];     // trouble with these, wait and retry
list gDeferredPositions = [];   // don't retry until target moves
integer gListenChannel;         // for detecting chat
vector gScale;                  // scale of character
key gOwner;                     // owner of character
key gGroup;                     // group of character
string gName;                   // our name
float gPathDistance = 0.0;      // how far to goal, for stuck detection

//  Patrol points
integer gPatrolEnabled;
integer gPatrolNotecardLine;
key gPatrolNotecardQuery;
list gPatrolPoints = [];        // list of points to patrol 
list gPatrolPointDwell = [];    // dwell time at patrol point
list gPatrolPointDir = [];      // direction to face on arriving
integer gNextPatrolPoint;       // next patrol point we are going to
vector gPatrolDestination;      // where are we going next?
float gDwell;                   // dwell time at next patrol point
float gFaceDir;                 // direction to face next


//
//  Static path tester constants
//
integer CHARTYPE = CHARACTER_TYPE_A;                        // humanoid
float CHARRADIUS = 0.25;                                    // radius 
float CHARHEIGHT = 1.8;                                     // height 1.8m
float TESTSPACING = 0.33;                                   // 3 test points per meter in height


add_patrol_point(string s)
{   s = llStringTrim(s, STRING_TRIM);           // clean
    if (s == "") { return; }                    // blank line        
    if (llGetSubString(s,0,0) == "#") { return; }   // comment line
    list parsed = llParseString2List(s,[" "],[]);          // parse
    vector pos = (vector)llList2String(parsed, 0);
    float dwell = (float)llList2String(parsed,1);       // dwell time
    float facedir = (float)llList2String(parsed,2)*DEG_TO_RAD;       // heading to face
    if (pos == ZERO_VECTOR)
    {   llSay(DEBUG_CHANNEL, "Invalid patrol point: " + s);
        return;
    }
    pathMsg(PATH_MSG_INFO,"Added patrol point: " + (string) pos + "  Dwell time: " + (string)dwell
        + " Face dir: " + (string)facedir); 
    gPatrolPoints += pos;
    gPatrolPointDwell += dwell;
    gPatrolPointDir += facedir;
}

//
//  pathUpdateCallback -- pathfinding is done. Analyze the result and start the next action.
//
pathUpdateCallback(integer status, key hitobj )
    {   pathMsg(PATH_MSG_INFO, "Path update: : " + (string) status);
        if (status == MAZESTATUSOK)          // success
        {   ////llOwnerSay("Pathfinding task completed.");
            if (gAction == ACTION_PURSUE)               
            {
                gAction = ACTION_FACE;  
                vector offset = dir_from_target(gTarget) * GOAL_DIST; 
                vector finaltarget = target_pos(gTarget) + offset;
                if (clear_sightline(gTarget, finaltarget))
                {            
                    pathMsg(PATH_MSG_INFO,"Get in front of avatar. Move to: " + (string)finaltarget);
                    gPathDistance = pathdistance(llGetPos(), gPatrolDestination, CHARACTER_WIDTH, CHARACTER_TYPE_A);  // measure distance to goal
                    pathNavigateTo(finaltarget, 0);
                    llResetTime();
                } else {
                    pathMsg(PATH_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                    face_and_greet();
                }                             
                return;
            }
            if (gAction == ACTION_FACE)
            {   //  Turn to face avatar.
                face_and_greet();
            } 
            if (gAction == ACTION_PATROL)
            {   face_dir(<llSin(gFaceDir), llCos(gFaceDir), 0.0>);   // face in programmed direction
                start_anim(IDLE_ANIM);
                gAction = ACTION_IDLE;
                llResetTime();                          // reset timeout timer but keep dwell time
                pathMsg(PATH_MSG_INFO,"Patrol point reached.");
            }  
            return;
        } else if (status == PATHEXEOBSTRUCTED || status == PATHEXECOLLISION)                   // obstruction ahead, must avoid
        {   if (status == PATHEXECOLLISION)                                                   // need to check for avatar
            {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
                integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
                pathMsg(PATH_MSG_WARN, "Collided with " + llList2String(details,1));
                if (pathfindingtype == OPT_AVATAR)                      // apologize if hit an avatar
                {   llSay(0,"Excuse me."); }
            }
            float dist = pathdistance(llGetPos(), gPatrolDestination, CHARACTER_WIDTH, CHARACTER_TYPE_A);  // measure distance to goal
            if (dist < gPathDistance)                                   // if we are making progress
            {   if (gAction == ACTION_PATROL)                       // if patrolling
                {   pathMsg(PATH_MSG_WARN,"Patrol stopped by obstruction, retrying.");
                    restart_patrol();
                    return;
                } else if (gAction == ACTION_PURSUE)
                {   pathMsg(PATH_MSG_WARN,"Pursue stopped by obstruction, retrying.");
                    restart_pursue();
                    return;
                }
            } else {                                                    // not getting better, do not retry
                pathMsg(PATH_MSG_WARN, "Distance to goal did not decrease. Now " + (string)dist + " was " + (string)gPathDistance);
            }
        }
        //  Default - errors we don't special case.
        {          
            if (gAction == ACTION_FACE)             // Can't get in front of avatar, greet anyway.
            {   pathMsg(PATH_MSG_WARN, "Can't navigate to point in front of avatar.");
                face_and_greet();
                return;
            }

            if (gTarget != NULL_KEY)
            {   pathMsg(PATH_MSG_WARN,"Unable to reach " + llKey2Name(gTarget) + " status: " + (string)status);
                gDeferredTargets += gTarget;    // defer action on this target
                gDeferredPositions += target_pos(gTarget);  // where they are
                gTarget = NULL_KEY;
            }
            
            //  Failed, back to idle.
            gAction = ACTION_IDLE;            
            pathMsg(PATH_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
            start_anim(IDLE_ANIM);
            return;
        }
    }
//
//  restart_patrol -- start or restart patrol
//
restart_patrol()
{
    gDwell = llList2Float(gPatrolPointDwell, gNextPatrolPoint);
    gFaceDir = llList2Float(gPatrolPointDir, gNextPatrolPoint);
    pathMsg(PATH_MSG_WARN,"Patrol to " + (string)gPatrolDestination);
    gPathDistance = pathdistance(llGetPos(), gPatrolDestination, CHARACTER_WIDTH, CHARACTER_TYPE_A);  // measure distance to goal
    start_anim(WAITING_ANIM);                     // applies only when stalled during movement
    pathNavigateTo(gPatrolDestination,0);           // head for next pos
    gAction = ACTION_PATROL;                        // patrolling
}
//
//  restart_pursue -- start or restart pursue
//
restart_pursue()
{   pathMsg(PATH_MSG_WARN,"Pursuing " + llKey2Name(gTarget));
    //  Temporary way to stop whatever is going on.
    llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);           // stop whatever is going on ***TEMP***
    gDwell = 0.0;
    list details = llGetObjectDetails(gTarget, [OBJECT_POS]);       // Where is avatar?
    vector goalpos = llList2Vector(details,0);                      // get object position
    gPathDistance = pathdistance(llGetPos(), goalpos, CHARACTER_WIDTH, CHARACTER_TYPE_A);  // measure distance to goal
    start_anim(WAITING_ANIM);                                       // applies only when stalled during movement
    llSleep(2.0);                                                   // allow stop time
    pathPursue(gTarget, GOAL_DIST*2);
    gAction = ACTION_PURSUE;
    llSetTimerEvent(1.0);                   // fast poll while moving
}

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
        restart_patrol();
    }  
}

//
//  startup - initialization
startup()
{   
    pathInit(CHARACTER_WIDTH, CHARACTER_HEIGHT, CHARACTER_TYPE_A, VERBOSITY);   // set up pathfinding system
    pathSpeed(CHARACTER_SPEED, CHARACTER_TURNSPEED_DEG*DEG_TO_RAD); // how fast to go
    gAction = ACTION_IDLE;
    gAnim = "";
    gScale = llGetScale();                  // scale of animesh
    gOwner = llGetOwner();                  // owner of animesh
    list groupdetails = llGetObjectDetails(llGetKey(), [OBJECT_GROUP]); // my group
    gGroup = llList2Key(groupdetails,0);    // group of animesh
    //  Start loading patrol points.
    gPatrolNotecardLine = 0;
    gPatrolPoints = [];
    gPatrolPointDwell = [];
    gNextPatrolPoint = 0;
    gDwell = 0.0;                           // dwell time after patrol point reached
    gPatrolEnabled = FALSE;                 // turns on when all points loaded
    //  Get our name
    gName = llGetObjectName();
    integer spaceIndex = llSubStringIndex(gName, " ");
    gName  = llGetSubString(gName, 0, spaceIndex - 1);       // first name of character
    if (llGetInventoryKey(PATROL_NOTECARD) == NULL_KEY)     // waypoints file no good
    {
        llSay(DEBUG_CHANNEL, "Notecard '" + PATROL_NOTECARD + "' missing or empty. Will not patrol.");
        return;
    }
    //  Start reading notecard. This may need a retry; dataserver is not reliable.
    gPatrolNotecardQuery = llGetNotecardLine(PATROL_NOTECARD, gPatrolNotecardLine);
    
    //  Set up character
    llSetTimerEvent(IDLE_POLL);                                 // check for work
    gListenChannel = llListen(PUBLIC_CHANNEL,"", "","");    // anybody talking to us?
    llOwnerSay("Restart.");
    start_anim(IDLE_ANIM);
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

    timer()                                     // timer tick
    {   pathTick();                             // timers in path library get updated
        //  Put name above character
        string msg = gName;                     // name of character
        vector color = <1.0,1.0,1.0>;
        llSetText(msg, color, 1.0);             // set text
        if (gAction == ACTION_IDLE && llGetTime() < ATTENTION_SPAN)
        {   return; }                           // talking to someone

        if (gAction == ACTION_IDLE || gAction == ACTION_PATROL) // begin pursuit
        {   
            float closestdist = 99999;              
            gTarget = NULL_KEY;                     
            integer i;
            integer targetix = -1;
            list newGreetedTargets;
            //  Get all agents on same owner parcels in region.
            list agents = llGetAgentList(AGENT_LIST_PARCEL_OWNER,[]);   
            integer num_detected = llGetListLength(agents);
            if (num_detected == 0)                      // nobody around
            {
                llSetTimerEvent(IDLE_POLL);             // back to slow mode
                if (gAction == ACTION_IDLE)
                {   start_patrol();                     // nothing to do, start patrolling
                }
                gGreetedTargets = [];                   // clear everything
                gDeferredTargets = [];
                gDeferredPositions = [];                      
                if (gTarget != NULL_KEY)
                {                   
                    pathMsg(PATH_MSG_INFO,"Forget old target.");
                } 
                gTarget = NULL_KEY;      
                return;
            }
            for (i=0; i < num_detected; i++)        
            {   key id = llList2Key(agents,i);           // agent to examine   
                integer j;
                for (j=0; j < llGetListLength(gGreetedTargets); j++) 
                {   if (id == llList2Key(gGreetedTargets,j))
                    {   newGreetedTargets += id;        // greeted target still in range
                        id = NULL_KEY;                  // do not re-greet this target
                    }
                }
                //  Check for avatar on deferred problem list. Skip if deferred and hasn't moved.
                for (j=0; j<llGetListLength(gDeferredTargets); j++)
                {   if (id == llList2Key(gDeferredTargets, j) 
                    && llVecMag(llList2Vector(gDeferredPositions, j) - target_pos(id)) < MIN_MOVE_FOR_RETRY)
                    {   id = NULL_KEY; }                // do not retry this one yet
                }
                if (!valid_dest(target_pos(id)))        // if in different parcel/out of region
                {   id = NULL_KEY;   }                  // ignore    
                if (id != NULL_KEY)
                {   float dist = llVecMag(vec_to_target(id));
                    if (dist > 0 && dist < closestdist) 
                    {   gTarget = id; 
                        targetix = j;                   // which index                  
                        closestdist = dist;
                    }
                }
            }
            gGreetedTargets = newGreetedTargets;        // only keep greeted targets still in range
            if (gTarget == NULL_KEY) {                  // no one to greet
                llSetTimerEvent(IDLE_POLL);             // back to slow polls
                start_patrol();                         // consider patrolling
            } else {
                //  New target found. Go to them.
                gAction = ACTION_PURSUE;
                gDwell = 0.0;                             // not relevant in this mode
                ////pathUpdateCharacter(CHAR_PARAMS + CHAR_AVOID_DISTANT); // avatar avoidance
                restart_pursue();
               ////pathPursue(gTarget, GOAL_DIST*2);
                llSetTimerEvent(1.0);                   // fast poll while moving
                // Remove pursue target from to-do list.
                gDeferredTargets = llDeleteSubList(gDeferredTargets, targetix, targetix);
                gDeferredPositions =llDeleteSubList(gDeferredPositions, targetix, targetix);                   
            }
        }
    }
    
    link_message(integer sender_num, integer num, string str, key id)
    {   pathLinkMsg(sender_num, num, str, id); }
   
    
    listen( integer channel, string name, key id, string msg)
    {   ////llOwnerSay("Listen from id " + (string) id + ": " + msg); // ***TEMP***
        if (gAction == ACTION_IDLE && llGetAgentSize(id) != ZERO_VECTOR)    // stay around if talked to by an avi
        {   
            face_dir(vec_to_target(id));                 // face who's talking
            llResetTime();                                  // reset attention span
        }
    }
    
    dataserver(key query_id, string data)
    {
        if (query_id == gPatrolNotecardQuery)           // reading patrol points from notecard
        {
            if (data == EOF)                            // done reading notecard
            {   llOwnerSay("Patrol points loaded."); 
                if (llGetListLength(gPatrolPoints) > 0)
                {
                    gPatrolEnabled = TRUE;              // start patrolling
                } else {
                    llSay(DEBUG_CHANNEL,"'" + PATROL_NOTECARD + "' did not contain any valid patrol points.");
                }
            }
            else
            {
                // Get next line, another dataserver request
                ++gPatrolNotecardLine;
                add_patrol_point(data);             // parse and add patrol point
                gPatrolNotecardQuery = llGetNotecardLine(PATROL_NOTECARD, gPatrolNotecardLine);
            }
        }
    }    
}

