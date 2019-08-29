//
//  Pathfinding controller
//
//  Approaches any new avatar and says hello.
//
//  Animats
//  2019
//
#include "pathcalls.lsl"


//  Configuration
string PATROL_NOTECARD = "Patrol points";   // read this notecard for patrol points
float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float GOAL_DIST = 2.0;                      // get this close to talk
float OUR_CHARACTER_RADIUS = 0.125;          // just for line of sight text
string IDLE_ANIM = "stand 2";    // idle or chatting         
string STAND_ANIM = "stand 2"; // just when stopped
float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                   // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;             // must move at least this far before we recheck on approach

integer PATH_STALL_TIME = 300;              // path stall time
////integer VERBOSITY = PATH_MSG_WARN;          // warning messages only
integer VERBOSITY = PATH_MSG_WARN;          // debug
list CHAR_PARAMS = [ CHARACTER_MAX_SPEED, 2.0,
                     CHARACTER_DESIRED_SPEED, 1.5,
                     CHARACTER_DESIRED_TURN_SPEED, 0.25,
                     CHARACTER_MAX_TURN_RADIUS, 0.25,
                     CHARACTER_TYPE, CHARACTER_TYPE_A,
                     CHARACTER_LENGTH, 2.0,
                     CHARACTER_ACCOUNT_FOR_SKIPPED_FRAMES, TRUE,
                     //// CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS | AVOID_DYNAMIC_OBSTACLES,
                     CHARACTER_STAY_WITHIN_PARCEL, FALSE,
                     CHARACTER_RADIUS, OUR_CHARACTER_RADIUS
                     ];
                     
//  Add one of these to the above.               
list CHAR_AVOID_DISTANT;
list CHAR_AVOID_CLOSE; // no avatar avoidance on close approach
 
//  Constants
float REGION_SIZE = 256.0;                  // size of a region   

integer ACTION_IDLE = 0;            
integer ACTION_PURSUE = 1;
integer ACTION_FACE = 2;
integer ACTION_PATROL = 3;

//  Global variables
integer gAction = ACTION_IDLE;
string gAnim;                   // current animation                       
key gTarget = NULL_KEY;
list gGreetedTargets = [];      // we have said hello
list gDeferredTargets = [];     // trouble with these, wait and retry
list gDeferredPositions = [];   // don't retry until target moves
integer gListenChannel;         // for detecting chat
vector gScale;                  // scale of character
key gOwner;                     // owner of character
key gGroup;                     // group of character
string gName;                   // name of character (first word)

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

//  Usual SLERP function for quaternion interpolation
rotation slerp(rotation a, rotation b, float t) {
   return llAxisAngle2Rot( llRot2Axis(b /= a), t * llRot2Angle(b)) * a;
}    

integer rand_int(integer bound)                 // bound must not exceed 2^24.
{   return((integer)llFrand(bound)); }          // get random integer                   

vector vec_to_target(key id)         
{   return(target_pos(id) - llGetPos());   }         

vector target_pos(key id)
{   list v = llGetObjectDetails(id, [OBJECT_POS]);  
    if (v == []) { return(ZERO_VECTOR); }         // not really a good choice for fails  
    return(llList2Vector(v,0));                             
}

vector dir_from_target(key id)                      
{
    list r = llGetObjectDetails(id, [OBJECT_ROT]);  
    if (r == []) { return(ZERO_VECTOR); }           
    rotation arot = llList2Rot(r,0);                
    vector facingdir = <1,0,0>*arot;
    facingdir.z = 0.0;                              
    return(llVecNorm(facingdir));                   
}

//
//  valid_dest - is target valid (same sim and same parcel ownership?)
//
integer valid_dest(vector pos) 
{
    if (pos.x <= 0 || pos.x >= REGION_SIZE || pos.y <= 0 || pos.y >= REGION_SIZE) { return(FALSE); }
    list theredata = llGetParcelDetails(pos, [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP]);
    list heredata = llGetParcelDetails(llGetPos(), [PARCEL_DETAILS_OWNER, PARCEL_DETAILS_GROUP]);
    integer thereflags = llGetParcelFlags(pos);         // flags for dest parcel
    key thereowner = llList2Key(theredata,0);
    key theregroup = llList2Key(theredata,1);
    if ((llList2Key(heredata,0) != thereowner) // dest parcel must have same ownership
    && (llList2Key(heredata,1) != theregroup))
    {   return(FALSE); } // different group and owner at dest parcel
    //  Check for no-script area
    if (thereflags & PARCEL_FLAG_ALLOW_SCRIPTS == 0)            // if scripts off for almost everybody
    {   if (gOwner != thereowner)
        {   if ((thereflags & PARCEL_FLAG_ALLOW_GROUP_SCRIPTS == 0) || (gGroup != theregroup))
            { return(FALSE); }                                  // would die
        }
    }                                // no script area, we would die
    //  Can we enter the destination parcel?
    if (thereflags && PARCEL_FLAG_ALLOW_ALL_OBJECT_ENTRY == 0) 
    {    if (gOwner == thereowner) { return(TRUE); } // same owner, OK
        {   if ((thereflags & PARCEL_FLAG_ALLOW_GROUP_OBJECT_ENTRY) || (gGroup != theregroup))
            { return(FALSE); }
        }
    }
    return(TRUE);  // OK to enter destination parcel
}

start_anim(string anim)
{
    llMessageLinked(LINK_THIS, 1,anim,"");    // tell our AO what we want
}


//  Check if point is directly visible in a straight line.
//  Used when trying to get in front of an avatar.
integer clear_sightline(key id, vector lookatpos)
{   list obstacles = llCastRay(target_pos(id), lookatpos,[]);
    integer status = llList2Integer(obstacles,-1);
    if (status == 0) { return(TRUE); }      // no errors, zero hits, clear sightline.
    pathMsg(PATH_MSG_WARN, "Clear sightline status " + (string)status + " hits: " + (string)obstacles);
    return(FALSE);                          // fails
}
face_dir(vector lookdir)                        // face this way
{
    lookdir = llVecNorm(lookdir);
    vector currentlookdir = <1,0,0>*llGetRot();
    if (currentlookdir * lookdir < 0.95)             // if need to turn to face
    {
        start_anim(STAND_ANIM);
        pathFaceInDirection(lookdir);       // face avatar
        start_anim(IDLE_ANIM);
    }
} 

face_and_greet()                            // turn to face avi
{
    face_dir(vec_to_target(gTarget));                   // turn to face avi
    gGreetedTargets += gTarget;                         
    gAction = ACTION_IDLE;
    llResetTime();              // start attention span timer.
    gDwell = ATTENTION_SPAN;    // wait this long
    llSay(0,"Hello");
}


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
        + "Face dir: " + (string)facedir); 
    gPatrolPoints += pos;
    gPatrolPointDwell += dwell;
    gPatrolPointDir += facedir;
}

pathUpdateCallback(integer type, list reserved )
    {   ////llOwnerSay("Path update: : " + (string) type);
        if (type == PATHSTALL_UNINITIALIZED)
        {   pathMsg(PATH_MSG_WARN, "Scripts out of sync. Restarting."); 
            startup();  // we are out of sync. Redo startup. 
            return;
        }
        
        if (type == PU_FAILURE_UNREACHABLE
        || type == PU_FAILURE_INVALID_GOAL
        || type == PU_FAILURE_INVALID_START
        || type == PU_FAILURE_TARGET_GONE
        || type == PU_FAILURE_NO_VALID_DESTINATION
        || type == PU_FAILURE_NO_NAVMESH
        || type == PATHSTALL_STALLED 
        || type == PATHSTALL_CANNOT_MOVE 
        || type == PATHSTALL_NOPROGRESS
        || type == PATHSTALL_UNREACHABLE)
        {  
        
            if (gAction == ACTION_FACE)             // Can't get in front of avatar, greet anyway.
            {   pathMsg(PATH_MSG_WARN, "Can't navigate to point in front of avatar.");
                face_and_greet();
                return;
            }

            if (gTarget != NULL_KEY)
            {   pathMsg(PATH_MSG_INFO,"Unable to reach " + llKey2Name(gTarget));
                gDeferredTargets += gTarget;    // defer action on this target
                gDeferredPositions += target_pos(gTarget);  // where they are
                gTarget = NULL_KEY;
            }
            
            //  Failed, back to idle.
            gAction = ACTION_IDLE;            
            pathMsg(PATH_MSG_WARN,"Failed to reach goal, idle. Path update status: " + pathErrMsg(type));
            start_anim(IDLE_ANIM);
            return;
        }
        if (type == PU_GOAL_REACHED)          // success
        {   ////llOwnerSay("Pathfinding task completed.");
            if (gAction == ACTION_PURSUE)               
            {
                gAction = ACTION_FACE;  
                vector offset = dir_from_target(gTarget) * GOAL_DIST; 
                vector finaltarget = target_pos(gTarget) + offset;
                if (clear_sightline(gTarget, finaltarget))
                {            
                    pathMsg(PATH_MSG_INFO,"Get in front of avatar. Move to: " + (string)finaltarget);
                    pathUpdateCharacter(CHAR_PARAMS + CHAR_AVOID_CLOSE); // no avatar avoidance
                    pathNavigateTo(finaltarget, []);
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
        }
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
        gDwell = llList2Float(gPatrolPointDwell, gNextPatrolPoint);
        gFaceDir = llList2Float(gPatrolPointDir, gNextPatrolPoint);
        pathMsg(PATH_MSG_INFO,"Patrol to " + (string)gPatrolDestination);
        pathUpdateCharacter(CHAR_PARAMS + CHAR_AVOID_DISTANT); // avatar avoidance 
        pathNavigateTo(gPatrolDestination,[]);            // head for next pos
        gAction = ACTION_PATROL;                        // patrolling
    }  
}

startup()
{   //  Constant intitialization
    CHAR_AVOID_DISTANT = [CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS | AVOID_DYNAMIC_OBSTACLES];
    CHAR_AVOID_CLOSE = [CHARACTER_AVOIDANCE_MODE, AVOID_CHARACTERS]; 
    pathInit();                             // init path library
    gPathMsgLevel = VERBOSITY;              // logging level
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
    if (llGetInventoryKey(PATROL_NOTECARD) == NULL_KEY)
    {
        llSay(DEBUG_CHANNEL, "Notecard '" + PATROL_NOTECARD + "' missing or empty. Will not patrol.");
        return;
    }
    //  Start reading notecard. This may need a retry; dataserver is not reliable.
    gPatrolNotecardQuery = llGetNotecardLine(PATROL_NOTECARD, gPatrolNotecardLine);
    
    //  Set up character
    llDeleteCharacter();
    pathCreateCharacter(CHAR_PARAMS + CHAR_AVOID_DISTANT);    // set up, don't approach too closely
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
        //  Name above character
        string msg = gName;                     // name of character
        vector color = <1.0,1.0,1.0>;
        float pathfindingpct = llGetSimStats(SIM_STAT_PCT_CHARS_STEPPED);// check for sim in severe overload
        if (pathfindingpct < 25)                // if severe overload
        {   msg = msg + "\nSim overload - " + (string)((integer)pathfindingpct) + "% speed.";
            color = <1.0, 0.0, 0.0>;            // red msg
        }
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
                pathMsg(PATH_MSG_INFO,"Pursuing " + llKey2Name(gTarget));
                gAction = ACTION_PURSUE;
                gDwell = 0.0;                             // not relevant in this mode
                pathUpdateCharacter(CHAR_PARAMS + CHAR_AVOID_DISTANT); // avatar avoidance 
                pathPursue(gTarget, [PURSUIT_GOAL_TOLERANCE,
                    GOAL_DIST*2, REQUIRE_LINE_OF_SIGHT, FALSE]);
                llSetTimerEvent(1.0);                   // fast poll while moving
                // Remove pursue target from to-do list.
                gDeferredTargets = llDeleteSubList(gDeferredTargets, targetix, targetix);
                gDeferredPositions =llDeleteSubList(gDeferredPositions, targetix, targetix);                   
            }
        }
    }
        
    path_update(integer type, list reserved)
    {   pathUpdate(type, reserved);                         // pass to library
    }
    
    collision_start(integer num_detected)  
    {   pathCollide(num_detected);                          // pass to library
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

