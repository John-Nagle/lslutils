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
#include "npc/pathavatartrackcall.lsl"

integer ACTION_IDLE = 0;         
integer ACTION_PURSUE = 1;
integer ACTION_FACE = 2;
integer ACTION_PATROL = 3;
integer ACTION_TALKING = 4;                 // facing an avi and talking


//  Configuration
float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.7;            // (fract) Retry if random < this.
float TESTSPACING = 0.33;                   // (fract) Multiply height and width by this to get ray cast spacing

//  Character dimensions
integer CHARTYPE = CHARACTER_TYPE_A;                        // humanoid

float CHARACTER_WIDTH = 0.5;
float CHARACTER_HEIGHT = 2.2;
float CHARACTER_SPEED = 2.5;                // (m/sec) speed
float CHARACTER_TURNSPEED_DEG = 90.0;       // (deg/sec) turn rate
////string IDLE_ANIM = "stand 2";               // idle or chatting         
////string STAND_ANIM = "stand 2";              // just when stopped
///string WAITING_ANIM = "stand arms folded";  // during planning delays
string WAITING_ANIM = "SEmotion-bento13";   // arms folded during planning delays
string IDLE_ANIM = "SEmotion-bento18";      // arms folded during planning delays
string STAND_ANIM = "SEmotion-bento18";     // just when stopped
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
#ifdef OBSOLETE
list gGreetedTargets = [];      // we have said hello
list gDeferredTargets = [];     // trouble with these, wait and retry
list gDeferredPositions = [];   // don't retry until target moves
#endif // OBSOLETE
integer gListenChannel;         // for detecting chat
vector gScale;                  // scale of character
key gOwner;                     // owner of character
key gGroup;                     // group of character
string gName;                   // our name

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
//  add_patrol_point -- add patrol point from notecard
//
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
{   pathMsg(PATH_MSG_INFO, "Path update: : " + (string) status + " obstacle: " + llKey2Name(hitobj));
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
                pathNavigateTo(finaltarget, 0);
                llResetTime();
            } else {
                pathMsg(PATH_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                start_anim(IDLE_ANIM);
                face_and_greet("Hello");
            }                             
            return;
        }
        if (gAction == ACTION_FACE)
        {   //  Turn to face avatar.
            start_anim(IDLE_ANIM);
            face_and_greet("Hello");
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
    //  If had trouble getting there, but got close enough, maybe we can just say hi.
    if ((gAction == ACTION_PURSUE || gAction == ACTION_FACE) && dist_to_target(gTarget) < MAX_GREET_DIST) // if close enough to greet
    {   start_anim(IDLE_ANIM);
        face_and_greet("Hello there.");                         // longer range greeting
        return;
    }
    //  If blocked by something, deal with it.
    if ((gAction == ACTION_PURSUE || gAction == ACTION_PATROL))             // if going somewhere
    {   if (is_active_obstacle(hitobj))                                 // and it might move
        {   if (llFrand(1.0) < OBSTACLE_RETRY_PROB)                     // if random test says retry
            {   pathMsg(PATH_MSG_WARN,"Obstacle " + llKey2Name(hitobj) + ",will try again.");
                if (gAction == ACTION_PURSUE)                           // try again.
                {   start_pursue(); }
                else 
                {   
                    gDwell = 0.0;
                }
            } else {                                                    // no retry, give up now
                gDwell = 0.0;                                           // no dwell time, do something else now
                pathMsg(PATH_MSG_WARN,"Obstacle, will give way.");
            }
        }
    }
    //  Default - errors we don't special case.
    {          
        if (gAction == ACTION_FACE)             // Can't get in front of avatar, greet anyway.
        {   pathMsg(PATH_MSG_WARN, "Can't navigate to point in front of avatar.");
            start_anim(IDLE_ANIM);
            face_and_greet("Hello");
            return;
        }

        if (gTarget != NULL_KEY)
        {   pathMsg(PATH_MSG_WARN,"Unable to reach " + llKey2Name(gTarget) + " status: " + (string)status);
            pathavatartrackreply(gTarget,"defer");          // tell tracker to defer action on this avi
            ////gDeferredTargets += gTarget;    // defer action on this target
            ////gDeferredPositions += target_pos(gTarget);  // where they are
            gTarget = NULL_KEY;                             // not tracking anyone
        }
            
        //  Failed, back to idle.
        gAction = ACTION_IDLE;            
        pathMsg(PATH_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        start_anim(IDLE_ANIM);
        return;
    }
}
 
//
//  face_dir -- face in indicated direction
//
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

//  
//  face_and_greet -- face in indicated direction and say hello
//
face_and_greet(string msg)                          // turn to face avi
{
    face_dir(vec_to_target(gTarget));               // turn to face avi
    gAction = ACTION_TALKING;
    llResetTime();                                  // start attention span timer.
    gDwell = ATTENTION_SPAN;                        // wait this long
    llSay(0,msg);
}

start_anim(string anim)
{
    llMessageLinked(LINK_THIS, 1,anim,"");    // tell our AO what we want
}    

//
//  start_pursue -- start pursuing avatar.
//
start_pursue()
{   pathMsg(PATH_MSG_WARN,"Pursuing " + llKey2Name(gTarget));
    gDwell = 0.0;
    start_anim(WAITING_ANIM);                                       // applies only when stalled during movement
    llSleep(2.0);                                                   // allow stop time
    pathPursue(gTarget, GOAL_DIST*2, TRUE);
    gAction = ACTION_PURSUE;
    llSetTimerEvent(1.0);                                           // fast poll while moving

}

//
//  requestpursue -- a request to pursue has arrived.
//
//  Start it if not already pursuing someone.
//
requestpursue(key target)
{   if (!((gAction == ACTION_IDLE || gAction == ACTION_PATROL)))    // busy doing someone else
    {   pathavatartrackreply(target, "busy");                       // tell the tracker to be quiet for a while.
        return;
    }
    gTarget = target;                                               // set new target
    start_pursue();                                                 // go for it
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
        restart_patrol();
    }  
}
//
//  restart_patrol -- start patrol to previously selected point.
//
restart_patrol()
{
    pathMsg(PATH_MSG_WARN,"Patrol to " + (string)gPatrolDestination);
    start_anim(WAITING_ANIM);                     // applies only when stalled during movement
    pathNavigateTo(gPatrolDestination,0);           // head for next pos
    gAction = ACTION_PATROL;                        // patrolling
    llSetTimerEvent(1.0);                       // fast poll while moving
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
    string msg = gName;                     // name of character
    vector color = <1.0,1.0,1.0>;           // white
    llSetText(msg, color, 1.0);             // set hover text
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
    {   pathTick();                                 // timers in path library get updated
        if (gAction == ACTION_TALKING)              // if talking to an avi
        {   if (llGetTime() < ATTENTION_SPAN)       // if clock has not run out
            {   return; }                           // do nothing
            //  Our patience is limited.
            pathavatartrackreply(gTarget,"done");   // tell tracker done with this avi.
            gTarget = NULL_KEY;
            gAction = ACTION_IDLE;                  // back to idle mode
        }
        if (gAction == ACTION_IDLE)                 // no one to greet
        {   llSetTimerEvent(IDLE_POLL);             // back to slow polls
            start_patrol();                         // consider patrolling
        }
    }
    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   ////pathMsg(PATH_MSG_INFO, jsn);                        // ***TEMP*** dump incoming JSON
        if (num == PATHAVATARTRACKREQUEST)                  // if avatar tracker wants us to track an avatar
        {   if (llJsonGetValue(jsn,["request"]) != "trackavi") { return; } // not for us
            requestpursue(llJsonGetValue(jsn,["id"]));      // go pursue, if appropriate.
            return; 
        } 
        pathLinkMsg(sender_num, num, jsn, id);
    }
    
    collision_start(integer num_detected)
    {   key hitobj = llDetectedKey(0);                       // first object hit
        list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
        integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
        pathMsg(PATH_MSG_WARN, "Collided with " + llList2String(details,1));
        if (pathfindingtype == OPT_AVATAR)                      // apologize if hit an avatar
        {   llSay(0,"Excuse me."); }
    }
   
    
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

