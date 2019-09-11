//
//  memoryleaktest.lsl 
//
//  UNIT TEST only.
//  
//  Demo for Animats pathfinding system.
//
//  This is a stripped-down version of the patroller demo
//  to check for memory leak problems. It just starts
//  random path plans, which are never used.
//
//  License: GPLv3.
//
//  Animats
//  2019
//
#include "npc/pathcall.lsl"



//  Configuration
float GOAL_TOL = 1.0;               
float GOAL_DIST = 1.75;                      // get this close to talk
//  Character dimensions
float CHARACTER_WIDTH = 0.5;
float CHARACTER_HEIGHT = 2.2;
float CHARACTER_SPEED = 2.5;                // (m/sec) speed
float CHARACTER_TURNSPEED_DEG = 90.0;       // (deg/sec) turn rate
string IDLE_ANIM = "stand 2";               // idle or chatting         
string STAND_ANIM = "stand 2";              // just when stopped
string WAITING_ANIM = "stand arms folded";  // during planning delays
float IDLE_POLL = 15.0;
#ifndef VERBOSITY                           // define VERBOSITY to override
#define VERBOSITY PATH_MSG_ERROR            // verbose
#endif // VERBOSITY

//  Configuration
string PATROL_NOTECARD = "Patrol points";   // read this notecard for patrol points

integer PATH_STALL_TIME = 300;              // path stall time


//  Global variables
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
                pathNavigateTo(finaltarget, 0);
                llResetTime();
            } else {
                pathMsg(PATH_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                start_anim(IDLE_ANIM);
                face_and_greet();
            }                             
            return;
        }
        if (gAction == ACTION_FACE)
        {   //  Turn to face avatar.
            start_anim(IDLE_ANIM);
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
    //  Default - errors we don't special case.
    {          
        if (gAction == ACTION_FACE)             // Can't get in front of avatar, greet anyway.
        {   pathMsg(PATH_MSG_WARN, "Can't navigate to point in front of avatar.");
            start_anim(IDLE_ANIM);
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
 





start_patrol()
{   //  Start patrolling if nothing else to do
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
        pathMsg(PATH_MSG_WARN,"Patrol to " + (string)gPatrolDestination);
        pathNavigateTo(gPatrolDestination,0);           // head for next pos
    }  
}

//
//  startup - initialization
startup()
{   
    pathInit(CHARACTER_WIDTH, CHARACTER_HEIGHT, CHARACTER_TYPE_A, VERBOSITY);   // set up pathfinding system
    pathSpeed(CHARACTER_SPEED, CHARACTER_TURNSPEED_DEG*DEG_TO_RAD); // how fast to go
    gScale = llGetScale();                  // scale of animesh
    gOwner = llGetOwner();                  // owner of animesh
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
    
    llOwnerSay("Restart.");
    llSetTimerEvent(15.0);      // run a test this often
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
    {   start_patrol();   }
    
    link_message(integer sender_num, integer num, string str, key id)
    {    }
    
     
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

