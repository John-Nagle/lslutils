//
//  bhvgreet.lsl  --  greeter behavior for Animats pathfinding system
//  
//  Demo for Animats pathfinding system.
//
//  Approaches any new avatar and says hello.
//  Otherwise moves between patrol points.
//
//  License: GPLv3.
//
//  Animats
//  November 2019
//
//  ***STARTING -- NEEDS MUCH WORK***
//
#include "npc/bhvcall.lsl"
#include "npc/pathavatartrackcall.lsl"
#include "npc/mathutils.lsl"
integer ACTION_IDLE = 0;         
integer ACTION_PURSUE = 1;
integer ACTION_FACE = 2;
////integer ACTION_PATROL = 3;
integer ACTION_TALKING = 4;                 // facing an avi and talking

#ifdef OBSOLETE

//  Configuration
float DETECTION_RADIUS = 60.0;      
float GOAL_TOL = 1.0;               
float TESTSPACING = 0.33;                   // (fract) Multiply height and width by this to get ray cast spacing

//  Character dimensions
integer CHARTYPE = CHARACTER_TYPE_A;                        // humanoid

#define CHARACTER_WIDTH  0.5
#define CHARACTER_HEIGHT  (gScale.z)        // use Z height as character height
#endif // OBSOLETE
float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.7;            // (fract) Retry if random < this.




#define PRIORITYPURSUE  3                   // higher than patrol, lower than evade vehicle

#ifndef CHARACTER_SPEED                     // overrideable
#define CHARACTER_SPEED  2.5                // (m/sec) speed
#endif // CHARACTER_SPEED
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
#define WALKSPEED 2.0                       // (m/sec) walking speed, non-run
////string IDLE_ANIM = "stand 2";            // idle or chatting         
////string STAND_ANIM = "stand 2";           // just when stopped
///string WAITING_ANIM = "stand arms folded";  // during planning delays
string WAITING_ANIM = "SEmotion-bento13";   // arms folded during planning delays
string IDLE_ANIM = "SEmotion-bento18";      // arms folded during planning delays
string STAND_ANIM = "SEmotion-bento18";     // just when stopped
string TALK_ANIM = IDLE_ANIM;               // for now
float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                  // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;            // must move at least this far before we recheck on approach

//  Configuration

integer PATH_STALL_TIME = 300;              // path stall time


//  Global variables
integer gAction = ACTION_IDLE;
string gAnim;                   // current animation                       
key gTarget = NULL_KEY;         // current avatar to pursue
integer gListenChannel;         // for detecting chat
float gDwell;                   // dwell time to wait
float gFaceDir;                 // direction to face next



//
//  bhvDoRequestDone -- pathfinding is done. Analyze the result and start the next action.
//
bhvDoRequestDone(integer status, key hitobj)
{   debugMsg(DEBUG_MSG_INFO, "Path update: : " + (string) status + " obstacle: " + llKey2Name(hitobj));
    if (status == PATHERRMAZEOK)          // success
    {   ////llOwnerSay("Pathfinding task completed.");
        if (gAction == ACTION_PURSUE)               
        {
            gAction = ACTION_FACE;  
            vector offset = dir_from_target(gTarget) * GOAL_DIST; 
            vector finaltarget = target_pos(gTarget) + offset;
            if (is_clear_sightline(gTarget, finaltarget))
            {            
                debugMsg(DEBUG_MSG_INFO,"Get in front of avatar. Move to: " + (string)finaltarget);
                bhvNavigateTo(ZERO_VECTOR, finaltarget, 0, WALKSPEED);
            } else {
                debugMsg(DEBUG_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                bhvAnimate([IDLE_ANIM]);            // should be greet anim
                face_and_greet("Hello");
            }                             
            return;
        }
        if (gAction == ACTION_FACE)
        {   //  Turn to face avatar.
            bhvAnimate([TALK_ANIM]);
            face_and_greet("Hello");
            return;
        } 
        //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        greet_done("defer");
        return;
        return;
    }
    //  If had trouble getting there, but got close enough, maybe we can just say hi.
    if ((gAction == ACTION_PURSUE || gAction == ACTION_FACE) && dist_to_target(gTarget) < MAX_GREET_DIST) // if close enough to greet
    {   bhvAnimate([TALK_ANIM]);
        face_and_greet("Hello there.");                         // longer range greeting
        return;
    }
    //  If blocked by something, deal with it.
    if ((gAction == ACTION_PURSUE))                                     // if going somewhere
    {   if (is_active_obstacle(hitobj))                                 // and it might move
        {   if (llFrand(1.0) < OBSTACLE_RETRY_PROB)                     // if random test says retry
            {   debugMsg(DEBUG_MSG_WARN,"Obstacle " + llKey2Name(hitobj) + ",will try again.");
                if (gAction == ACTION_PURSUE)                           // try again.
                {   start_pursue(); }
                else 
                {   
                    gDwell = 0.0;
                }
            } else {                                                    // no retry, give up now
                gDwell = 0.0;                                           // no dwell time, do something else now
                debugMsg(DEBUG_MSG_WARN,"Obstacle, will give way.");
            }
        }
    }
    //  Default - errors we don't special case.
    {          
        if (gAction == ACTION_FACE)                                     // Can't get in front of avatar, greet anyway.
        {   debugMsg(DEBUG_MSG_WARN, "Can't navigate to point in front of avatar.");
            bhvAnimate([IDLE_ANIM]);
            face_and_greet("Hello");
            return;
        }

        if (gTarget != NULL_KEY)
        {   debugMsg(DEBUG_MSG_WARN,"Unable to reach " + llKey2Name(gTarget) + " status: " + (string)status);
            greet_done("defer");
            return;
        }
            
        //  Failed, back to idle.
        gAction = ACTION_IDLE;            
        debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        greet_done("defer");
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
    bhvAnimate([STAND_ANIM]);
    if (currentlookdir * lookdir < 0.95)            // if need to turn to face
    {
        bhvTurn(lookdir);                           // face avatar
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

   
//
//  start_pursue -- start pursuing avatar.
//
start_pursue()
{   debugMsg(DEBUG_MSG_WARN,"Pursuing " + llKey2Name(gTarget));
    gDwell = 0.0;
    bhvAnimate([WAITING_ANIM]);                                     // applies only when stalled during movement
    llSleep(2.0);                                                   // allow stop time
    bhvPursue(gTarget, GOAL_DIST*2, TRUE);
    gAction = ACTION_PURSUE;
    llSetTimerEvent(1.0);                                           // fast poll while moving
}
//
//  greet_done -- done here, return to other tasks
//
greet_done(string action)
{   gAction = ACTION_IDLE;                                          // done here
    bhvAnimate([IDLE_ANIM]);                                        // back to idle anim
    if (action != "") { pathavatartrackreply(gTarget,action); }     // tell tracker to defer action on this avi
    bhvSetPriority(0);
}

//
//  requestpursue -- a request to pursue has arrived.
//
//  Start it if not already pursuing someone.
//
requestpursue(key target)
{   if (!(gAction == ACTION_IDLE))                                  // busy doing someone else
    {   pathavatartrackreply(target, "busy");                       // tell the tracker to be quiet for a while.
        return;
    }
    gTarget = target;                                               // set new target
    bhvSetPriority(PRIORITYPURSUE);                                 // request control of NPC
}

//
//  bhvDoStart -- we now have control of NPC
//
bhvDoStart()
{
    gAction = ACTION_IDLE;                                          // whatever we were doing is cancelled
    bhvAnimate([IDLE_ANIM]);                                        // back to idle anim
    if (gTarget == NULL_KEY)                                        // if we have no one to pursue
    {
        bhvSetPriority(0);                                          // we don't need to run now, give up control
        return;
    }
    llResetTime();                                                  // reset the clock
    start_pursue();
}
//
//  bhvDoStop -- this behavior no longer has control
//
bhvDoStop()
{
    gAction = ACTION_IDLE;                                          // we've been preempted. No pursue now.
    gTarget = NULL_KEY;                                             // need a new target later
    llSetTimerEvent(0);                                             // don't need a timer
}

#ifdef OBSOLETE
//
//  startup - initialization
startup()
{   gScale = llGetScale();                  // scale of animesh
    llOwnerSay("Character height: " + (string)CHARACTER_HEIGHT);    // ***TEMP***
    pathInit(CHARACTER_WIDTH, CHARACTER_HEIGHT, CHARACTER_TYPE_A, VERBOSITY);   // set up pathfinding system
    pathSpeed(CHARACTER_SPEED, CHARACTER_TURNSPEED_DEG*DEG_TO_RAD); // how fast to go
    gAction = ACTION_IDLE;
    gAnim = "";
    gOwner = llGetOwner();                  // owner of animesh
    list groupdetails = llGetObjectDetails(llGetKey(), [OBJECT_GROUP]); // my group
    gGroup = llList2Key(groupdetails,0);    // group of animesh
}
#endif // OBSOLETE

default
{
    on_rez(integer start_param)
    {
        llResetScript();
    }
 
    state_entry()
    {
        gDebugMsgLevel = DEBUG_MSG_INFO;                // ***TEMP*** need way to set debug level dynamically
        bhvInit();
    }

    timer()                                         // timer tick
    {   bhvTick();                                 // timers in path library get updated
        if (gAction == ACTION_TALKING)              // if talking to an avi
        {   if (llGetTime() < ATTENTION_SPAN)       // if clock has not run out
            {   return; }                           // do nothing
            //  Our patience is limited.
            greet_done("done");                     // tell tracker done with this avi.
        }
    }
    
    link_message(integer sender_num, integer num, string jsn, key id)
    {   debugMsg(DEBUG_MSG_INFO, jsn);                      // ***TEMP*** dump incoming JSON
        if (num == gBhvMnum || num == BHVMSGFROMSCH)        // if from scheduler to us
        {   
            bhvSchedMessage(num,jsn);                       // message from scheduler
            return;
        }
        if (num == PATHAVATARTRACKREQUEST)                  // if avatar tracker wants us to track an avatar
        {   if (llJsonGetValue(jsn,["request"]) != "trackavi") { return; } // not for us
            requestpursue((key)llJsonGetValue(jsn,["id"])); // go pursue, if appropriate.
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
   
    
    listen(integer channel, string name, key id, string msg)
    {   ////llOwnerSay("Listen from id " + (string) id + ": " + msg); // ***TEMP***
        if (gAction == ACTION_IDLE && llGetAgentSize(id) != ZERO_VECTOR)    // stay around if talked to by an avi
        {   
            bhvTurn(vec_to_target(id));                 // face who's talking
            llResetTime();                                  // reset attention span
        }
    }
}

