//
//  bhvgreet.lsl  --  greeter behavior for Animats pathfinding system
//  
//  Demo for Animats pathfinding system.
//
//  Approaches any new avatar and says hello.
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
integer ACTION_FACE = 2;                    // turn to face target, has callback
integer ACTION_GREET = 4;                   // face finished, do greet
integer ACTION_DISTANT_GREET = 5;           // face finished, do distant greet msg
integer ACTION_TALKING = 6;                 // facing an avi and talking

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
                llResetTime();
            } else {
                debugMsg(DEBUG_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                bhvAnimate([IDLE_ANIM]);
                face(gTarget, ACTION_DISTANT_GREET);
            }                             
            return;
        }
        if (gAction == ACTION_FACE)
        {   //  Turn to face avatar.
            bhvAnimate([IDLE_ANIM]);
            face(gTarget, ACTION_GREET);
            return;
        }
        if (gAction == ACTION_GREET)
        {   greet("Hello");
            return;
        }
        if (gAction == ACTION_DISTANT_GREET)                    // not face to face
        {   greet("Hello there");
            return;
        }
        if (gAction == ACTION_TALKING)              // if someone near is talking
        {   return;                                 // continue pretending to talk 
        }
        //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        gAction = ACTION_IDLE;            
        bhvAnimate([IDLE_ANIM]);
        return;
    }
    //  If had trouble getting there, but got close enough, maybe we can just say hi.
    if ((gAction == ACTION_PURSUE || gAction == ACTION_FACE) && dist_to_target(gTarget) < MAX_GREET_DIST) // if close enough to greet
    {   bhvAnimate([IDLE_ANIM]);
        face(gTarget, ACTION_DISTANT_GREET);
        ////face_and_greet("Hello there.");                         // longer range greeting
        return;
    }
    //  If blocked by something, deal with it.
    if ((gAction == ACTION_PURSUE))                                     // if going somewhere
    {   if (is_active_obstacle(hitobj))                                 // and it might move
        {   
            if (llFrand(1.0) < OBSTACLE_RETRY_PROB)                     // if random test says retry
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
        if (gAction == ACTION_FACE)             // Can't get in front of avatar, greet anyway.
        {   debugMsg(DEBUG_MSG_WARN, "Can't navigate to point in front of avatar.");
            bhvAnimate([IDLE_ANIM]);
            face(gTarget, ACTION_GREET);
            return;
        }

        if (gTarget != NULL_KEY)
        {   debugMsg(DEBUG_MSG_WARN,"Unable to reach " + llKey2Name(gTarget) + " status: " + (string)status);
            pathavatartrackreply(gTarget,"defer");          // tell tracker to defer action on this avi
            gTarget = NULL_KEY;                             // not tracking anyone
        }
            
        //  Failed, back to idle.
        gAction = ACTION_IDLE;            
        debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        bhvAnimate([IDLE_ANIM]);
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
        bhvAnimate([STAND_ANIM]);
        pathFaceInDirection(lookdir);       // face avatar
        bhvAnimate([IDLE_ANIM]);
    }
} 

//  
//  face -- face in indicated direction
//
face(key target, integer nextstate)                 // turn to face avi
{
    vector dir = vec_to_target(target);             // direction to face
    float heading = llAtan2(dir.x,dir.y);           // direction to face as heading (0=north)
    bhvTurn(heading);                               // turn to face avi
    gAction = nextstate;                            // on completion, greet
}

//  
//  greet -- say hello
//
greet(string msg)                                   // turn to face avi
{
    gAction = ACTION_TALKING;                       // on completion, fake being in conversation
    llResetTime();                                  // start attention span timer.
    gDwell = ATTENTION_SPAN;                        // wait this long
    bhvSay(msg);
}

   
//
//  start_pursue -- start pursuing avatar.
//
start_pursue()
{   debugMsg(DEBUG_MSG_WARN,"Pursuing " + llKey2Name(gTarget));
    gDwell = 0.0;
    bhvAnimate([WAITING_ANIM]);                                     // applies only when stalled during movement
    llSleep(2.0);                                                   // allow stop time
    bhvPursue(gTarget, GOAL_DIST*2, CHARACTER_SPEED);               // go after the avatar, stopping a bit short
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
{   if (!gBhvRegistered) { return; }                                // scheduler init still in progress, wait, will be repolled
    if (!(gAction == ACTION_IDLE))                                  // busy doing someone else
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

//
//  bhvDoCollisionStart -- a collision has occured.
//
//  We don't actually have to do anything about the collision except apologise for it.
//  The path system will stop for this, and pathstart will retry.
//  Only avatars and physical objects generate collisions, because we're keyframe motion.
//
bhvDoCollisionStart(key hitobj)
{       
    list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE, OBJECT_NAME]);
    integer pathfindingtype = llList2Integer(details,0);        // get pathfinding type
    debugMsg(DEBUG_MSG_WARN, "Collided with " + llList2String(details,1));
    if (pathfindingtype == OPT_AVATAR)                          // apologize if hit an avatar
    {   bhvSay("Excuse me."); }         
}    

//
// bhvRegistered -- scheduler is ready to run us.
//
bhvRegistered()                                                     // tell controlling script to go
{
}


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
        llSetText("", <1,1,1>, 1.0);                // ***TEMP*** flush out old hover text
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
    {   ////debugMsg(DEBUG_MSG_INFO, jsn);                      // ***TEMP*** dump incoming JSON
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
    
    listen(integer channel, string name, key id, string msg)
    {   ////llOwnerSay("Listen from id " + (string) id + ": " + msg); // ***TEMP***
        if (channel == 0)                                       // if local chat
        {   if (gAction == ACTION_IDLE && llGetAgentSize(id) != ZERO_VECTOR)    // stay around if talked to by an avi
            {   
                vector dir = (vec_to_target(id));               // face who's talking
                float heading = llAtan2(dir.x,dir.y);           // direction to face as heading (0=north)
                bhvTurn(heading);
                llResetTime();                                  // reset attention span
            }
            return;
        }
#ifdef DEBUGCHAN
        if (channel == DEBUGCHAN)                               // if debug control
        {   bvhDebugCommand(msg);
            return;
        }
#endif // DEBUGCHAN
    }
}

