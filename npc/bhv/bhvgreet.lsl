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
//  Notecard config formats:
//
//      greet, speed, SPEED
//      greet, msgfacing, MESSAGE
//      greet, msgnear, MESSAGE
//      greet, msggroup, MESSAGE
//      greet, idling, ANIM, ANIM...
//      greet, waiting, ANIM, ANIM...
//      greet, talking, ANIM, ANIM...
//
//  Talking should have a short talk anim and a looped stand anim.
//
//  TODO: 
//  - When others are talking, wait for N seconds of silence before talking.
//
//
#include "npc/bhv/bhvcall.lsl"
#include "npc/pathavatartrackcall.lsl"
#include "npc/mathutils.lsl"
//
//  States
//
integer ACTION_IDLE = 0;         
integer ACTION_PURSUE = 1;                  // go to place near target
integer ACTION_FACE = 2;                    // turn to face target, has callback
integer ACTION_GREET = 4;                   // face finished, do greet
integer ACTION_DISTANT_GREET = 5;           // face finished, do distant greet msg
integer ACTION_TALKING = 6;                 // facing an avi and talking

float GOAL_DIST = 1.75;                     // (m) get this close to talk
float MAX_GREET_DIST = 10.0;                // (m) if can get this close, say "Hello there"
float OBSTACLE_RETRY_PROB = 0.7;            // (fract) Retry if random < this.




#define PRIORITYPURSUE  PRIORITY_TASK       // higher than patrol, lower than evade vehicle

#define CHARACTER_SPEED  2.5                // (m/sec) pursue speed
#define CHARACTER_TURNSPEED_DEG  90.0       // (deg/sec) turn rate
#define WALKSPEED 2.0                       // (m/sec) walking speed, non-run


float IDLE_POLL = 10.0;
float ATTENTION_SPAN = 20;                  // will stick around for this long
float MIN_MOVE_FOR_RETRY = 0.25;            // must move at least this far before we recheck on approach

//  Configuration

integer PATH_STALL_TIME = 300;              // path stall time

//
//  Animations
//
list gAnimIdle = ["stand"];                 // standing on purpose
list gAnimWait = ["stand"];                 // standing during a move, i.e. waiting for planner
list gAnimTalk = ["stand","talk"];          // talking
//
//  Messages
//
string gMsgFacing = "";                     // can get to facing position
string gMsgNear = "";                       // can't get to facing position
string gMsgGroup = "Hello all";             // many avis present
//
//  Speed
//
float gSpeed = CHARACTER_SPEED;             // default speed

//  Global variables
integer gAction = ACTION_IDLE;
string gAnim;                   // current animation                       
key gTarget = NULL_KEY;         // current avatar to pursue
integer gListenChannel = 0;     // for detecting chat
float gDwell;                   // dwell time to wait
float gFaceDir;                 // direction to face next

//
//  bhvfinished -- finished with this greet cycle
//
bhvfinished()
{   gAction = ACTION_IDLE;
    bhvAnimate(gAnimIdle);
    bhvSetPriority(PRIORITY_OFF);                                   // give up control of the NPC
}

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
            vector targetpos = target_pos(gTarget);
            if (targetpos == ZERO_VECTOR)                               // target is gone
            {   debugMsg(DEBUG_MSG_INFO,"Avatar has left the area.");             
                bhvfinished();                                          // done
                return;               
            }
            vector offset = dir_from_target(gTarget) * GOAL_DIST; 
            vector finaltarget = targetpos + offset;
            if (is_clear_sightline(gTarget, finaltarget))               // if can get to face target
            {            
                debugMsg(DEBUG_MSG_INFO,"Get in front of avatar. Move to: " + (string)finaltarget);
                bhvNavigateTo(llGetRegionCorner(), finaltarget, 0, WALKSPEED);  // get to social talking position
                llResetTime();
            } else {                                                    // can't get to social talk position
                debugMsg(DEBUG_MSG_WARN,"Can't get in front of avatar due to obstacle.");
                bhvAnimate(gAnimIdle);
                face(gTarget, ACTION_DISTANT_GREET);
            }                             
            return;
        }
        if (gAction == ACTION_FACE)
        {   //  Turn to face avatar.
            bhvAnimate(gAnimIdle);
            face(gTarget, ACTION_GREET);
            return;
        }
        if (gAction == ACTION_GREET)
        {   greet(gMsgFacing, TRUE);
            return;
        }
        if (gAction == ACTION_DISTANT_GREET)        // not face to face
        {   greet(gMsgNear, FALSE);
            return;
        }
        if (gAction == ACTION_TALKING)              // if someone near is talking
        {   return;                                 // continue pretending to talk 
        }
        //  Got completion in unexpected state
        debugMsg(DEBUG_MSG_ERROR,"Unexpected path completion in state " + (string)gAction + " Status: " + (string)status);
        bhvfinished();                                          // done
        return;
    }
    //  If had trouble getting there, but got close enough, maybe we can just say hi.
    if ((gAction == ACTION_PURSUE || gAction == ACTION_FACE) && dist_to_target(gTarget) < MAX_GREET_DIST) // if close enough to greet
    {   bhvAnimate(gAnimIdle);
        face(gTarget, ACTION_DISTANT_GREET);
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
            bhvAnimate(gAnimIdle);
            face(gTarget, ACTION_GREET);
            return;
        }

        if (gTarget != NULL_KEY)
        {   debugMsg(DEBUG_MSG_WARN,"Unable to reach " + llKey2Name(gTarget) + " status: " + (string)status);
            pathavatartrackreply(gTarget,"defer");          // tell tracker to defer action on this avi
            gTarget = NULL_KEY;                             // not tracking anyone
        }
            
        //  Failed, back to idle.
        debugMsg(DEBUG_MSG_WARN,"Failed to reach goal, idle. Path update status: " + (string)status);
        bhvfinished();                                          // done
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
        bhvAnimate(gAnimIdle);
        pathFaceInDirection(lookdir);       // face avatar
        bhvAnimate(gAnimIdle);
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
greet(string msg, integer whisper)                  // say or whisper
{
    gAction = ACTION_TALKING;                       // on completion, fake being in conversation
    llResetTime();                                  // start attention span timer.
    gDwell = ATTENTION_SPAN;                        // wait this long
    bhvAnimate(gAnimTalk);                          // talking
    if (whisper) {                                  // quietly, if possible
        bhvWhisper(msg);
    } else {
        bhvSay(msg);
    }
}

#define getroot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))  // move to common code
#ifdef OBSOLETE
//
//  inmovingvehicle -- true if avatar in moving vehicle
//
integer inmovingvehicle(key id)
{   key rootid = getroot(id);                                   // avi if not setting, else sit root
    if (id == rootid) { return(FALSE); }                        // not sitting
    list objinfo = llGetObjectDetails(rootid, [OBJECT_VELOCITY]);  // get velocity of sitting object
    if (llGetListLength(objinfo) < 1) { return(FALSE); }        // failed to get velocity
    float vel = llVecMag(llList2Vector(objinfo,0));             // get velocity
    if (vel < 0.001) { return(FALSE); }                         // don't approach
    {   debugMsg(DEBUG_MSG_WARN,llKey2Name(id) + " is in a moving vehicle, do not greet");
        return(TRUE);                                           // no greet
    }
}
#endif // OBSOLETE

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
//  start_pursue -- start pursuing avatar.
//
start_pursue()
{   debugMsg(DEBUG_MSG_WARN,"Pursuing " + llKey2Name(gTarget));
    gDwell = 0.0;
    bhvAnimate(gAnimWait);                                          // applies only when stalled during movement
    llSleep(2.0);                                                   // allow stop time
    bhvPursue(gTarget, GOAL_DIST*2, gSpeed);                        // go after the avatar, stopping a bit short
    gAction = ACTION_PURSUE;
    llSetTimerEvent(1.0);                                           // fast poll while moving
}
//
//  greet_done -- done here, return to other tasks
//
greet_done(string action)
{   
    if (action != "") { pathavatartrackreply(gTarget,action); }     // tell tracker to defer action on this avi
    bhvfinished();                                          // done
}

//
//  requestpursue -- a request to pursue has arrived.
//
//  Start it if not already pursuing someone.
//
requestpursue(key target)
{   if (!gBhvRegistered) { return; }                                // scheduler init still in progress, wait, will be repolled
    if (invehicle(target))                                          // if in a physical vehicle
    {   pathavatartrackreply(target,"defer");                       // try again later
        return;
    }
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
    bhvAnimate(gAnimIdle);                                          // back to idle anim
    if (gTarget == NULL_KEY)                                        // if we have no one to pursue
    {
        bhvSetPriority(PRIORITY_OFF);                               // we don't need to run now, give up control
        return;
    }
    llResetTime();                                                  // reset the clock
    //  Begin listening
    if (gListenChannel == 0) {
        gListenChannel = llListen(0, "", NULL_KEY, "");             // start listening for local chat to cue the NPC to be quiet.
        ////llOwnerSay("Listening on channel " + (string) gListenChannel); // ***TEMP***
    }
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
    if (gListenChannel != 0) { 
        llListenRemove(gListenChannel);
        ////llOwnerSay("Listen stopped on channel " + (string) gListenChannel); // ***TEMP***
    }
    gListenChannel = 0;
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
//
//  Config reading
//
//
//  bhvDoConfigLine -- handle a config line, test version
//
bhvDoConfigLine(list params)
{   if (llList2String(params,0) == "greet")
    {   integer valid = loadparams(params);                         // load and diagnose param line
        if (!valid) { return; }                                     // fails, we will not start
    }
    bhvGetNextConfigLine();                                         // on to next notecard line
}

//  
//  bhvConfigDone -- entire config read, done
//
bhvConfigDone(integer valid)
{
    gBhvConfigNotecardLine = -1;                        // no more reading
    if (valid)
    {   if (gMsgFacing == "" || gMsgNear == "")         // turned off in config
        {   llOwnerSay("Greeting off - no greet message in notecard.");
            return;
        }
        bhvInit();                                      // set up scheduler system
    }
    else
    {   llSay(DEBUG_CHANNEL,"=== GREET BEHAVIOR CONFIGURATION FAILED ==="); // stuck
    }
}

//
//  loadparams -- load requested anims from notecard info
//
//  Param format is
//
//  animator, walk, NEWANIM ...
//
//  Returns TRUE if all params valid.
//
integer loadparams(list params)
{
    string cmd = llList2String(params,1);               // request type
    params = llDeleteSubList(params,0,1);               // get rid of "greet, cmd"
    string param2 = llList2String(params,0);            // message if a message type line
    if (cmd == "msgfacing") { gMsgFacing = param2; return(TRUE); }    // this language needs dicts
    else if (cmd == "msgnear") { gMsgNear = param2; return(TRUE);  }
    else if (cmd == "msggroup") { gMsgGroup = param2; return(TRUE); }
    else if (cmd == "speed")
    {
        gSpeed = (float)param2 ;                        // how fast
        if (gSpeed < 0.25) { gSpeed = 0.25; }           // bound speed
        if (gSpeed > 8.0) { gSpeed = 8.0; }
        return(TRUE);
    }
    //  Must be an anim if we get here.
    //  Validate anims list
    integer i;
    for (i=0; i<llGetListLength(params); i++)
    {   string anim = llList2String(params,i);          // get this anim's name
        if (llGetInventoryType(anim) != INVENTORY_ANIMATION) // if no such animation
        {   llSay(DEBUG_CHANNEL,"Configured animation \"" + anim + "\" not found.");
            return(FALSE);                              // fails
        }
    
    }
    //  Store animations per cmd.
    if (cmd == "waiting") { gAnimWait = params; }
    else if (cmd == "idling") { gAnimIdle = params; }
    else if (cmd == "talking") { gAnimTalk = params; }
    else { llSay(DEBUG_CHANNEL,"No greet command \"" + cmd + "\"."); return(FALSE); }
    return(TRUE);                                       // successful config    
}

default
{
    on_rez(integer start_param)
    {   
        llResetScript();
    }
 
    state_entry()
    {
        gDebugMsgLevel = DEBUG_MSG_INFO;            
        bhvReadConfig();                            // start reading the config
    }
    
    timer()                                         // timer tick
    {   bhvTick();                                  // timers in path library get updated
        if (gAction == ACTION_TALKING)              // if talking to an avi
        {   if (llGetTime() < ATTENTION_SPAN)       // if clock has not run out
            {   return; }                           // do nothing
            //  Our patience is limited.
            greet_done("done");                     // tell tracker done with this avi.
        } else if (gAction == ACTION_PURSUE)        // if pursuing
        {   if (invehicle(gTarget))                 // if avi is in a moving vehicle
            {   debugMsg(DEBUG_MSG_WARN,"Pursuit aborted");                
                bhvStop();                                  // abort pursue now
                pathavatartrackreply(gTarget,"defer");      // tell tracker to wait on this one
                bhvSetPriority(PRIORITY_OFF);               // stop greeting now, do something else
            }
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
        if (num == DEBUG_MSGLEV_BROADCAST)                  // set message level from broadcast
        {   debugMsgLevelSet(jsn); return; }
    }
    
    //  Listen for avatars talking in local chat, so we shut up.
    listen(integer channel, string name, key id, string msg)
    {   ////llOwnerSay("Listen from id " + (string) id + ": " + msg); // ***TEMP***
        if (channel == 0)                                       // if local chat
        {   if (gAction == ACTION_TALKING && llGetAgentSize(id) != ZERO_VECTOR)    // stay around if talked to by an avi
            {   
                vector dir = (vec_to_target(id));               // face who's talking
                float heading = llAtan2(dir.x,dir.y);           // direction to face as heading (0=north)
                bhvTurn(heading);
                llResetTime();                                  // reset attention span
            }
            return;
        }
    }
    
    dataserver(key query_id, string data)               
    {   ////llOwnerSay("Dataserver callback: " + data);     // ***TEMP***
        //  Handle all the various data server events
        if (query_id == gBhvConfigNotecardQuery)       {   bhvParseConfigLine(data, gBhvConfigNotecardLine);}
    }
}

