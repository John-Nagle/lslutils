//
//
//  pathcall.lsl -- caller interface to new pathfinding system
//
//  Animats
//  August, 2019
//
//  License: GPLv3.
//
#include "npc/patherrors.lsl"
#include "npc/pathmazedefs.lsl"
#include "debugmsg.lsl"

//

//  Constants
#define PATHCALLSTALLTIME 300 ////120                                   // if stalled for 120 seconds, reset everything 
//
//  Globals
//
integer gPathcallReset = FALSE;                                 // have we reset the system since startup?
integer gPathcallStarttime = 0;                                 // time last command started
integer gPathcallInitialized = FALSE;                           // not initialized yet
string  gPathcallLastCommand = "None";                          // last JSON sent to start
integer gPathcallRequestId = 0;                                 // path serial number
//
//  State used after a restart and reinit
float   gPathcallWidth;                                         // dimensions and type of character
float   gPathcallHeight;
integer gPathcallChartype;
float   gPathcallTurnspeed = 1.57079;                           // 90*DEG_TO_RAD default if not set, 90 degrees per second
integer gPathcallVerbose = FALSE;                               // not in verbose mode by default



//
//  User API functions
//
//  pathInit -- sets up the path planning system
//
//  Sets up the path planning system. 
//  width and height are the dimensions of the character. 
//  chartype is the pathfinding type of the character,
//  usually CHARACTER_TYPE_A for humanoid forms taller than they are wide.

//  The width and height define a vertical cylinder
//  around the character's center. The character's collision model must fit within cylinder. 
//  If it does not, the character will bump into obstacles and stop.
//
pathInit(float width, float height, integer chartype)
{
    if (!gPathcallReset)
    {   pathmasterreset();                          // reset everybody
        gPathcallReset = TRUE; 
    } // everybody has been reset
    //  Save params for path system crash recovery
    gPathcallWidth = width;                         // dimensions of character
    gPathcallHeight = height;
    gPathcallChartype = chartype;                   // its character type
    //  Broadcast params to everybody.
    llMessageLinked(LINK_THIS,PATHPARAMSINIT,llList2Json(JSON_OBJECT,["request","pathparams",
        "width",width, "height",height,"chartype",chartype]),"");
    gPathcallInitialized = TRUE;                    // initialized
}

//
//  pathTurnspeed -- set turn speed for future moves.
//
//  Usually sent once at startup. Linear speed is in every move call now.
//
//  turnspeed is the turning speed when changing direction, in radians per second.
//  0.2 is a reasonable value. When in a path segment with both moving and turning,
//  movement speed is slowed to allow the turn to complete at the usual turn rate.
//  So characters slow down when turning corners.
//
pathTurnspeed(float turnspeed)
{   gPathcallTurnspeed = turnspeed;                 // save for path system crash recovery
    llMessageLinked(LINK_THIS, PATHSTARTREQUEST, llList2Json(JSON_OBJECT,["request","pathspeed",
        "turnspeed",turnspeed]),"");
}


//
//  pathStop -- stop current operation
//
//  A new command can then be sent. 
//  This is not usually necessary.
//  Sending a command while one is already running will stop 
//  the current movement, although not instantly.
//
pathStop()
{   pathNavigateTo(llGetPos(), 100.0, 1.0);              // stop by navigating to where we are
}

//
//  pathTick -- call every few seconds when running the path system.
//
//  This is used only for a stall timer.
//
pathTick()
{   if ((gPathcallStarttime != 0) && (llGetUnixTime() - gPathcallStarttime) > PATHCALLSTALLTIME)
    {   //  TROUBLE - the system is stalled.
        debugMsg(DEBUG_MSG_ERROR, "Stalled and reset. Last command: " + gPathcallLastCommand); // tell owner
        pathmasterreset();                          // reset other scripts
        llSleep(10.0);                              // wait for reset
        //  Rebroadcast params to everybody. These were lost during the restart.
        llMessageLinked(LINK_THIS,PATHPARAMSINIT,llList2Json(JSON_OBJECT,["request","pathparams",
            "width",gPathcallWidth, "height",gPathcallHeight,"gPathcallChartype",gPathcallChartype]),"");
        pathTurnspeed(gPathcallTurnspeed);          // rebroadcast turnspeed
        debugMsgLevelBroadcast(gDebugMsgLevel, gPathcallVerbose, FALSE);// send debug params to path system
        pathUpdateCallback(PATHERRMAZETIMEOUT, NULL_KEY);       // report problem to caller
    }
}

//
//  pathNavigateTo -- go to indicated point
//
//  Go to the indicated location, in the current region, avoiding obstacles.
//
//  Stop short of the target by the distance stopshort. This can be zero. 
//
pathNavigateTo(vector endpos, float stopshort, float speed)
{
    pathBegin(NULL_KEY, endpos, stopshort, FALSE, speed);             // common for pathNavigateTo and pathPursue
}


//
//  pathPursue -- go to target avatar. 
//
//
//  Pursue the object target, usually an avatar.
//  Stop short of the target by the distance stopshort, so as not to get in the avatar's face.
//  1.75 to 2.0 meters is a reasonable social distance for stopshort.
//
//  Pursue works by navigating to the target, while the scan task checks if the target moves much.
//  If the target moves, the navigate operation is terminated with an error, causing a retry.
//  The retry heads for the new target position.
//
pathPursue(key target, float stopshort, integer dogged, float speed)
{   pathBegin(target, ZERO_VECTOR, stopshort, dogged, speed);          // start pursuit.
}

//
//  pathTurn -- turn to indicated heading.
//
//  Gets a callback.
//
pathTurn(float heading)
{
    llMessageLinked(LINK_THIS, PATHSTARTREQUEST, llList2Json(JSON_OBJECT,["request","pathturn",
        "heading",heading]),"");    
}
//
//  pathBegin -- go to indicated point or target. Common for pursue and navigateTo
//
//  Go to the indicated location, in the current region, avoiding obstacles.
//
//  Stop short of the target by the distance stopshort. This can be zero. 
//
pathBegin(key target, vector endpos, float stopshort, integer dogged, float speed)
{   if (!gPathcallInitialized) { panic("Path request made before init call"); } // don't let things start out of sequence
    gPathcallRequestId = (gPathcallRequestId+1)%(PATHMAXUNSIGNED-1);      // request (not path) serial number, nonnegative
    gPathcallLastCommand = llList2Json(JSON_OBJECT,["request","pathbegin","requestid",gPathcallRequestId,
        "target",target, "goal",endpos,"stopshort",stopshort,"dogged",dogged, "speed",speed]); // save for diagnostic
    llMessageLinked(LINK_THIS, PATHSTARTREQUEST,gPathcallLastCommand,"");
    gPathcallStarttime = llGetUnixTime();                               // start the stall timer
}

//
//  pathIgnoreOldReplies -- makes all old replies "stale".
//
//  Used when the scheduler switches tasks, so as to avoid confusion between completions
//
pathIgnoreOldReplies()
{
    gPathcallRequestId = (gPathcallRequestId+1)%(PATHMAXUNSIGNED-1);      // request (not path) serial number, nonnegative
}

//
//  End of user API
//

//
//  pathLinkMsg -- reply coming back from path execution
//
pathLinkMsg(integer sender_num, integer num, string jsn, key hitobj)
{   
    if (num == PATHSTARTREPLY)
    {   integer status = (integer)llJsonGetValue(jsn, ["status"]);  // status and pathid via JSON.
        integer requestid = (integer)llJsonGetValue(jsn, ["requestid"]);  // hitobj as key param in link message
        if (requestid != gPathcallRequestId)                         // result from a cancelled operation
        {   debugMsg(DEBUG_MSG_WARN, "Stale request completed msg discarded."); return; }
        debugMsg(DEBUG_MSG_WARN,"Path complete, status " + (string)status + " Time: " + (string)llGetTime());
        gPathcallStarttime = 0;                             // really done, stop clock
        pathUpdateCallback(status, hitobj);
    }
}




//  
//  pathmasterreset -- reset all scripts whose name begins with "path".
//
pathmasterreset()
{   string myname = llGetScriptName();                                  // don't reset me
    integer count = llGetInventoryNumber(INVENTORY_SCRIPT);             // Count of all items in prim's contents
    while (count > 0)
    {   string sname = llGetInventoryName(INVENTORY_SCRIPT, count);     // name of nth script
        if (sname != myname && llSubStringIndex(llToLower(sname),"path") == 0)  // if starts with "path", and it's not us
        {   llOwnerSay("Resetting " + sname);                           // reset everybody
            llResetOtherScript(sname);                                  // reset other script
        }
        count--;
    }
    gPathcallRequestId = 0;                                             // restart path IDs which keep scripts in sync
    llSleep(5.0);                                                       // wait 5 secs for reset.
    llOwnerSay("Master reset complete.");                               // OK, reset
}

//
//  Misc. support functions. For user use, not needed by the path planning system itself.
//

integer rand_int(integer bound)                 // bound must not exceed 2^24.
{   return((integer)llFrand(bound)); }          // get random integer                   

vector vec_to_target(key id)         
{   return(target_pos(id) - llGetPos());   }         

vector target_pos(key id)
{   list v = llGetObjectDetails(id, [OBJECT_POS]);  
    if (v == []) { return(ZERO_VECTOR); }         // not really a good choice for fails  
    return(llList2Vector(v,0));                             
}

float dist_to_target(key id)
{   
    list v = llGetObjectDetails(id, [OBJECT_POS]);  
    if (v == []) { return(INFINITY); }         // if gone, infinitely far away
    return(llVecMag(llList2Vector(v,0) - llGetPos()));  // distance to target                            
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
//  is_active_obstacle -- true if obstacle might move.
//
integer is_active_obstacle(key id)
{   if (id == "" || id == NULL_KEY) { return(FALSE); }          // no object
    ////return(TRUE);                                               // anything that obstructed us is alive, for now.
    //  Guess if this is a live object.
    list details = llGetObjectDetails(id, [OBJECT_VELOCITY, OBJECT_PHYSICS, OBJECT_PATHFINDING_TYPE, OBJECT_ANIMATED_COUNT]);
    integer pathfindingtype = llList2Integer(details,2);            // get pathfinding type
    if (pathfindingtype == OPT_AVATAR || pathfindingtype == OPT_CHARACTER) { return(TRUE); } // definitely alive, yes
    if (pathfindingtype != OPT_LEGACY_LINKSET) { return(FALSE); }                           // if definitely static, no.
    if (llVecMag(llList2Vector(details,0)) > 0.0 || llList2Integer(details,1) != 0 || llList2Integer(details,3) > 0) { return(TRUE); } // moving or physical or animesh
    //  Need a really good test for KFM objects.
    return(FALSE);                                                      // fails, for now.
}


//
//   pathLinearInterpolate  -- simple linear interpolation
//
float pathLinearInterpolate(float n1 , float n2 , float fract )
{   return n1 + ( (n2-n1) * fract );    } 

//
//  easeineaseout  --  interpolate from 0 to 1 with ease in and ease out using cubic Bezier.
//
//  ease = 0: no smoothing
//  ease = 0.5: reasonable smoothing
//
float easeineaseout(float ease, float fract)
{   float ym = pathLinearInterpolate(0, ease, fract);
    float yn = pathLinearInterpolate(ease, 1, fract);
    float y =  pathLinearInterpolate(ym, yn,  fract);
    return(y);
}



//  Check if point is directly visible in a straight line.
//  Used when trying to get in front of an avatar.
integer clear_sightline(key id, vector lookatpos)
{   list obstacles = llCastRay(target_pos(id), lookatpos,[]);
    integer status = llList2Integer(obstacles,-1);
    if (status == 0) { return(TRUE); }      // no errors, zero hits, clear sightline.
    debugMsg(DEBUG_MSG_WARN, "Clear sightline status " + (string)status + " hits: " + (string)obstacles);
    return(FALSE);                          // fails
}




