//
//  pathdefs.lsl - part of pathfinding library
//
//  Makes Second Life pathfinding work reliably.
//
//  Animats
//  January, 2019
//  License: GPL
//  (c) 2019 John Nagle
//
//  Definitions common to multiple scripts
//
//  Constants
//
//  Pathmode - what we're doing now.
integer PATHMODE_UNINITIALIZED = 0;                         // no character yet
integer PATHMODE_OFF = 1;                                   // what we're doing now
integer PATHMODE_NAVIGATE_TO = 2;
integer PATHMODE_PURSUE =  3;
integer PATHMODE_WANDER =  4;
integer PATHMODE_EVADE = 5;
integer PATHMODE_FLEE_FROM = 6;
integer PATHMODE_CREATE_CHARACTER = 7;                      // create character, used at start
integer PATHMODE_UPDATE_CHARACTER = 8;                      // update character

list PATHMODE_NAMES = ["Uninitalized", "Off", "Navigate to", "Pursue", "Wander", "Evade", "Flee", "Create", "Update"];  // for messages
//
//  Path statuses.  Also, the LSL PU_* pathfinding statuses are used
integer PATHSTALL_NONE = -1;                                // not stalled, keep going
integer PATHSTALL_RETRY = -2;                               // unstick and retry
integer PATHSTALL_STALLED = -3;                             // failed, despite retries
integer PATHSTALL_CANNOT_MOVE = -4;                         // can't move at all at current position
integer PATHSTALL_NOPROGRESS = -5;                          // not making progress, fail
integer PATHSTALL_UNSTICK = -6;                             // stuck, need to try an unstick
integer PATHSTALL_UNREACHABLE = -7;                         // pursue did not start, unreachable
integer PATHSTALL_LINKMSGFAIL = -8;                         // link messages stopped - script crash or queue overflow
integer PATHSTALL_UNINITIALIZED = -9;                       // not initialized yet			                

//  Error levels
integer PATH_MSG_ERROR = 0;
integer PATH_MSG_WARN = 1;
integer PATH_MSG_INFO = 2;
//
//  Message direction (because both ends see a reply)
integer PATH_DIR_REQUEST = 101;                             // application to path finding script
integer PATH_DIR_REPLY = 102;                               // reply coming back
integer PATH_DIR_REPLY_TICK = 103;                          // reply indicating other end is still alive
//
//  Global
//
integer gPathMsgLevel = 0;                                  // debug logging off by default. Set this to change level
//
//  pathMsg  -- call for pathfinding problems
//
pathMsg(integer level, string msg)                              // print debug message
{   if (level > gPathMsgLevel) { return; }                      // ignore if suppressed
    string s = "Pathfinding: " + msg;
    llOwnerSay(s);                                              // message
    if (level <= PATH_MSG_ERROR) 
    {   llSay(DEBUG_CHANNEL, s); }                              // serious, pop up the red message
}
//
//  pathErrMsg -- path error number to string.  Mostly for debug.
//
string pathErrMsg(integer patherr)
{   list patherrspos =[
    "Character is near current goal.",
    "Character has reached the goal.",
    "Character cannot navigate from the current location.",
    "Goal is not on the navmesh.",
    "Goal is no longer reachable.",
    "Target avi can no longer be tracked.",
    "No place for character to go.",
    "Evade  - hiding.",
    "Evade - running",
    "Fatal pathfinding server error.",
    "Dynamic pathfinding disabled.",
    "Not allowed to enter parcel."];

    list patherrsneg = [                // our additional errors
        "Error 0",                      // should not make it to user
        "No error",                     // should not make it to user
        "Retrying pathfinding", 
        "Stall timeout",
        "Cannot move from current position",
        "Not making progress",
        "Stuck, need unstick",
        "Goal unreachable",
        "Link message failed",
        "Uninitialized"];
        
    if (patherr >= 0 && patherr < llGetListLength(patherrspos)) // positive numbers, defined by LL
    {   return(llList2String(patherrspos,patherr));
    } else if (-patherr < llGetListLength(patherrsneg))          // negative numbers, defined by us
    {    return(llList2String(patherrsneg, -patherr)); 
    } else {
        return("Unknown path error " + (string) patherr);       // some bogus number
    }
}
