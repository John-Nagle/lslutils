#ifndef DEBUGMSGLSL
#define DEBUGMSGLSL
//
//  debugmsg.lsl -- debugging message support
//
//  Include this.
//
//
//  Error levels
//
#define DEBUG_MSG_ERROR     0                                    // decreasing order of importance
#define DEBUG_MSG_WARN      1
#define DEBUG_MSG_INFO      2
#define DEBUG_MSG_DEBUG     3
#define DEBUG_MSG_NAME_LIST ["error","warn","info","debug"]     // for UI use


#define DEBUG_MIN_IM_INTERVAL 3600                              // seconds between IMs. Do not overdo.
#define DEBUG_LOG_CHANNEL 999                                         // Send debug log messages on this channel
#define DEBUG_MSGLEV_BROADCAST  -99999                          // link message number for setting msg level
//
//  Global
//
integer gDebugMsgLevel = DEBUG_MSG_WARN;                        // debug at warn by default. Set this to change level
integer gDebugLastIMTime = 0;                                   // last instant message sent. Do this rarely.

//
//  debugMsg  -- call for pathfinding problems
//
#define debugMsg(level, msg) { if (level <= gDebugMsgLevel) { debugMsgFn((level),(msg)); }} // as macro, to prevent building message that will never print

debugMsgFn(integer level, string msg)                           // print debug message
{   if (level > gDebugMsgLevel) { return; }                     // ignore if suppressed
    string s = llGetScriptName() + ": " + msg;                  // add name of script
    llWhisper(DEBUG_LOG_CHANNEL, (string) level + "|" + s);     // message, with level on the front
}

//
//  debugMsgLevelSet -- set debug message level from broadcast message. Call for link messages on DEBUG_MSGLEV_BROADCAST
//
#define debugMsgLevelSet(jsn) {    gDebugMsgLevel = (integer)llJsonGetValue((jsn),["msglev"]);}  // set message level
//
//  debugMsgLevelBroadcast -- send broadcast msg to set message level and other debug params
//
#define debugMsgLevelBroadcast(msglev, verbose, dumplog) { llMessageLinked(LINK_SET,DEBUG_MSGLEV_BROADCAST,llList2Json(JSON_OBJECT,["request","msglev","msglev",(msglev), "verbose",(verbose),"dumplog",(dumplog)]),""); }
#endif // DEBUGMSGLSL
