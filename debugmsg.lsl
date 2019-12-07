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
#define LOG_CHANNEL 999                                         // Send debug log messages on this channel
//
//  Global
//
integer gDebugMsgLevel = 0;                                     // debug logging off by default. Set this to change level
integer gDebugLastIMTime = 0;                                   // last instant message sent. Do this rarely.

//
//  debugMsg  -- call for pathfinding problems
//
#define debugMsg(level, msg) { if (level <= gDebugMsgLevel) { debugMsgFn((level),(msg)); }} // as macro, to prevent building message that will never print

debugMsgFn(integer level, string msg)                           // print debug message
{   if (level > gDebugMsgLevel) { return; }                     // ignore if suppressed
    string s = llGetScriptName() + ": " + msg;                  // add name of script
    llWhisper(LOG_CHANNEL, (string) level + "|" + s);           // message, with level on the front
}

#endif // DEBUGMSGLSL
