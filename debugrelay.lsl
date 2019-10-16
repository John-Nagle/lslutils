//
//   debugrelay.lsl -- relay script for DEBUG_CHANNEL
//
//   General purpose use debug tool.
//
//   Animats
//   2019
//
//  This picks up anything on DEBUG_CHANNEL and 
//  forwards it to llOwnerSay and IM. It's to catch
//  errors such as "Stack/heap collision" that don't show up
//  in owner chat unless you're close to the source of the problem.
//
//  Messages on LOG_CHANNEL are saved in a circular buffer. 
//  They can be dumped on demand, or are dumped to IM on an error-level message.
//
//  Just add a prim with this to anything that generates error messages
//  you might miss and don't want to.  It has to be in a prim other than
//  the one that generates the error, because you can't listen to yourself.
//
//
#include "debugmsg.lsl"
//
#define DEBUG_LOG_MAX 30                                    // max number of messages to keep

#define pathGetRoot(obj) (llList2Key(llGetObjectDetails((obj),[OBJECT_ROOT]),0))   // get root key of object.

//
//  Globals
//
integer gDebugChannel = 0;                                  // listen channels
integer gLogChannel = 0;
integer gVerbose = FALSE;                                   // quiet unless turned on
list gMsgLog;                                               // circular buffer of messages
key gRootkey;                                               // our root prim

//
//  zerofill -- add leading zeroes to number as requested
//
string zerofill(integer n, integer filllen)
{   string s = string(n);
    while (llStringLength(s) < filllen) { s = "0" + s; }
    return(s);
}
//
//  timestamp -- get a text timestamp HH:MM:SS, SLT time.
//
string timestamp()
{
    integer now = (integer)llGetWallclock();
    now = now % (60*60*24);                                 // time of day
    integer secs = now % 60;
    integer mins = (now / 60) % 60;
    integer hours = now / (60*60);
    return("[" + zerofill(hours,2) + ":" + zerofill(mins,2) + ":" + zerofill(secs,2) + "]");
}

//
//  buffermsg  -- add message to circular buffer
//
buffermsg(string name, key id, integer msglev, string message)
{   string rootname = llKey2Name(pathGetRoot(id));          // name of root object, not debug
    string s = timestamp() + " (" + name + "," + rootname + ") " + message;
    gMsgLog += [s];                                         // add to circular buffer
    if (llGetListLength(gMsgLog) > (DEBUG_LOG_MAX + 10))      // if becoming too long
    {   gMsgLog = llList2List(gMsgLog,-DEBUG_LOG_MAX,-1); } // discard old msgs
}

//
//  logdump  -- dump log to llOwnerSay
//
logdump(string why)
{
    llOwnerSay("=== STORED LOG DUMP === from " + why);
    integer i;
    for (i=0; i<llGetListLength(gMsgLog); i++)              // print all stored log messages
    {   string s = llList2String(gMsgLog,i);
        llOwnerSay(s);
    }
    llOwnerSay("=== END STORED LOG DUMP ===");
}

//
//  logverbose -- turn verbose mode on/off
//
logverbose(integer newmode)
{   gVerbose = newmode;                                     // set verbose mode
    if (gVerbose)
    {   llOwnerSay("Verbose mode on."); }
    else 
    {   llOwnerSay("Verbose mode off."); }
}

logdumptoim(string why)
{   integer IMMAXLEN = 1000;                                // conservative max IM length
    ////llOwnerSay("=== STORED LOG DUMP === : " + why);
    string msg;
    integer i = llGetListLength(gMsgLog) -1;
    //  Add stored messages to IM string in reverse up to max string length
    while (i > 0 && llStringLength(msg) < IMMAXLEN - 100)
    {   string s = llList2String(gMsgLog,i);
        msg = s + "\n" + msg;
        i--;
    }
    msg = why + "\n" + msg;                                 // preface with why
    llInstantMessage(llGetOwner(), msg);                    // send IM to owner
}

//
//  logdumptoowner - Dump stored log to owner output
//
logdumptoowner(string why)
{   llOwnerSay("=== STORED LOG DUMP === : " + why);
    integer i;
    for (i=0; i<llGetListLength(gMsgLog); i++)
    {   string s = llList2String(gMsgLog,i);
        llOwnerSay(s);
    }
    llOwnerSay("=== STORED LOG DUMP END ===");
}
//
//  logdumptodebug -- dump to debug channel
//
logdumptoowner(string why)
{   llSay(DEBUG_CHANNEL,"=== STORED LOG DUMP === : " + why);
    integer i;
    for (i=0; i<llGetListLength(gMsgLog); i++)
    {   string s = llList2String(gMsgLog,i);
        llSay(DEBUG_CHANNEL,s);
    }
    llSay(DEBUG_CHANNEL,"=== STORED LOG DUMP END ===");
}



//
//  Debug channel message
//
debugchanmsg(string name, key id, string message)
{   if (pathGetRoot(id) != gRootkey) { return; }            // has to be from us
    seriouserrormsg(name,id,message);
}


//
//  Serious error message
//
seriouserrormsg(string name, key id, string message)
{   if (pathGetRoot(id) != gRootkey) { return; }            // has to be from us
    message = name + " in trouble at " + llGetRegionName() + " " + (string)llGetPos() + ": " + message;
    buffermsg(name, id, DEBUG_MSG_ERROR, message);
    logdumptoowner(message);                                // to local owner
    integer now = llGetUnixTime();
    if (now - gDebugLastIMTime > DEBUG_MIN_IM_INTERVAL)     // do this very infrequently
    {   logdumptoim(message);                               // dump to IM
        gDebugLastIMTime = now;
        gMsgLog = [];                                       // clears log
    }
}


//
//  Log channel message
//
//  Log channel messages should begin with "n|" to indicate the error level.
//
logchanmsg(string name, key id, string message)
{   message = llStringTrim(message, STRING_TRIM);           // remove unwanted whitespace
    //  Check for commands
    if (llSubStringIndex(message, "dump") == 0) 
    {   logdump("Dump command"); 
        gMsgLog = []; 
        return;
    } 
    if (llSubStringIndex(message, "verbose") == 0) 
    {   logverbose(TRUE); 
        return;
    }
    if (llSubStringIndex(message, "quiet") == 0)
    {   logverbose(FALSE);
        return;
    }
    if (id != gRootkey)                                     // not a command, must come from own object        
    {   if (pathGetRoot(id) != gRootkey) { return; }
    }
    integer barix = llSubStringIndex(message,"|");          // look for bar
    if (barix < 3 && barix > 0)                             // if proper marker
    {   
        integer msglev = (integer)llGetSubString(message,0,barix-1);    // get message level
        message = llGetSubString(message,barix+1,-1);       // get rest of message
        if (msglev == DEBUG_MSG_ERROR)                      // bad, pop up alert, IM send
        {   seriouserrormsg(name, id, message);
            return;
        }
        buffermsg(name, id, msglev, message);               // log the message
        if (gVerbose)
        {   llOwnerSay("(" + name + ") : " + message); }
        return;
    }
    llOwnerSay("Bogus msg on log channel from " + name + ": " + message);   // bogus 
}
//
//  Command
//
//
//  Main program
//
default
{
    state_entry()
    {   gRootkey = llGetLinkKey(LINK_ROOT);                                 // only listen to our own root prim
        gLogChannel = llListen(LOG_CHANNEL, "", "", "");                    // listen forever on LOG_CHANNEL and DEBUG_CHANNEL
        gDebugChannel = llListen(DEBUG_CHANNEL, "", "", "");
    }
    
    on_rez(integer start_param)
    {
        llResetScript();
    }
    
    //  Message on debug channel. Relay.
    listen(integer channel, string name, key id, string message)
    {   ////llOwnerSay("Channel " + (string)channel + ": " + message);   // ***TEMP***
        if (channel == LOG_CHANNEL) { logchanmsg(name, id, message); }
        else if (channel == DEBUG_CHANNEL) { debugchanmsg(name, id, message); }
    }
}
