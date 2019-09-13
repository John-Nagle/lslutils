//
//   debugrelay.lsl -- relay script for DEBUG_CHANNEL
//
//   General purpose use debug tool.
//
//   Animats
//   2019
//
//  This just picks up anything on DEBUG_CHANNEL and 
//  forwards it to llOwnerSay and IM. It's to catch
//  errors such as "Stack/heap collision" that don't show up
//  in owner chat unless you're close to the source of the problem.
//
//  Just add a prim with this to anything that generates error messages
//  you might miss and don't want to.  It has to be in a prim other than
//  the one that generates the error, because you can't listen to yourself.
//


//
//  Constants
//
integer MIN_IM_INTERVAL = 300;                                  // seconds between IMs. Do not overdo.
//
//  Global
//
integer gPathMsgLevel = 0;                                      // debug logging off by default. Set this to change level
integer gPathLastIMTime = 0;                                    // last instant message sent. Do this rarely.

//
//  Main program
//
default
{
    state_entry()
    {
        llListen(DEBUG_CHANNEL, "", "", "");                    // listen forever on DEBUG_CHANNEL
    }
    
    //  Message on debug channel. Relay.
    listen(integer channel, string name, key id, string message)
    {   message = name + " in trouble at " + llGetRegionName() + " " + (string)llGetPos() + ": " + message;
        llOwnerSay(message);                                    // to local owner
        integer now = llGetUnixTime();
        if (now - gPathLastIMTime > MIN_IM_INTERVAL)            // do this very infrequently
        {   llInstantMessage(llGetOwner(), message);            // send IM to owner
            gPathLastIMTime = now;
        } 
    }
}
