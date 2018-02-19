//
//  SeatBelt - assist in recovery from vehicle problems at region crossings
//
//  Animats
//  February, 2018
//
//
//  Configuration
//
integer DEBUG = TRUE;                                   // verbose

//  Constants
float TIME_IDLE = 5.0;                                  // run every 5 secs when not otherwise busy
float TIME_SITTING = 0.2;                               // run every 0.2 secs when sitting
float TIME_DETECT_FAIL = 3.0;                           // seconds to fail detect

//  Globals
key gAvatar = NULL_KEY;                                 // no attached avatar yet
key gSitObject = NULL_KEY;                              // not sitting on anything yet
vector gGlobalPos = <0,0,0>;                            // current position in global coordinates
float gFailTime = 0.0;                                  // how long in fail mode

//  Utility functions
debugmsg(string msg)                                    // debug message print
{   if (DEBUG) llOwnerSay(msg); }                       // debug message print

string integer2binstr(integer n)                        // converts integer to string, base 2
{   string s = "";
    do
    {   if (n & 1) {   s = "1" + s; } else { s = "0" + s; }  // slow
        n = (n >> 1) & (0x7fffffff);                    // logical shift right by 1 bit
    }   while (n != 0);
    return(s);
}

default
{
    state_entry()
    {
        debugmsg("Script reset.");
        key id = llGetLinkKey(LINK_ROOT);             // apparently meaningless for an attachment
        if (id == NULL_KEY)                           // not attached to anything
        {   debugmsg("No root."); return; }
        list details = llGetObjectDetails(id, [OBJECT_NAME]);
        debugmsg("Attached to " + llList2String(details,0));       
    }
    
    changed(integer change)                             // link changed events
    {   debugmsg("Change: " + integer2binstr(change));  // change as bits
    }
    
    attach(key id)                                      // attach/detach events
    {   if (id == NULL_KEY)
        {   
            gAvatar = NULL_KEY;                         // not attached
            llSetTimerEvent(0.0);                       // stop periodic processing
            debugmsg("Detached from avatar.");
        } else {
            list details = llGetObjectDetails(id, [OBJECT_NAME]);
            debugmsg("Attached to " + llList2String(details,0));
            gAvatar = id;                               // attached to this avatar
            llSetTimerEvent(TIME_IDLE);                 // timer period when not doing anything
        }
    }
    
    timer()                                             // every N seconds
    {   
        if (gAvatar == NULL_KEY) return;                // we're not attached, do nothing.
        list details = llGetObjectDetails(gAvatar, [OBJECT_ROOT]);
        key root = llList2Key(details,0);
        integer infobits = llGetAgentInfo(gAvatar);     // get avatar info bits
        integer fault = FALSE;                          // no fault yet
        if (root != gAvatar)                            // if we are sitting on something
        {   if (gSitObject != root)
            {   gSitObject = root;                       // not sitting on something
                list details = llGetObjectDetails(root, [OBJECT_NAME]);
                debugmsg("Sat on " + llList2String(details,0));
                gFailTime = 0.0;                        // no fails yet
                llSetTimerEvent(TIME_SITTING);          // go to fast timer
            }
            //  Analyze sit situation
            vector apos = llList2Vector(llGetObjectDetails(gAvatar, [OBJECT_POS]),0);
            vector spos = llList2Vector(llGetObjectDetails(gSitObject, [OBJECT_POS]),0);
            float disttoseat = llVecMag(apos - spos);
            if (disttoseat > 1.0)                       // ***TEMP*** detect unsit
            {   fault = TRUE;                           // fault detected
                debugmsg("Distance to seat: " + (string)disttoseat);  
            }
        } else {                                        // not sitting
            if (gSitObject != NULL_KEY)
            {   //  May have fallen off. Or may have stood up. Must decide which.
                if ((infobits & AGENT_SITTING) == 0)        // sitting has ended
                {   debugmsg("No longer sitting. AGENT_SITTING is off.");
                    gSitObject = NULL_KEY;                  // forget sat-on object
                    llSetTimerEvent(TIME_IDLE);             // timer period when not sitting
                    return;                                 // done checking for now
                }
                //  Try checking distances, even though supposedly not sitting.
                vector apos = llList2Vector(llGetObjectDetails(gAvatar, [OBJECT_POS]),0);
                vector spos = llList2Vector(llGetObjectDetails(gSitObject, [OBJECT_POS]),0);
                float disttoseat = llVecMag(apos - spos);
                if (disttoseat > 1.0)                       // ***TEMP*** detect unsit
                {   fault = TRUE;                           // fault detected
                    debugmsg("Distance to seat, not sitting: " + (string)disttoseat);  
                } 
                //  Try checking sit count. Not too useful.
                list details = llGetObjectDetails(gSitObject, [OBJECT_SIT_COUNT]);
                integer len = llGetListLength(details);
                string s;
                if (len > 0) 
                {   integer sitcount = llList2Integer(details,0);
                    s = (string) sitcount;
                } else { s = "No sit object"; }
                if (infobits & AGENT_SITTING) s = s + " [AGENT SITTING]";
                if (infobits & AGENT_ON_OBJECT) s = s + " [AGENT ON_OBJECT]";              
                debugmsg("Avatar not linked to sit object. Sit count: " + s);         // but this might indicate trouble
            }
        }
        if (!fault)
        {   gFailTime = 0.0; return; }                      // no fault, reset
        gFailTime = gFailTime + TIME_SITTING;               // record time in fault mode
        if (gFailTime < TIME_DETECT_FAIL) return;           // not long enough in fault mode yet
        debugmsg("=== SIT FAULT DETECTED. ===");            // trouble
        llUnSit(gAvatar);                                   // trying an unsit
        //  ***MORE****
    }
}


