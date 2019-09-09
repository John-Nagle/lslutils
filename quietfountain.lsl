//
//  quietfountain.lsl -- fountain sound controller
//
//  Turns down sound if there is an avatar very close.
//
#define SOUND "fountainsound"

#define VOLHI 0.40                                  // loud sound, with nobody nearby
#define VOLLO 0.10                                  // soft sound, with someone nearby
#define RANGE 15.0                                  // someone this close

#define TIMEINTERVAL    (20)                        // check every 20 seconds

float gVolume = VOLHI;                              // current audio volume

default
{
    state_entry()
    {   llStopSound();
        llLoopSound(SOUND,gVolume);
        llSetTimerEvent(TIMEINTERVAL);              // long timer
    }
    
    timer()
    {
        list agents = llGetAgentList(AGENT_LIST_PARCEL_OWNER,[]); // get agents in parcel
        integer length = llGetListLength(agents);
        integer i;
        float volume = VOLHI;                                       // assume nobody around
        for (i=0; i<length; i++)                                    // for all agents in parcel
        {   ////llOwnerSay("Agent: " + llKey2Name(llList2Key(agents,i)));    // ***TEMP***
            vector apos = llList2Vector(llGetObjectDetails(llList2Key(agents,i), [OBJECT_POS]),0); // get
            if (llVecMag(apos-llGetPos()) <= RANGE)                 // if within range
            {   volume = VOLLO;                                     // turn down volume
            }
        }
        if (volume != gVolume)                                      // if volume changed
        {   gVolume = volume;
            llStopSound();
            llLoopSound(SOUND, gVolume);                            // change volume            
        }       
    }
    
    on_rez(integer rezarg)
    {   llResetScript(); }
}
