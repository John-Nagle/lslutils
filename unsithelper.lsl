//  
//  Unsit helper - more graceful unsits
//

//
//  Globals
//
integer gSitting = FALSE;                                   // true if sitting

//
//  stoodup -- avatar just stood up
//
stoodup()
{   // Dumb version  - move 1m forward.
    vector target = llGetPos() + <1,0,0>*llGetRot();
    llMoveToTarget(target, 1.0);
    llSleep(1.0);
    llStopMoveToTarget();
   
}

default
{
    state_entry()
    {
        llSetTimerEvent(1.0);                               // always running timer - bad.
    }
    
    touch_start(integer total_number)
    {
        llSay(0, "Touched.");
        ////vector target = llGetPos() + <1,0,0>;
        vector target = llGetPos() + <1,0,0>*llGetRot();
        llMoveToTarget(target, 1.0);
    }
    
    timer()
    {  
        {
            integer sitting = (llGetAgentInfo(llGetOwner()) & AGENT_SITTING) != 0;  // true if sitting now
            ////llOwnerSay("Sitting: " + (string)sitting);
            if (gSitting & !sitting)
            {   llOwnerSay("Stood up");
                stoodup();
            }
            gSitting = sitting;                            // update sitting status
        }   
    }
}

