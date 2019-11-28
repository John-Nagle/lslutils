//
//
//  bhvcall.lsl -- interface to new NPC system for behaviors.
//
//  Animats
//  November, 2019
//
//  License: GPLv3.
//
//  A behavior is a script which, for a time, has control of an NPC.
//  Behaviors are scheduled by a scheduler. They are told when to
//  go and stop, and can be preempted by higher priority behaviors.
//
//  Behaviors use regular LSL calls to find out information about
//  the world, but must send messages to the scheduler to move 
//  or animate the NPC. This is to eliminate race conditions.
//
//  ***MORE*** mostly unimplemented
//
//
#include "npc/patherrors.lsl"
#include "debugmsg.lsl"
//
//

//
//  Globals
//



//
//  Behavior API functions
//
//
//  bhvNavigateTo  -- go to point
//
//  Region corner (ZERO_VECTOR for current region) allows inter-region moves.
//  Stop short of "goal" by distance "stopshort"
//  Travel at "speed", up to 5m/sec, which is a fast run. 
//
bhvNavigateTo(vector regioncorner, vector goal, float stopshort, float speed)
{
}

//
//  bhvPursue -- pursue target avatar
//
bhvPursue(key target, float stopshort, float speed)
{
}

//
//  bhvTurn -- turn self to face heading
//
bhvTurn(float heading)
{
}

//
//  bhvAnimate -- run list of animations.
//
//  These only run when the avatar is stationary.
//  A new list replaces the old list, keeping any
//  anims already running still running without a restart
//
bhvAnimate(list anims)
{
}

//
//  bhvStop -- stop current movement
//
//  This is not usually necessary.
//  Sending a command while one is already running will stop 
//  the current movement, although not instantly.
//
bhvStop()
{   
}

//
//  bhvick -- call every at least once a minute when running the path system.
//
//  This is used only for a stall timer.
//  Failure to call this will cause a script restart.
//
bhvTick()
{
}

//
//  bhvSetPriority -- set priority for this behavior
//
//  Larger numbers have higher priority.
//  Zero means this behavior does not want control now.
//
bhvSetPriority(integer priority)
{
}

//
//  Callbacks
//
//
//  bhvInit -- behavor is being initialized
//  
//  The path planning system has restarted.
//
bhvInit()
{
}
//
//  bhvDoRun -- behavior has control
//
bhvDoRun()
{   
}
//
//  bhvDoStop -- behavior no longer has control
//
bhvDoStop()
{
}

//
//  Incoming events
//
//  Pass all incoming link messages to this.
//
bhvLinkMessage(int num, string jsn)
{
}

