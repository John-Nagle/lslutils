//
//  bhvscheduler.lsl --- Behavior scheduler for NPCs.
//
//  Animats
//  November, 2019
//  License: GPLv3.
//
//
//  This is the scheduler for the behavior tasks of Animats NPCs.
//  One behavior runs at a time, the highest priority one.
//  This task controls those behavior tasks.
//
//  Tasks do all movement and animation by sending messages to this
//  task, to avoid race conditions.
//
//  Startup
//
//  At startup, this task sends a broadcast message to all
//  scripts in all prims, asking behaviors to register. 
//  Behaviors send a message back, with their script name,
//  and prim number. The scheduler registers them, 
//  assigns them a "num" value for messages, and 
//  replies to the register message to the correct prim
//  on a common channel. The behavior is then live,
//  but stopped and at zero priority. It can then
//  request a run priority from the scheduler.
//
//  Run
//
//  When the scheduler selects a behavior to run, it sends
//  a "run" message to the desired prim and num value to enter
//  run mode. This includes a serial number of the run cycle,
//  to prevent race conditions. The behavior than has control
//  and can make path planning and animation requests by sending
//  messages to the scheduler. These include the "num" value and
//  the run cycle number. Those have to match the scheduler's
//  currently active behavior, or the request will be rejected.
//  This prevents race conditions during behavior switching.
//  Rejected requests produce a "stop" message from scheduler to
//  behavior.
//
//  Stop
//
//  A stop message to a behavior may indicate that it has been
//  preempted by a higher priority behavior. The behavior can
//  no longer have movement requests executed. But it can still
//  compute and sense the world. When a behavior is done
//  and no longer needs to run, it can request priority 0,
//  which will yield a "stop" message, and cause the scheduler
//  to find something else to do.
//
//  
//  ***MORE***
