# Path planning for non-player characters in Second Life

## How it works

John Nagle

Animats

September, 2019

(PRELIMINARY - WILL CHANGE BEFORE RELEASE)

## Introduction
The basic concept here is straightforward. Use SL's **llGetStaticPath** to get an initial guess at a path. 
Check the path for obstacles with **llCastRay**.
If an obstacle is found, find the far side of the obstacle on the static path.
Use a maze solver to try to find a way to get around the obstacle. 
Put all these pieces of a full path together and follow that path using keyframe animation.
Monitor the movement along the path for collisions and new obstacles ahead. If there's a problem, replan
from the current position.

So movement is totally preplanned a few seconds in advance of the move.
This allows more precise motion. Our NPCs can go over narrow bridges and
through tight spots, unlike SL's pathfinding characters. 

## The problems
Doing this in Linden Scripting Language brings out some problems. Scripts only get 64K of memory and
are allocated a tiny amount of CPU power. Working within those restrictions complicates the system
considerably. 

First, speed. Planning the entire path from beginning to end is slow for long paths - tens of seconds.
We need to start the NPC moving faster than that.
So there needs to be pipelining to get movement started before planning is complete. 

Second, memory space. Each program in Linden Scripting Language gets 64K of memory. 
More programs, more memory. Programs can send messages to each other, but there is
no flow control, and filling the event queue causes events to be lost.
Reliable script coordination is complex.

Third, responsiveness. Plans can change. Obstacles can move. We must deal with all that.

## System structure.

### Top level program
This is up to the user. We have a demo, "Patroller", but expect users to use their own or
modify that one. 
It calls the functions in the README file - **pathNavigateTo** and **pathPursue** to cause the NPC to move.


### Start task (UNIMPLEMENTED - will be split from pathcall.lsl)
Starts the planning process and handles restarts. Path planning often fails to reach the goal,
especially when obstacles are moving. This task checks whether a restart would help, and restarts
the operation from the current position.
### Prep task
Prepares for planning. 

Starts with a llGetStaticPath call to get an initial guess for a route.
Some cleanup is done on that route, and it is passed to the Plan task.
### Plan task
Where most of the work gets done.

The static route is checked for obstacles, and the obstructed areas are sent to the maze solver, which
attempts to find a way around the obstacle. This process is incremental; as soon as a part of the route
is available, it is passed forward to the Assemble task. As each maze solve finishes, the "advance"
function is called to continue the planning. This allows NPC movement to often be nearly continuous
even though the planning and maze solving take several seconds. 
### Maze solver task
The plan task finds two points on the static path which bracket an obstacle. It is the
job of the maze solver to find a usable path between those points. The maze solver lays
out a grid with cells the size of the character width, usually 0.5m. The grid is not
axis-aligned; it's aligned to the path between the endpoints. 

This is a follow-the-wall type maze solver. It's moderately efficient, but not optimal.
When it follows a wall, it does so in both directions simultaneously, and chooses the
shortest path.

The maze solver is 2D, but has some limited Z awareness to avoid problems when a path
crosses under another path. A typical situation is a stairway which can be both walked
up and walked under. 
### Assemble task
This puts together the path sections from the plan task and the maze solver task.
It does some cleanup and optimization on the paths. 
### Move task
Generates the keyframe movement commands to follow the path.
Makes llCastRay calls to check for obstructions ahead, and detects collisions.
Controls movement speed, slowing down for turns.
Tracks the avatar being pursued during Pursue operations to see if a course
change is needed.
Also maintains a list of recent "good points" visited, so that if the NPC gets
stuck, it can back up to a good point.

## Additional components
### Animesh AO
Triggers animations based on the movement of the character - walk, slow walk, run, stand, turn. 
"Stand" can be changed with a link message, so the top level program is in control of the animation when the character is not moving.
### Marker task
Debug tool. Puts temporary colored markers along the path to show what the planner is doing. Good static path sections are green, 
areas the maze solver found clear are yellow, and areas the maze solver found obstructed are red. This is only enabled if
the preprocessor symbol **MARKERS** is defined for the maze solver task and the scan (motion) task.
Running this is a sizable load on the sim, so it's only for debug use.

### Debug logger
A debugging tool. 
Listens for errors reported on DEBUG_CHANNEL and outputs them with llOwnerSay, so they can be seen even if the owner is further away than 10 meters.
Has to be in a separate prim from the one with the main scripts, because **listen** will not receive messages from the same prim.

### Include files
These are included by various of the programs above.

## Include file "pathcall.lsl"
This file is included by the top level program to get access to the path planning system. Documented in README file.
## Include file "pathbuildutils.lsl"
General purpose functions used by other programs. Included by most of the programs.
## Include file "mazesolvercall.lsl"
Included by Plan task, and has the code for calling the maze solver.
## Include file "pathmovecall.lsl"
Included by tasks that need to communicate with the Scan (motion) task.
## Include file "assert.lsl" 
Assert and panic functions.
## Include file "patheerrors.lsl"
Error code names and numbers.

