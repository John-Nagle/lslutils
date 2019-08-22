# Path planing for non-player characters in Second Life

John Nagle

June, 2019

(PRELIMINARY)

## Movement and pathfinding
Second Life has a built-in pathfinding system, released in 2011. It was little used 
until the release of "animesh" in 2019, which allowed the creation of human and 
animal non-player characters similar to avatars.

Using the pathfinding system with animesh revealed major problems with pathfinding.
The problems are detailed in this bug report: https://jira.secondlife.com/browse/BUG-226391
The major issues are

* Static path planning is reasonably good
* Path following is poor and has many movement errors
* When an SL sim is overloaded, pathfinding starts to break down, and can break
down so badly that characters go flying around at high speed and get stuck in walls.

Pathfinding can be made to almost work acceptably on a lightly loaded sim, but
degrades badly under load. With overloaded sims the new normal in Second Life,
a better alternative is needed.

### Our alternative to SL pathfinding

Static path planning, from "llGetStaticPath", is reasonably good, and can generate
good paths for hard problems. Its paths are flat, on walkable surfaces; it has no
notion of character height. We use those paths as a starting point.

Paths can be checked for non-static obstacles and above the ground obstacles with
"llCastRay". So we do that. We then try to construct bypass paths around obstacles
found, using a "follow the wall" maze solver. This creates a grid, heads for the
goal, and if it hits an obstacle, follows the edge of the obstacle until an
open path to the destination appears.

All this planning is done before character movement starts. It takes about 0.5 to 5
seconds, depending on path length and clutter.

Once a path has been planned, the character follows it using keyframe animation. 

### Path following

With a plan in place, path following can be done quickly. Both keyframed motion
and vehicle-type motion could be used. We are trying keyframed motion first, because
it works well under sim overload conditions. During keyframed motion, we must constantly
check for obstacles ahead of the character, using llCastRay. If an obstacle is
encountered, the character stops. We then repeat the planning process from the 
current position and try to follow the new plan.

## Path planning system usage
### Calls
### pathinit
    pathInit(float width, float height, integer chartype, integer verbose)
    
Sets up the path planning system. **width** and **height** are the dimensions of the character. **chartype**
is the pathfinding type of the character, usually **CHARACTER_TYPE_A** for humanoid forms taller than they are wide.

The width and height define a cylinder
around the character's center. The character's collision model must not project beyond that cylinder. If it does,
the character will bump into obstacles and stop motion.
### pathNavigateTo
    pathNavigateTo(vector endpos)
    
Go to the indicated location, in the current region, avoiding obstacles.
### pathPursue
    pathPursue(key target, float stopshort)
    
Pursue the object **target**, usually an avatar. Stop short of the target by the distance **stopshort**, so as not to get in the avatar's face.
1.75 to 2.0 is a reasonable value for **stopshort**.

This just does a pathNavigateTo to the target's current location. If the target moves, it won't 
change the character's course. This may be improved in later versions. 
### pathStop
    pathStop()
    
Stop any current motion. A new command can then be sent. 
This is not usually necessary; sending a command while one is already running will stop the current movement, although not instantly.

## pathSpeed
    pathSpeed(float speed, float turnspeed)
    
Sets the movement speed for future operations. 

**speed** is in meters per second. Reasonable values are 0.5 to 4.
Ordinary walking speed in Second Life is 1.5 m/sec.

**turnspeed** is the turning speed when changing direction, in radians per second.
0.2 is a reasonable value.

### Callbacks

The user must call

    pathLinkMsg(integer sender_num, integer num, string msg, key hitobj)
    
on each incoming link message, and must define

    pathUpdateCallback(integer callbackstat, key hitobj)
   
which will be called as each path operation completes.
**callbackstat** is one of the values in "patherrors.lsl". 

The user should also call 

    pathTick()
    
every few seconds. This is for a stall timer only; if something goes wrong
in the pathfinding system and it stalls, **pathUpdateCallback** will still be called,
after a long delay. So the caller can rely on getting a callback. This an emergency
backup only; we've run days without needing it.

### Technical notes

#### Limitations of llGetStaticPath

#### Limitations of llCastRay

If the starting point for llCastRay is inside an object, that object will not be detected. This
makes it hard to find the far side of an object. Working backwards from the next waypoint of the
static path will fail if that waypoint is inside an object. 

Safely checking a path requires working forwards from a known point outside any solid object.


