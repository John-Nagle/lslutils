# Non-player characters for Second Life

John Nagle

June, 2019

(PRELIMINARY)

## Background

...

## Movement and pathfinding
Second LIfe has a built-in pathfinding system, released in 2011. It was little used 
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
found.

Bypass paths are constructed with two algorithms. One simply tries a rectangular
route, a 90 degree left or right turn from the original path, a forward section,
and a turn back to the original path. This gets us past simple obstacles.

If the simple approach fails, we need a real maze-solving algorithm. A*
was coded, but it's too expensive. Its breadth-first serch visits most of the
squares on the grid. Examining a square is expensive; it costs us a lslGetStaticPath
call (which we might skip) and about 15 llCastRay calls. 

This led to a "follow the wall" maze solver. This creates a grid, heads for the
goal, and if it hits an obstacle, follows the edge of the obstacle until an
open path to the destination appears. This tests relatively few irrelevant points.
But the paths that come out are far from optimal.

So we have to apply a path straightener. Remove all collinear points. Try, for
each non-endpoint, a short-cut route which bypasses that point. Repeat until
no improvement. 

All this planning is done before character movement starts. It takes about 0.5 to 5
seconds, depending on path length and clutter. 

### Path following

With a plan in place, path following can be done quickly. Both keyframed motion
and vehicle-type motion could be used. We are trying keyframed motion first, because
it works well under sim overload conditions. During keyframed motion, we must constantly
check for obstacles ahead of the character, using llCastRay. If an obstacle is
encountered, the character stops. We then repeat the planning process from the 
current position and try to follow the new plan.

### Technical notes

#### Limitations of llGetStaticPath

#### Limitations of llCastRay

If the starting point for llCastRay is inside an object, that object will not be detected. This
makes it hard to find the far side of an object. Working backwards from the next waypoint of the
static path will fail if that waypoint is inside an object. 

Safely checking a path requires working forwards from a known point outside any solid object.


