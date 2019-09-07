# Path planning for non-player characters in Second Life

## Internal documentation

John Nagle

Animats

June, 2019


## What's going on here

### The basics
The basic concept is straightforward. First, get a basic path by using **llGetStaticPath**. 
This understands static obstacles and walkable areas, but does not check for vertical clearance or dynamic obstacles such
as moveable obstacles, avatars, or other NPCs. 

Second, check that basic path for obstacles using **llCastRay**. Find both where the basic path goes into the obstacle, and where it comes out.

Third, run a maze solver for each obstacle to find a path from where the basic path went into the obstacle to where it comes out.

Fourth, put the resulting path together, turn it into a form suitable for Second Life keyframe animation, and call **llSetKeyframedMotion** to
make it happen.

Fifth, while the character is traversing the path, make **llCastRay** calls along the path just ahead of the character to detect any new obstacles.
Also check for collisions.

### The details

(More to come)

## License

This system is free software licensed under the [General Public License version 3.](https://www.gnu.org/licenses/gpl-3.0.en.html)
You can look at it, copy it, and create things from it, but they must also be licensed under the GPLv3. 
In Second Life terms, that means **full perms, zero cost.**

Requests for commercial use can be addressed to "nagle@animats.com".












