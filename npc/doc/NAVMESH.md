# The Second Life pathfinding system - notes

John Nagle

Animats

September, 2019


![Path planning the Animats way](images/paththroughdoor.jpg)

## The static navmesh

Second Life servers can precompute a static navigation mesh for path planning purposes.
This mesh is used both by the built-in pathfinding system and by the LSL call
[**llGetStaticPath**](http://wiki.secondlife.com/wiki/LlGetStaticPath). 
It represents only objects marked by the object owner as static obstacles, walkable surfaces,
or keep-out areas. All this is documented.

(PICTURE)

What the documentation doesn't tell you is that the static navmesh is a coarse approximation to the obstacles in the
virtual world. It does not match it exactly. It does not follow every little detail.
The navmesh seems to be a low level of detail approximation 
of the world. That makes it fast to use. 
Because of this, it can only be used as a starting point for where to go.

### Z axis issues

If an block is placed on flat Linden ground, with the top of the block about 0.25m above ground,
and the block is made walkable, something strange happens.
The navmesh becomes be a flat plane, above the ground and below the top of the block.

(sandboxnavmesh0250.png)

This looks strange. Note the avatar's feet, on the ground but below the navmesh.

Along the Z axis, the static navmesh seems to be within 0.25 to 0.35 meter of actual obstacles.

If the top of the block is 0.375m above ground, something quite different happens.

(sandboxnavmesh0375.png)

Now the top of the block is a separate piece of navmesh from the ground.

This is how the navmesh represents level changes which can be stepped over, and
those which cannot. A discontinuity in the navmesh cannot be crossed by a path.

Calling **llGetStaticPath** returns elevations on the navmesh.
These are only an approximation to the ground level, and are not good food positions.
Using them as foot positions will sometimes result in feet below the ground and above the ground.
Users of the navmesh must use **llCastRay** to find the ground for each point returned
by **llGetStaticPath**.



