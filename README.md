# lslutils
Utility scripts for Second Life, in Linden Scripting Language

License: GPLv3

These can be used in free Second Life items, but not in ones for commercial sale,
per the General Public License. Second Life items using these scripts under this
license should be no-cost and have modify, copy, and transfer permissions.

We also license these for commercial use. That requires prior arrangement.

# putback.lsl

A script for creating destructable enviroments. Useful for bar fights and such.

Add the script to the base object of furniture-like objects. The object can now be pushed
around and knocked over. When no avatar is within 20m of the object, it will be
automatically returned to its original position.

## Status
Production

## Known problems

Knocking an object into a parcel where object entry is allowed but scripts are not allowed to run
will leave the object stuck there.

# motorcycledrive.lsl

Control script for motorcycles

Automatic handling of region crossing problems.

Includes our "vehicleutils.lsl" and "regionrx.lsl", which are entirely new code. Those contain
the region crossing handling. 

By default, the vehicle is a demo. Provide a script to lock and unlock the vehicle and have it send
MSG_LOCKED or MSG_UNLOCKED to this script.

The script sends log messages about region crossings to a data logging script, but that's not included
because it data sends to our server. We put the logger in test vehicles to debug region crossing problems.

# rotatewheel.lsl

Makes a vehicle wheel rotate as the vehicle moves. Just set the wheel diameter and axis of rotation 
and it handles everything else.  Needs to be sent DIR_START and DIR_STOP link messages
to turn it on and off. Don't leave it running when the vehicle is parked as this will waste sim time.

# travelbelt.lsl

Moving sidewalk. The "belt" moves forward slowly, then jerks back. An avatar standing on it will move
forward smoothly.
The texture of the belt should have a texture repeat of 1 meter in the movement direction, so that
the jerk back will be invisible. When idle, the belt switches from keyframed motion to a texture
animation, to eliminate idle overhead.

## Status
Production

# touchtester.lsl

Demo of how to check whether a "touch" operation comes from an avatar with line of sight to
the object being touched. Useful for preventing buttons from being pushed or doors opened by
someone camming.

## Status
Demo
