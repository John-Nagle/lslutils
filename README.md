# lslutils
Utility scripts for Second Life, in Linden Scripting Language

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

An experiment in automatic handling of region crossing problems.

## Status
Experimental
