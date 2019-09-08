# Vehicle info messages - concepts

From a conversation between John Nagle (Animats resident) and Cinn Bouchard.
This is an outline for a general way for SL objects associated with transportation
to talk to each other.

This is very preliminary. It's written as a protocol specification so there's something
concrete to discuss. It's a skeleton to be filled in and changed as necessary.

## Goals

- Standardized way for vehicles, loaders, and other vehicle support facilities to communicate

- Communication is short-distance local within SL and does not require a server

- A few mandatory features, other optional features.

- Should initially support GTFO loading and fueling. Extend to other vehicle related tasks.

## Protocol outline

### Basic format

Everything is in [JSON](http://wiki.secondlife.com/wiki/Json_usage_in_LSL), because LSL has a built-in JSON parser
which will turn JSON into LSL lists.

All keywords are in lower case and contain only ASCII letters and numbers.
String fields for messages are full Unicode in any relevant language.

All string fields have no leading or trailing blanks. This simplfies testing for keywords.

Messages are sent on channel -673645769, which is a random number from a random number generator.
That will be a standardized channel number used grid-wide for this. It is not a secret.
(There is an SL convention that negative channel numbers are used for machine to machine communication.
Positive numbers are for humans.)

## Query types

### Finding nearby objects    

The **search** message asks nearby objects of a given class to speak up and identify themselves. 

    {"request" : "search", "serial" : INTEGER, "range": FLOAT, "objectclass" : STRING}
    
Any cooperating object of type **objectclass** hearing this within **range** of the sending object replies with

    {"reply" : "search", "serial": INTEGER, "version": INTEGER}
    
The ID of the sender comes with the LSL listen event, so we don't need to send that.
Serial number is simply the sequence number of the request, to avoid confusion with
old messages.

**version** is the protocol version. For now, that's 1. This is in case we need to make big changes
later and still interoperate with old objects. Objects tell you what version they speak, but all
objects reply to **search** without a version, so we can still find obsolete objects in future.
    
This is used to locate nearby objects of interest.
    
**objectclass** values are few and standardized:

    "vehicle" - some kind of vehicle

    "loading" - some type of loading facility
    
    "fuel" - some type of fueling facility
    
    "hub" - some kind of hub facility
    
    "parking" - some place where parking is allowed
    
    "door" - some type of openable obstruction
    
    "rez" - a rez zone
    
    "npc" - some type of non player character
    
    "avatar" - an avatar equipped for this system (unusual, but allowed)
    
These are intended to be very general. Just enough to make search work.
This section is just finding something that will talk to you.
There's just enough detail here to find things to talk to and filter out
things you don't care about. More details can be requested for each object.
 
All objects using this channel should understand these messages.
These are fixed responses for each object, except for echoing back the **serial** field. 

Tools for building maps can easily query entire regions and find all cooperating transportation-related objects.
 
### Querying objects

    {"request" : "info", "serial" : INTEGER, "id", KEY }
    
This asks for basic info about an object, including what queries an object understands.

    {"reply" : "info", "serial" : INTEGER, "objectclass": STRING, "name" : STRING, "fields" : [STRING, STRING ...]}
    
The values in the "fields" list indicate what can be asked for. Those
values can be used in a "request".

This is a fixed response for each object, except for echoing back the **serial** field. 

This message is mostly for querying more complex objects to find out what they can do. 
For example, a hub with few requests available might do its loading with the usual
sit and wait for a minute while nothing happens system. A more elaborate hub might
ask a trucker to back to a specific loading door, open the door, open the truck's
door, use an automated forklift to place crates in the truck, close everything up,
and have a animesh NPC hand the driver the paperwork. The idea here is to allow
for lots of expansion capability while retaining backwards compatibility. 
    
### An example: "door"

Anything that is openable on command is a "door". A gate, a drawbridge,
a loading dock a vehicle door, etc. Also, possibly, a gas cap or a
vehicle hood. This is useful when several
objects need to coordinate loading a truck, and is also useful when non-player
objects need to open a door.

Requests for a door:

   {"request" : "info", "serial" : INTEGER, "id", KEY, "language" : "en" } 
   
sent to a door should return

   {"reply" : "info", "serial" : INTEGER, "objectclass": "door", "fields" : ["status", "open", "close"]}
   
which tells you that the door will reply to those commands. 

   {"request" : status", "serial" : INTEGER, "id", KEY, "language" : "en" } 
   
returns some door info:

   {"reply" : status", "serial" : INTEGER, "state": STRING, "pos": VECTOR, "rot": ROTATION, "size" : VECTOR,
    "rpos": BOOLEAN, "regioncorner": VECTOR, doortype" : STRING, "error" : STRING, "msg" : STRING }
    
**language** in the request is the language for the "msg" field, if the object has multiple language options.
The keywords are fixed for all language options.
Language codes are per [ISO 639-1](https://www.loc.gov/standards/iso639-2/php/code_list.php), the 2-letter language
codes used on the Web, similar to the country suffixes on domains.
    
**state** - "open", "closed", "locked", "auto", or "moving", as you'd expect. "locked" implies "closed".
"auto" means it will open when approached and you don't need to request an open. Doors which say they
are "auto" must open for vehicles, avatars, pathfinding objects, and keyframed NPCs such as
forklifts. Many older open
scripts don't do that. "moving" is for slow moving large doors, drawbridges, and such, where you may
have to start the open process and keep polling while the massive object does its thing.

A door has the UUID of the object requesting the open; that comes with the message and is not forgeable.
So the door can decide if the UUID is allowed to ask for an open. It should return "locked" to a "status"
request if the UUID's request for an open would be rejected.

**pos**, **rot**, **size**, and **rpos** tell where the door is. Pos, rot, and size define a box which
will fit through the door. The +X direction for the prim is the vector to travel through the door. For swing
doors and gates, the box should cover the area of swing, so that's understood as a keep-out area during opening.
**rpos**, if TRUE, means these coordinates are relative to the ID queried. That's for vehicles. All that
is fixed information, set once during setup. **regioncorner** is the corner of the region, from **llGetRegionCorner()**.
This identifies the region, which may needed near a region boundary. Omit  if **rpos** is true. 

The pos/rpos/regioncorner set of data is the way all positions will be represented. 

**error**, if present, means something went wrong. Meanings for error strings are not defined here.
If **error** is not present, the request was executed. 

**msg**, if present, is a message for users. It doesn't indicate an error; it can be a welcome message.

**doortype**, if present, is the type of door. Meaning is not defined here.

    {"request" : "open", "serial" : INTEGER, "id", KEY }
    
is a request to open the door. The reply is the same as for **status**.

    {"request" : "close", "serial" : INTEGER, "id", KEY } does what you think it does.
    
So that's an example of one kind of object.

We should distribute a door script which talks this protocol.

### GTFO load and unload messages

(To be defined)
    
## Examples

Find a rez zone within 200m:

   {"request" : "search", "serial" : 1234, "range": 200.0, "objectclass" : "rez"}
   
Reply:

   {"reply" : "search", "serial" :1234, "objectclass" : "rez", "pos" : <134,455,22>, "regioncorner" : <64000.0, 192000.0, 0.0> }
   
Multiple replies are possible.
   
Find a GTFO hub (reply): 

   {"reply" : "search", "serial" :1234, "objectclass" : "hub", "pos" : <134,455,22>, "regioncorner" : <64000.0, 192000.0, 0.0>,
    "name" : "GTFO World HQ"}
    
Open a door:

   {"request" : "open", "serial" : 1234, "id", "16c59568-4edc-49a9-91ab-7e393f3431dd" } 
   
Rejected door open:

   {"reply" : "open" , "serial" : 1234, "state" : "closed", "error" : "rejected", "msg" : "You are not allowed to open this door." }
   
## Security

Anyone can make requests. Objects can refuse them. An object receiving an "open" for example, might check a list of authorized
users.

Objects get the UUID of the sender from the LSL "listen" event, and
that's controlled by the sim, so that can be trusted. An object's own UUID is never sent in the JSON of a message.
An "id" there is for the destination object. 
    
Objects can check whether something is close enough to execute a command. Most SL fuel pumps, and some doors, do this now.
Rejecting a request from a source too far away makes griefing a very short range operation. Probably short range enough
that the griefer is on your parcel and can be banned. For many requests, checking if the sender is on the same parcel
is a good idea.

Anybody should be allowed to do parcel-wide "search", but that has limited griefing potential.

In severe cases, consider counting the number of messages per minute from the most active sender and, if it
is excessive, ignore them.

    
    



    
    
