# Region crossings in Second Life
Problems and solutions

John Nagle
Animats
January, 2018

# Background
The Second Life virtual world is divided into regions 256 meters
on a side. Large areas are built up from many such regions, yielding 
a virtual world with hundreds of square kilometers in area. This division
into regions is both the great strength of Second Life. It allows building a
world of great size which appears almost seamless to the user. Most competing
virtual worlds are divided into limited areas. Second Life, unlike such systems,
can and has scaled geographically.

Each region is implemented as a "sim", a process handling all the objects in one
region. Each sim communcates with other sims, especially the immediately adjacent ones.
The technical problems of crossing between regions are difficult, as
they involve handoffs between distributed processes. Problems in this area have
plagued Second Life since its inception. The big virtual world is crippled by 
the difficulties of reliably moving around within it. Fix this, and Second Life
gains an edge over its competitors.

## Avatar region crossing

When an avatar representing one user crosses from one region to another, there is
a handoff between the neighboring "sim" programs. Avatars are active in only one
sim at a time. Moving an avatar involves freezing its state the losing sim, copying
information to the gaining sim, and starting its simulation on the gaining sim.
This generally works well.

## Vehicle region crossing

Vehicles can carry avatars across region crossings. The vehicle and the avatars
cross in separate operations. Vehicle crossings do not work as well as avatar
crossings, and can fail badly. Most of these problems
show themselves to the user as the avatar or vehicle being out of the expected position.
It's hard for an end user to distinguish the problems. Thus, although these are well known
problems and have been reported in multiple bug reports for a decade, the problems have not been solved.

# Region crossing problems

There are at least five (NNN) separate problems in region crossing. They all look
similar to the end user, and thus bug reports on them have been difficult to address.
Here we sort them out, discuss each separately, and provide solutions or workarounds
for many of them.

## Velocity extrapolation errors
When a vehicle or avatar crosses between regions, its state is frozen for about 0.5 seconds
to, in the worst case, 10 seconds. About 1 second is typical. During this period, no 
object position update messages are sent to the viewer client. The viewer attempts to 
extrapolate the position during this dead time to give the user a better user experience.

The extrapolation is done by code intended for another purpose entirely - reducing 
network traffic by not sending object updates when the position can be calculated
from the previous update's position, velocity, angular velocity, and linear acceleration
values. This is a well known video game optimization borrowed from multiplayer first person shooters,
where information for bullets and other flying objects need be sent only when they are
fired and when they hit something. 

Using this algorithm during the freeze at a region crossing can produce grossly incorrect
visual results. The slightest movement glitch in the last update as an avatar leaves the
control of one sim will be amplified by the extrapolator into a huge error. Avatars and
vehicles roll over, go into the ground, and fly wildly into the air because of this.
This is disconcerting to users, some of whom are unable to drive vehicles because of this
effect. 

This bug results in only temporary mis-positioning. When the gaining sim sends
an object update message, the correct position is restored. 

We have implemented a solution to this problem in the open source Firestorm viewer.
See https://jira.phoenixviewer.com/browse/FIRE-21915

This change clips motion extrapolation at sim boundaries, preventing bogus predictions.
The freeze happening during sim handoff is represented as the avatar or vehicle stopping
during the region crossing. This appears to be a good solution for land vehicles.
The user experience at region crossing becomes much cleaner.

With this change, slow sailing vessels look somewhat different at crossings; they stop for a moment.
Flags continue to wave, and sounds continue, but the animated wake looks wrong for a 
few seconds. Before the change, there were sometimes worse artifacts, including the
boat sinking for a few seconds, and the boat being jerked backwards as the region crossing
completes. But often, in open water, the region crossing seemed seamless, because all
water looks the same and the jerk-back was invisible. Some work
on wake animation may be able to restore the illusion of seamless crossings.

This fix is on track for inclusion in a Firestorm beta version. Linden Labs is welcome
to copy the code.

## Road problems.

***MORE***

### Sags and fall-throughs

### Physics / collision jams

## Animation shutdown

It is quite common for a vehicle rider animation to stop at a region crossing.
The default "sit" animation is then run. 
This can be fixed in vehicle scripts by detecting the region crossing, using
a "changed'" event, and then restarting the animation.

We check for a number of conditions after each region crossing - that the
avatar is close to their normal sit point, that the vehicle links to the
avatar and the avatar links back to the vehicle, and that all requested
animation permissions are available. There are moments during normal region handoffs
during which some of those conditions do not hold. Trying to do anything
from a script during those periods seems to have problems. Se we wait until 
the avatar/vehicle relationship is back to normal before proceeding.

Both this, and the camera fix below, are implemented in our test
motorcycle, "Region cross test bike v0.10"

The script is available at 

https://github.com/John-Nagle/lslutils/blob/master/motorcycledrive.lsl

and we usually leave a free motorcycle with these fixes parked at
Vallone/227/19/36. Look for a bike with yellow and black angled stripes.
This identifies our test bike. Feel free to take a copy.  This bike
also detects the "unsit" problem mentioned below, stops, and produces
some error messages. 

## Bogus camera movement

Big jumps in camera position are common at region crossing. 
So, at the same time we restart the animation, we reset the
camera parameters. This usually eliminates bogus camera motion
at region crossings. It also deals with the problem of slow loss of
camera position over time due to some cumulative error problem. 

## Half-unsits

This is the big one. The bug where avatars fall off vehicles and get stuck so
backly that they can't move.  This can ruin the user's experience so badly that they
have to log out and back in.

This is a sim-side bug, and we are unable to work around it from scripts or via
the viewer.  The avatar becomes separated from the vehicle.
The "Unsit" button in the viewer is ineffective. Movement keys will not move the
avatar.  Calling "llUnsit()" in the vehicle script does not return an error but
accomplishes nothing.

Video of this experience:

***MORE***

This problem has been seen for years, but until we got some of the other problems
out of the way, was hard to recognize and debug.

It's hard to reproduce this problem. The most successful way we've found to 
reproduce it is with a rail car run past two closly spaced sim crossings.
Because the car is on rails, it's in the same place every time.
Even then, it appears that the sims must not have seen avatar or rail car
recently, within the last 10 minutes or so. Otherwise, the sim crossing 
goes smoothly, presumably because some information is cached.





