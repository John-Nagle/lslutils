#   Door control for NPCs.
How to remotely operate common Second Life doors.

## NTBI doors

Web site: https://wiki.ntbigroup.com/index.php?title=IntelliDoor_Home,_Commercial,_Industrial,_and_Alarmed#API

### Example command
    llSay(-494827459,"NONEAnimatsworkshop&&open");
    
The documentation says to send

    [BBID][DoorID]&&open
    
The BBID is in the door's description field. It is, by default, "BBIDNONE".
The "NONE" part goes in the command. The "BBID" does not.

The DoorID is in the door's notecard.

Tested.

## HD Emergency doors

Web site: http://hdemergency.com/manuals/OverheadDoor.html#API

### Example command
    llSay(-4864231,"OPEN@24");
    
Untested.

## KoolDoor doors

Documentation available via notecards.

### Example command

    llSay(33,"open");
    
The user can change the channel. 33 is the default.

Untested.




