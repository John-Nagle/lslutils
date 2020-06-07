//
//
//  pathmazereset.lsl -- reset scripts in maze prim.
//
//  Cannot reset scripts in another prim, so we have to have this.
//
//  Animats
//  June, 2020
//
//  License: GPLv3.
//
#include "npc/pathmazedefs.lsl"
//  
//  pathmasterreset -- reset all scripts whose name begins with "path".
//
//  Except this one.
//
pathmasterreset()
{   string myname = llGetScriptName();                                  // don't reset me
    integer count = llGetInventoryNumber(INVENTORY_SCRIPT);             // Count of all items in prim's contents
    while (count > 0)
    {   string sname = llGetInventoryName(INVENTORY_SCRIPT, count);     // name of nth script
        if (sname != myname && llSubStringIndex(llToLower(sname),"path") == 0)  // if starts with "path", and it's not us
        {   llOwnerSay("Resetting " + sname);                           // reset everybody
            llResetOtherScript(sname);                                  // reset other script
        }
        count--;
    }
}

//
//  The main program of the maze queue task
//
default
{  
    link_message( integer sender_num, integer num, string jsn, key id )
    {   if (num == MAZERESETREQUEST)
        {   pathmasterreset(); 
        }
    }

}

