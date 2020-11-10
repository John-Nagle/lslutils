//
//  Sit for avatar that has only one animation
//
//  Takes first animation in prim
//
//  Getting error: llStartAnimation: Script trying to trigger animations but agent not found
//  {RootName Smooth Rider - V2.9 (DEMO), RootPosition { 1.54013, 2.58784, 35.7088 }}

#include "motorcycle/vehicleutils.lsl"

string gAnim = "Bike_Passenger01";                                       // rider sit animation
key gSitter = NULL_KEY;

default
{
    state_entry()
    {   //  Clear any old sit targets beyond the root.
        integer i;
        for (i=2; i<llGetNumberOfPrims() + 1; i++)
        {
             llLinkSitTarget(i, ZERO_VECTOR, ZERO_ROTATION);
        }
        llSitTarget(<-0.25,0,0.54>, ZERO_ROTATION);     // set our sit target
        gAnim = llGetInventoryName(INVENTORY_ANIMATION, 0);
    }
    
    on_rez(integer num)
    {   llResetScript(); }                              // reset on rez
    
    changed(integer change) {
        if (change & CHANGED_LINK)                      // only when sitters change
        {   gSitter = llAvatarOnSitTarget();            // do we have a sitter
            if (gSitter != NULL_KEY)                    // if avatar on prim
            {   llRequestPermissions(llAvatarOnSitTarget(), PERMISSION_TRIGGER_ANIMATION);   }
        }
        //  Restart anim on region cross. Sometimes it gets turned off. 
        if ((change & CHANGED_REGION) && gSitter != NULL_KEY)
        {  llSetTimerEvent(1.0);   }                    // fix animation after clean region crossing
    }
    
    //  Initial animation setup
    run_time_permissions(integer perm) {
        if (gAnim != "" && gSitter != NULL_KEY) {
            if (ifnotseated(gSitter, 5.0, FALSE))
            {   llSetTimerEvent(1.0);                   // wait for seating
                ////llWhisper(0,"Got perms, retrying.");    // ***TEMP***
                return; 
            }     // avatar has not arrived yet
            llStopAnimation("sit");
            llStopAnimation("sit_generic");
            llStopAnimation("sit_female");            
            llStartAnimation(gAnim);
            ////llWhisper(0,"Resetting anim for passenger on seat 1: " + (string) llKey2Name(gSitter)); // ***TEMP***
            llSetTimerEvent(0.0);                       // stop timer, don't need it again
        }
    }
    
    //  Fix up after each region crossing
    //  But make sure we have perms.  They're off temporarily during some region crossings.
    timer() {
        if (gSitter == NULL_KEY || gAnim == "")  // if passenger got off or no animation
        {   ////llWhisper(0,"No sitter, stopping timer.");  // ***TEMP***
            llSetTimerEvent(0.0);
            return;
        }
        //  If everything is OK to restart the animation, do so.
        if (ifnotseated(gSitter, 3.0, FALSE)) { return; }     // avatar has not arrived yet
        if ((llGetPermissions() & PERMISSION_TRIGGER_ANIMATION) == 0)           // don't have perms, must ask
        {   llRequestPermissions(llAvatarOnSitTarget(), PERMISSION_TRIGGER_ANIMATION);      // ask for perms
            ////llWhisper(0,"Asking for perms");    // ***TEMP***
            return;
        }
        //  Re-seating complete. Have perms. Restart anims.
        ////llWhisper(0,"Resetting anim for passenger on seat 2: " + (string) llKey2Name(gSitter)); // ***TEMP***
        llStartAnimation(gAnim); 
        llSetTimerEvent(0.0);                 
    }
}

