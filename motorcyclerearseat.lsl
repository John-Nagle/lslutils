//
//  Sit for avatar that has only one animation
//
//  Takes first animation in prim
//
//  Getting error: llStartAnimation: Script trying to trigger animations but agent not found
//  {RootName Smooth Rider - V2.9 (DEMO), RootPosition { 1.54013, 2.58784, 35.7088 }}

string gAnim = "";                                       // rider sit animation
key gSitter = NULL_KEY;

default
{
    state_entry()
    {
        llSitTarget(<-0.25,0,0.54>, ZERO_ROTATION);
        gAnim = llGetInventoryName(INVENTORY_ANIMATION, 0);
    }
    
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
        if (gAnim != "") {
            llStopAnimation("sit");
            llStopAnimation("sit_generic");
            llStopAnimation("sit_female");            
            llStartAnimation(gAnim);
        }
    }
    
    //  Fix up after each region crossing
    //  But make sure we have perms.  They're off temporarily during some region crossings.
    timer() {
        if (gSitter == NULL_KEY || gAnim == "")  // if passenger got off or no animation
        {   llSetTimerEvent(0.0); return; }
        //  If everything is OK to restart the animation, do so.
        if ((llAvatarOnSitTarget() != NULL_KEY) && (llGetPermissions() & PERMISSION_TRIGGER_ANIMATION))
        {   ////llWhisper(0,"Resetting anim for pssenger on seat: " + (string) llKey2Name(gSitter)); // ***TEMP***
            llStartAnimation(gAnim); 
            llSetTimerEvent(0.0);                
        }  
    }
}

