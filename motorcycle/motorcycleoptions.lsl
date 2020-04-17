integer     DIR_START = 101;
integer     MSG_DEMO = 200;                         
integer     MSG_LOCKED = 201;                       
integer     MSG_UNLOCKED = 202;   
integer     MSG_LOUD = 301;
integer     MSG_SOFT = 302;                  

////////////////////////////////////////////////////////////////
//
//  Change 2018-0418 by Rachel Stardust
//  
//  Added color options
//
//  Colors inspired by real classic US bike old style colors
//
//  Order of names and color values must match
//
list colornames = ["Army", "Black", "Indian Red", "Petrol", "Pink", "Silver", "Yellow"];

list colors = [
<0.412, 0.329, 0.188>, <0.341, 0.161, 0.157>,    // Army
<0.039, 0.039, 0.039>, <0.502, 0.055, 0.016>,    // Black
<0.427, 0.094, 0.063>, <0.710, 0.533, 0.271>,    // Indian Red
<0.094, 0.345, 0.357>, <0.882, 0.976, 0.839>,    // Petrol
<1.000, 0.714, 0.757>, <1.000, 1.000, 1.000>,    // Pink
<0.498, 0.518, 0.533>, <0.773, 0.000, 0.000>,    // Silver
<1.000, 0.910, 0.290>, <0.039, 0.039, 0.039>     // Yellow
];

//  Toggle-type buttons
string BTNUNLOCKED  = "☐ Locked";
string BTNLOCKED    = "☑ Locked";
string BTNSOFT      = "☐ Loud";
string BTNLOUD      = "☑ Loud";
//  Non-toggle buttons
string BTNPAINT     = "Paint";

integer nColorIndex = 0;                                    // color index selected
integer nTank;
integer nFenderFront;
integer nFenderRear;

integer FindPrim( string name)
{
    integer i = llGetNumberOfPrims();
    for (; i >= 0; --i)
    {
        if (llGetLinkName(i) == name)
        {
            return i;
        }
    }
    return -1;
}

integer PaintPrim(integer nIndex)
{   ////llOwnerSay("Paint color: " + (string)nIndex);   // ***TEMP***
    if (nIndex < 0) { return FALSE; }
        
    vector color1= llList2Vector( colors, 2*nIndex);
    vector color2= llList2Vector( colors, 2*nIndex+1);
    
    llSetLinkPrimitiveParamsFast(LINK_SET, [
        PRIM_LINK_TARGET, nTank,
        PRIM_COLOR, 2, color1, 1.0,
        PRIM_COLOR, 4, color2, 1.0,
        PRIM_LINK_TARGET, nFenderFront,
        PRIM_COLOR, 3, color1, 1.0,
        PRIM_LINK_TARGET, nFenderRear,
        PRIM_COLOR, ALL_SIDES, color1, 1.0
     ]); 
    nColorIndex = nIndex;                                   // save as permanent state
    return TRUE;
}
//
//  end color options
//
////////////////////////////////////////////////////////////////

integer gDialogChannel;                             
integer gListenHandle;                              
integer gLockState = MSG_LOCKED;         
integer gLoudState = MSG_LOUD;           
list COLORBUTTONS = ["Army", "Black", "Indian Red", "Petrol", "Pink", "Silver", "Yellow"];
string DIALOGINFO = "\nBike owner menu.";
string PAINTINFO = "\nBike color options.";

//
//  Look up color by name
//
integer color_index(string message)
{   string pstring = llStringTrim(llDumpList2String(llParseString2List(message,["⬤"],[" "]),""),STRING_TRIM);
    integer i;
    for (i = 0; i < llGetListLength(COLORBUTTONS); i++)
    {   if (llList2String(COLORBUTTONS,i) == pstring) { return(i); }}  // find
    return(-1);                         // no find
}
//
//  Bring up main menu
//
main_dialog(key toucherid)
{   llListenRemove(gListenHandle);                  
    gListenHandle = llListen(gDialogChannel, "", toucherid, "");
    list buttons = [];                  // no buttons yet
    if (gLockState == MSG_LOCKED)
    {   buttons += BTNLOCKED; }
    else
    {   buttons += BTNUNLOCKED; }
    if (gLoudState == MSG_LOUD)
    {   buttons += BTNLOUD; }
    else
    {   buttons += BTNSOFT; }
    buttons += BTNPAINT;               // repaint button
    llDialog(toucherid, DIALOGINFO, buttons, gDialogChannel);
    llSetTimerEvent(60.0);
}
//
//  Bring up paint menu
//
paint_dialog(key toucherid)
{   llListenRemove(gListenHandle);                  
    gListenHandle = llListen(gDialogChannel, "", toucherid, "");
    list buttons = [];
    integer i;
    for (i=0; i<llGetListLength(COLORBUTTONS); i++)
    {   string s = llList2String(COLORBUTTONS,i);   // get color name
        if (i == nColorIndex) { s = "⬤ " + s; }     // add dot
        buttons += s;
    }
    llDialog(toucherid, PAINTINFO, buttons, gDialogChannel);
    llSetTimerEvent(60.0);
}


default
{
    state_entry()
    {
        //
        //  get link number of surfaces to be colored
        //  if names changed in links, update below
        //
        nTank= FindPrim("Tank");
        nFenderFront= FindPrim("Fender front");
        nFenderRear= FindPrim("Rear fender");
        //  end get link number
        
        gDialogChannel = -1 - (integer)("0x" + llGetSubString( (string) llGetKey(), -7, -1) );
        
    }
    
    on_rez(integer param)
    {   gLockState = MSG_LOCKED;                    
        list params = llGetPrimitiveParams([ PRIM_TEMP_ON_REZ ]); 
        integer isdemo = llList2Integer(params,0);  
        if (isdemo)                                 
        {   gLockState = MSG_DEMO;
            llWhisper(0,"Enjoy your demo bike. Buy one from 2RAW EXTREME."); 
        }    
    }                                     
    
    touch_start(integer total_number)
    {   key toucherid;
        toucherid = llDetectedKey(0);
        llListenRemove(gListenHandle);              // just in case                 
        gListenHandle = 0;    
        if (toucherid != llGetOwner())                  
        {   llWhisper(0,"Only the owner can use the owner menu."); 
            return;
        }
        main_dialog(toucherid);
        llSetTimerEvent(60.0);
    }
    
    
    link_message(integer sender, integer num, string message, key id)
    {  if(num == DIR_START)
        {   //  Synchronize main script states with menu states on start
            llMessageLinked(LINK_ROOT, gLockState, "", NULL_KEY);    
            llMessageLinked(LINK_ROOT, gLoudState, "", NULL_KEY);   // tell root script   
        }       
    }
    
    listen(integer channel, string name, key id, string message)
    {   
        //  Used for both dialogs
        llListenRemove(gListenHandle);                      
        llSetTimerEvent(0.0);
        integer nColorIndex = color_index(message); // is it a color?
        if (nColorIndex >= 0)
        {   PaintPrim(nColorIndex);                 // yes, paint
        } 
        else if (message == BTNPAINT)               // request for paint menu
        {
            paint_dialog(id);                       // 
        }        
        else if (message == BTNUNLOCKED)            // locking
        {
            llPlaySound("carlockchirp",1.0);
            gLockState = MSG_LOCKED;
            llMessageLinked(LINK_ROOT, gLockState, "", NULL_KEY);   
        }
        else if (message == BTNLOCKED)         // unlocking
        {
            llPlaySound("carlockchirp",1.0);
            gLockState = MSG_UNLOCKED;
            llMessageLinked(LINK_ROOT, gLockState, "", NULL_KEY); 
        }
        else if (message == BTNLOUD)                // toggle noise level
        {   gLoudState = MSG_SOFT;
            llMessageLinked(LINK_ROOT, gLoudState, "", NULL_KEY);   // tell root script
        }
        else if (message == BTNSOFT)
        {   gLoudState = MSG_LOUD;
            llMessageLinked(LINK_ROOT, gLoudState, "", NULL_KEY);   // tell root script
        }
        else 
        {   llSay(DEBUG_CHANNEL, "Dialog option bug."); // bad
        }
    }
    
    timer()
    {   //  Get rid of dead dialog if any     
        llSetTimerEvent(0); 
        llListenRemove(gListenHandle);
        gListenHandle = 0;        
    }
}

