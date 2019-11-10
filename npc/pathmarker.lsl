//
//  pathmarker.lsl - makes debug markers to show path planning.
//
//  Animats
//  July, 2019
//
#include "npc/mazedefs.lsl"
integer MARKERCHANNEL = -3938235;                                // arbitrary channel number
integer MARKERREPLYCHANNEL = -3938236;                              // reply channel
string MARKERNAME = "Path marker, rounded (TEMP)";                        // rez this
integer LINKMSGMARKER = 1001;                                   // link message for marker                   
//
//  Globals
integer gListenHandle = 0;                                          // our listen handle
integer gMarkerSerial;                                          // serial number of marker
list gPendingMarkers = [];                                      // list of markers to do




//
//  handlemsg -- handle incoming JSON message from a marker checking in for instructions.
//
//  Format:
//  {"reply": "marker", "id": INTEGER }
//
handlemsg(integer channel, string name, key id, string message)
{
    ////llOwnerSay("Marker reply: " + message);               // ***TEMP***
    if (channel != MARKERREPLYCHANNEL) { return; }   // not ours
    string reply = llJsonGetValue(message, ["reply"]);       // what to do
    if (reply != "marker")                            // not valid 
    {   llSay(DEBUG_CHANNEL, "Invalid reply from marker: " + message); return; }
    integer id = (integer)llJsonGetValue(message,["id"]);
    integer length = llGetListLength(gPendingMarkers);  // items in pending marker list
    integer i;
    for (i=0; i<length; i += 2)            // for all markers waiting for info
    {   if (id == llList2Integer(gPendingMarkers,i))   // if ID matches
        {   list msg = ["request", "marker", "id",id] +  llJson2List(llList2String(gPendingMarkers,i+1));    // add ID to params
            llSay(MARKERCHANNEL, llList2Json(JSON_OBJECT,msg)); // tell marker what to do
            gPendingMarkers = llListReplaceList(gPendingMarkers,[], i, i+1);   // remove from list
        }
    }
    if (llGetListLength(gPendingMarkers) == 0)
    {   ////llOwnerSay("All markers rezzed");
        llListenRemove(gListenHandle);                              // no need to keep listening
        gListenHandle = 0; 
    }
}

//
//  The main program of the marker task.
//
default
{
    state_entry()
    {
        gMarkerSerial = 0;
        gPendingMarkers = [];
    }
    
    on_rez(integer param)
    {   llResetScript(); }

    
    listen(integer channel, string name, key id, string message)
    {
        handlemsg(channel, name, id, message);              // told what to do
    }
    
    link_message(integer sender_num, integer num, string json, key name)
    {   ////llOwnerSay("Link msg #" + (string)num + ": " + json);               // ***TEMP***
        if (num != LINKMSGMARKER) { return; }                               // not ours
        if (gListenHandle == 0) { gListenHandle = llListen(MARKERREPLYCHANNEL, "", NULL_KEY, ""); } // listen for marker replies
        gMarkerSerial++;
        gPendingMarkers += [gMarkerSerial,json];                      // save params for later reply
        //  Rez the object
        vector rezpos = llGetPos() + <0,0,2>;           // rez above object
        llRezObject(name, rezpos, ZERO_VECTOR, ZERO_ROTATION, gMarkerSerial );             
    }
}

