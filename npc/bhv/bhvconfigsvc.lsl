//
//  bhvconfigsvc.lsl -- Read config file, notify other tasks of config info
//
//
//  General format of config files:
//
//  Comma-separated values.
//  
//  scriptname, param, param, param, ...
//
//  Each line is sent to the matching SERVICENAME in BHV folder as JSON
//         - {"request","param","script",SCRIPTNAME,"values",[...]}
//
//  License: GPLv3.
//
//  Animats
//  2020
//
#include "npc/assert.lsl"
//
//  Configuration
//
string CONFIG_NOTECARD = "Config";          // read this notecard for patrol points

#define LINKMSGCONFIGREQUEST    2000        // ***TEMP*** move to config

//  Global variables
//  Config card reader
integer gConfigEnabled = FALSE;             // have not finished config startup yet
integer gConfigNotecardLine;
key gConfigNotecardQuery;
//
//  startup -- read notecard.
//
//  Don't call this until behavior scripts are registered.
//
startup()
{
    //  Start loading lines from notecard
    gConfigNotecardLine = 0;
    gConfigEnabled = FALSE;                 // turns on when all points loaded
    if (llGetInventoryKey(CONFIG_NOTECARD) == NULL_KEY)     // waypoints file no good
    {
        llSay(DEBUG_CHANNEL, "Notecard '" + CONFIG_NOTECARD + "' missing or empty. Will not run.");
        configdone(FALSE);                  // fails
        return;
    }
    //  Start reading notecard. This may need a retry; dataserver is not reliable.
    getnextconfigline();                    // start getting config lines
 }

//
//  parseconfigline -- parse an incoming config line
//
//  Returns error if bad parse
//
parseconfigline(string data, integer lineno)
{
    data = llStringTrim(data, STRING_TRIM);                 // remove unwanted whitespace
    if (llStringLength(data) == 0 || llGetSubString(data,0,0) == "#")
    {   getnextconfigline();  // comment or blank
        return;
    }
    list params = llCSV2List(data);                         // parse string to list
    if (llGetListLength(params) < 2)                        // must have at least two fields
    {   badconfig(data, lineno, "Must have at least two comma-separated fields."); return; }
    string scriptname = llList2String(params,0);            // target script name
    if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
    {   scriptname = "bhv" + scriptname;                    // preface with bhv and retry
        if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
        {   badconfig(data, lineno, "No script named \"" + scriptname + "\"."); return; }
    }
    // We have a valid script name and params
    doconfigline(params);                                   // handle this config line  
}

//
//  getnextconfigline -- ready to get next config line
//
getnextconfigline()
{   if (gConfigNotecardLine < 0) { return; }                // done, ignore callback
    llOwnerSay("Get notecard line");
    gConfigNotecardQuery = llGetNotecardLine(CONFIG_NOTECARD, gConfigNotecardLine++); 
}

//
//  badconfig -- report config problem
//
badconfig(string data, integer lineno, string msg)
{   llSay(DEBUG_CHANNEL, "Configuration notecard problem: " + msg + "\n" + (string)lineno + ". " + data);
}

//  
//  configdone -- entire config read, done
//
configdone(integer valid)
{
    llOwnerSay("Configuration done, success = " + (string)valid);
    gConfigNotecardLine = -1;                           // no more reading
}

//
//  Region name lookup
//
key gLookupRegionStatusQuery = NULL_KEY;                // query ID for region
key gLookupRegionCornerQuery = NULL_KEY;                // query ID for region corner
string gLookupRegionName = "";                          // name being looked up
list gLookupRegionParams;                               // info associated with current query for callback
//
lookupregion(string name, list params)
{
    //  Stage 1: see if region is known.
    //  A quary for the position of an unknown region generates no reply.
    //  But a status query gets a reply of "unknown"
    gLookupRegionParams = params;                       // save params for callback
    gLookupRegionName = name;                           // save name for callback
    assert(gLookupRegionStatusQuery == NULL_KEY);
    gLookupRegionStatusQuery = llRequestSimulatorData(name,DATA_SIM_STATUS);
}
//
//  lookupregionstatusreply -- dataserver event called after region status lookup complete
//
lookupregionstatusreply(string data)
{   //  Stage 2: get location of known region
    gLookupRegionStatusQuery = NULL_KEY;                // note no dataserver request in progress
    if (data == "unknown")                                      // status no find
    {   lookupregioncallback(ZERO_VECTOR, gLookupRegionParams);// report no find
        return;
    }
    gLookupRegionCornerQuery = llRequestSimulatorData(gLookupRegionName,DATA_SIM_POS);
}
//
//  lookupregioncornerreply -- dataserver event called after region corner lookup complete
//
lookupregioncornerreply(string data)
{
    lookupregioncallback((vector)data,gLookupRegionParams);     // got data 
}
//
//  lookupregioncallback -- actual callback to do the work
//
lookupregioncallback(vector corner, list params)
{
    llOwnerSay("Region info: " + llDumpList2String(params,",") + " at " + (string)corner);
    getnextconfigline();                                        // all done, go on 
}

//
//  doconfigline -- handle a config line, test version
//
doconfigline(list params)
{
    llOwnerSay("Config: " + llDumpList2String(params,","));
    if (llList2String(params,0) == "patrol")
    {   lookupregion(llList2String(params,1),params);  } // look up name
    else 
    {   getnextconfigline(); }
}
//

default
{
    on_rez(integer start_param)
    {
        llResetScript();
    }
 
    state_entry()
    {
        startup();
    }

    
    link_message(integer sender_num, integer num, string jsn, key id)
    {      }
    
    dataserver(key query_id, string data)               
    {   llOwnerSay("Dataserver callback: " + data);     // ***TEMP***
        if (query_id == gConfigNotecardQuery)           // reading config line from notecard
        {
            if (data == EOF)                            // done reading notecard
            {   configdone(TRUE);                       // success
            }
            else
            {   parseconfigline(data, gConfigNotecardLine); // parse this config line and ask for another
            }
        }
        //  Region corner lookups
        else if (query_id == gLookupRegionStatusQuery) { lookupregionstatusreply(data); }
        else if (query_id == gLookupRegionCornerQuery) { lookupregioncornerreply(data); }
    }
}

