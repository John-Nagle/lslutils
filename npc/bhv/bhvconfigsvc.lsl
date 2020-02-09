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
//  ***MOVE THIS TO bhvschedulercall.lsl***
//
//
//  Config card reader. Used by behaviors with configured parameters.
//
//  Reads the first notecard in the prim, which must be in CSV format.
//  The first field of each line must be the name of the script that wants
//  that config line.
//  The "bhv" at the beginning of the script name can be ommitted.
//  Lines beginning with "#" are comments. Blank lines are ignored.
//
//  Each behavior reads the config file separately.
//
//  Global variables
//
integer gBhvConfigEnabled = FALSE;              // have not finished config bhvReadConfig yet
integer gBhvConfigNotecardLine;                 // current line on notecard
key gBhvConfigNotecardQuery;                    // database query callback key
string gBhvConfigName;                          // name of notecard
//
//  bhvReadConfig -- read notecard.
//
//  Don't call this until behavior scripts are registered.
//
bhvReadConfig()
{
    //  Start loading lines from notecard
    gBhvConfigNotecardLine = 0;
    gBhvConfigEnabled = FALSE;                 // turns on when all points loaded
    gBhvConfigName = llGetInventoryName(INVENTORY_NOTECARD, 0);  // get first notecard
    ////if (llGetInventoryKey(CONFIG_NOTECARD) == NULL_KEY)     // waypoints file no good
    if (gBhvConfigName == "")                                   // if no notecard
    {
        llSay(DEBUG_CHANNEL, "No notecard, containing configuration, in behaviors prim. Will not start.");
        bhvConfigDone(FALSE);                  // fails
        return;
    }
    //  Start reading notecard. This may need a retry; dataserver is not reliable.
    bhvGetNextConfigLine();                    // start getting config lines
 }

//
//  bhvParseConfigLine -- parse an incoming config line
//
//  Returns error if bad parse
//
bhvParseConfigLine(string data, integer lineno)
{
    data = llStringTrim(data, STRING_TRIM);                 // remove unwanted whitespace
    if (llStringLength(data) == 0 || llGetSubString(data,0,0) == "#")
    {   bhvGetNextConfigLine();  // comment or blank
        return;
    }
    list params = llCSV2List(data);                         // parse string to list
    if (llGetListLength(params) < 2)                        // must have at least two fields
    {   bhvBadConfig(data, lineno, "Must have at least two comma-separated fields."); return; }
    string scriptname = llList2String(params,0);            // target script name
    if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
    {   scriptname = "bhv" + scriptname;                    // preface with bhv and retry
        if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
        {   bhvBadConfig(data, lineno, "No script named \"" + scriptname + "\"."); return; }
    }
    // We have a valid script name and params
    bhvDoConfigLine(params);                                   // handle this config line  
}

//
//  bhvGetNextConfigLine -- ready to get next config line
//
bhvGetNextConfigLine()
{   if (gBhvConfigNotecardLine < 0) { return; }                // done, ignore callback
    gBhvConfigNotecardQuery = llGetNotecardLine(gBhvConfigName, gBhvConfigNotecardLine++); 
}

//
//  bhvBadConfig -- report config problem
//
bhvBadConfig(string data, integer lineno, string msg)
{   llSay(DEBUG_CHANNEL, "Configuration notecard problem: " + msg + "\n" + (string)lineno + ". " + data);
}
//
//  Caller must supply the following functions:
//
//  bhvConfigDone(integer valid)        // called when config is complete, with TRUE if valid config
//  bhvDoConfigLine(list params)        // configure line for this script, as a list from CSV fields
//
//  Caller must call the following functions:
//
//  bhvReadConfig()                     // start the config reading process
//  bhvGetNextConfigLine();             // call to cause read of next line, after called by DoConfigLine
// 
//
//  ***END OF MOVE THIS TO bhvschedulercall.lsl***
//  
//  bhvConfigDone -- entire config read, done
//
bhvConfigDone(integer valid)
{
    llOwnerSay("Configuration done, success = " + (string)valid);
    gBhvConfigNotecardLine = -1;                           // no more reading
}

//
//  Region name lookup. This takes two data server calls for each region name.
//  So we have a one-item cache.
//
key gLookupRegionStatusQuery = NULL_KEY;                // query ID for region
key gLookupRegionCornerQuery = NULL_KEY;                // query ID for region corner
string gLookupRegionName = "";                          // name being looked up
list gLookupRegionParams;                               // info associated with current query for callback
string gCachedRegionName;                               // saved name
vector gCachedRegionCorner;                             // saved corner
//
lookupregion(string name, list params)
{
    //  Stage 1: see if region is known.
    //  A quary for the position of an unknown region generates no reply.
    //  But a status query gets a reply of "unknown"
    if (name == gCachedRegionName)
    {   lookupregioncallback(gCachedRegionCorner, params);  // in cache, use cached result
        return;
    }
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
{   gCachedRegionName = gLookupRegionName;                  // cache this result
    gCachedRegionCorner = (vector)data;                     // cache this result
    lookupregioncallback(gCachedRegionCorner,gLookupRegionParams);     // got data 
}
//
//  lookupregioncallback -- actual callback to do the work
//
lookupregioncallback(vector corner, list params)
{
    llOwnerSay("Region info: " + llDumpList2String(params,",") + " at " + (string)corner);
    bhvGetNextConfigLine();                                        // all done, go on 
}

//
//  bhvDoConfigLine -- handle a config line, test version
//
bhvDoConfigLine(list params)
{
    llOwnerSay("Config: " + llDumpList2String(params,","));
    if (llList2String(params,0) == "patrol")
    {   lookupregion(llList2String(params,1),params);  } // look up name
    else 
    {   bhvGetNextConfigLine(); }
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
        bhvReadConfig();
    }

    
    link_message(integer sender_num, integer num, string jsn, key id)
    {      }
    
    dataserver(key query_id, string data)               
    {   llOwnerSay("Dataserver callback: " + data);     // ***TEMP***
        if (query_id == gBhvConfigNotecardQuery)           // reading config line from notecard
        {
            if (data == EOF)                            // done reading notecard
            {   bhvConfigDone(TRUE);                       // success
            }
            else
            {   bhvParseConfigLine(data, gBhvConfigNotecardLine); // parse this config line and ask for another
            }
        }
        //  Region corner lookups
        else if (query_id == gLookupRegionStatusQuery) { lookupregionstatusreply(data); }
        else if (query_id == gLookupRegionCornerQuery) { lookupregioncornerreply(data); }
    }
}

