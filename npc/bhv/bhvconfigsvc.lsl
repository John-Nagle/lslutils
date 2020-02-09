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
#include "npc/bhv/bhvcall.lsl"


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
    {   ////llOwnerSay("Dataserver callback: " + data);     // ***TEMP***
        //  Handle all the various data server events
        if (query_id == gBhvConfigNotecardQuery)       {   bhvParseConfigLine(data, gBhvConfigNotecardLine);}
        else if (query_id == gLookupRegionStatusQuery) { lookupregionstatusreply(data); }
        else if (query_id == gLookupRegionCornerQuery) { lookupregioncornerreply(data); }
    }
}

