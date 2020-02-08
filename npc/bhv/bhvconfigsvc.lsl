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
        return;
    }
    //  Start reading notecard. This may need a retry; dataserver is not reliable.
    gConfigNotecardQuery = llGetNotecardLine(CONFIG_NOTECARD, gConfigNotecardLine);   
}

//
//  parseconfigline -- parse an incoming config line
//
//  Returns error if bad parse
//
integer parseconfigline(string data, integer lineno)
{
    data = llStringTrim(data, STRING_TRIM);                 // remove unwanted whitespace
    if (llStringLength(data) == 0 || llGetSubString(data,0,0) == "#") { return(TRUE); } // comment or blank
    list params = llCSV2List(data);                         // parse string to list
    if (llGetListLength(params) < 2)                        // must have at least two fields
    {   badconfig(data, lineno, "Must have at least two comma-separated fields."); return(FALSE); }
    string scriptname = llList2String(params,0);            // target script name
    params = llDeleteSubList(params,0,0);                   // params without script name
    if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
    {   scriptname = "bhv" + scriptname;                    // preface with bhv and retry
        if (llGetInventoryType(scriptname) != INVENTORY_SCRIPT) // if no such script
        {   badconfig(data, lineno, "No script named \"" + scriptname + "\"."); return(FALSE); }
    }
    // We have a valid script name and params
    string jsn = llList2Json(JSON_OBJECT,["request","config","script",scriptname,"params",llList2Json(JSON_ARRAY,params)]);   // make into JSON
    llOwnerSay(jsn);                                        // ***TEMP*** 
    llMessageLinked(LINK_THIS,LINKMSGCONFIGREQUEST,jsn,""); // send to the other scripts in this prim
    return(TRUE); 
   
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
}

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
    {
        if (query_id == gConfigNotecardQuery)           // reading config line from notecard
        {
            if (data == EOF)                            // done reading notecard
            {   configdone(TRUE);                       // success
            }
            else
            {
                // Get next line, another dataserver request
                ++gConfigNotecardLine;
                integer valid = parseconfigline(data, gConfigNotecardLine); // parse and add patrol point
                if (valid)                                  // if successful parse, get next line
                {   gConfigNotecardQuery = llGetNotecardLine(CONFIG_NOTECARD, gConfigNotecardLine); }
                else
                {   configdone(FALSE); }                    // bad config, won't run
            }
        }
    }
}

