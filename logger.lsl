//
//  Log to remote server
//
//  Animats
//  March, 2018
//  License: LGPL
//
//  This is for use as an #include for other scripts.
//
//  Globals
//
string gLogURL = "";                        // API endpoint for logging
string gLogAuthToken = "XXXX";              // no auth token yet
string gLogOwnerName = "";                  // name of object owner

loginit(string url, string authtoken)       // set up logging
{   gLogURL = url;
    gLogAuthToken = authtoken;
    gLogOwnerName = llKey2Name(llGetOwner());// name of owner
}

key logstring(string s)                     // sends a POST to an API endpoint for logging
{   if (gLogURL == "") { return(NULL_KEY); }// logging not enabled
    s = llStringTrim(s,STRING_TRIM);        // trim string for consistency
    list params = [HTTP_METHOD, "POST", HTTP_CUSTOM_HEADER, "SHA1_AUTHKEY"];   // send with auth key
    params = params + llSHA1String(gLogAuthToken + s); // key is SHA1 of of authtoken followed by message
    llOwnerSay("Log msg: " + s);            // ***TEMP***
    return(llHTTPRequest(gLogURL, params, s)); // make HTTP request
}

key logjson(list items)                     // logs as JSON
{   //  Add some standard data items
    vector pos = llGetPos();
    vector corner = llGetRegionCorner();    // region corner
    items += ["objectname"] + llGetObjectName() 
        + ["objectkey"] + llGetKey()
        + ["owner"] + gLogOwnerName
        + ["regionname"] + llGetRegionName() + ["posx"] + pos.x + ["posy"] + pos.y
        + ["cornerx"] + corner.x + ["cornery"] + corner.y 
        + ["timestamp"] +  llGetUnixTime();             // Note: Year 2038 problem.
    return(logstring(llList2Json(JSON_OBJECT, items))); // generate JSON
}

//  Call this for an HTTP response event 
integer logresponse(key request_id, integer status, list metadata, string body)
{   
    if (status >= 200 && status < 299)
    {   llOwnerSay("HTTP request successful.");     // ***TEMP***
        return(TRUE);
    }
    llOwnerSay("Logging request failed. Status " + (string) status);
    return(FALSE);
}

