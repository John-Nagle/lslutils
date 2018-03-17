//
//  Log to remote server
//
//  Animats
//  March, 2018
//  License: LGPL
//
//  Library for use as an #include for other scripts.
//
//  This takes the SHA1 of the auth token and the message,
//  and puts them it in the HTTP header, so the log message
//  can be authenticated.
//
//  The log message is sent as a string to the server.
//  It can be whatever format the server expects.
//
//  OBSOLETE - use separate script for logging
//
//  Constants
//
//  Globals
//
string gLogURL = "";                        // API endpoint for logging
//  Auth token - values must be known to server
string gLogAuthTokenName = "TEST";          // name for auth token
string gLogAuthTokenValue = "NONE";         // random value for auth token
integer gLogFailCount = 0;                     // HTTP failure count

loginit(string url, string authtokenname, string authtokenvalue)       // set up logging
{   gLogURL = url;
    gLogAuthTokenName = llStringTrim(authtokenname, STRING_TRIM);
    gLogAuthTokenValue = llStringTrim(authtokenvalue, STRING_TRIM);
    gLogFailCount = 0;
}

key logstring(string s)                     // sends a POST to an API endpoint for logging
{   if (gLogURL == "") { return(NULL_KEY); }// logging not enabled
    s = llStringTrim(s,STRING_TRIM);        // trim string for consistency
    list params = [HTTP_METHOD, "POST"];    // send with auth token
    params = params + [HTTP_CUSTOM_HEADER, "X-AUTHTOKEN-NAME"] + gLogAuthTokenName;
    params = params + [HTTP_CUSTOM_HEADER, "X-AUTHTOKEN-HASH"] + llSHA1String(gLogAuthTokenValue + s); // key is SHA1 of of authtoken followed by message
    return(llHTTPRequest(gLogURL, params, s)); // make HTTP request
}

//  Call this for an HTTP response event 
integer logresponse(key request_id, integer status, list metadata, string body)
{   if (status >= 200 && status < 299)
    {   gLogFailCount = 0;                  // reset fail count
        return(TRUE);
    }
    if (gLogFailCount % 10 == 0)
    {   llOwnerSay("Logging request to " + gLogURL + " failed. Status " + (string) status);} // mention every 10th fail
    gLogFailCount++;
    return(FALSE);
}

