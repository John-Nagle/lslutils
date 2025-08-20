//
//  Height recorder test
//
//  Record ground height for sim.
//  Logs to server.
//
//  Animats
//  2025
//
//  License: GPLv3.
//
//
//  TODO:
//  1. Make two passes, find elevation range, scale data accordingly.
//

float INTERVAL = 4;                     // (m) measurement interval
float SIMSIZE = 256;                    // (m) should be a sim property
string HEX = "0123456789ABCDEF";        // hex encoding
float TINY = 0.01;                      // (m) small distance to avoid going over edge
//
//  Globals
//
string gLogURL = "https://www.animats.info/actions/echo.fcgi";                        // API endpoint for logging
//  Auth token - values must be known to server
string gLogAuthTokenName = "TEST";          // name for auth token
string gLogAuthTokenValue = "NONE";         // random value for auth token
integer gLogFailCount = 0;                  // HTTP failure count
//
//  globals
//
float gzoffset;                         // lowest Z value
float gzscale;                          // scale factor for Z values          

//
//  list2hex - Convert list of integers in 0..255 to hex
//
string list2hex(list items)
{   string s = "";
    integer i;
    integer len = llGetListLength(items);
    for (i=0; i<llGetListLength(items); i++)
    {
        integer n = llList2Integer(items,i);
        integer hi = (n >> 4) & 0xf;
        integer lo = n & 0xf;
        string c1 = llGetSubString(HEX,hi,hi);
        string c2 = llGetSubString(HEX,lo,lo);
        s += (c1 + c2);                     // accum string
    }
    return(s); 
}
//
//  bound  -- bound to limits
//
float bound(float val, float lo, float hi)
{   if (val < lo) { val = lo; }
    if (val > hi) { val = hi; }
    return(val);
}
//
//  elevscale -- get terrain elevation bounds for sim
//
//  We have to scan twice, because we don't have enough memory to 
//  hold all those floats.
//
elevscale()
{
    float zmin = 4096.0;                                // bounds
    float zmax = 0;
    float x; float y;
    for (x = 0; x<=SIMSIZE; x += INTERVAL)
    {   for (y=0; y<=SIMSIZE; y += INTERVAL)
        {   float xtest = bound(x,TINY, SIMSIZE-TINY);              // avoid off-sim roundoff errors
            float ytest = bound(y,TINY, SIMSIZE-TINY);
            vector testpos = <xtest,ytest,0> - llGetPos();
            //  Prevent asking for off-sim position
            float z = llGround(testpos);
            if (z < zmin) { zmin = z; }
            if (z > zmax) { zmax = z; }
        }
    }
    llOwnerSay("Z range: " + (string)zmin + " .. " + (string)zmax);
    gzoffset = zmin;
    float zrange = zmax - zmin;
    if (zrange > 0.0)
    {   gzscale = 1.0/zrange; }                             // scale factor for elevs
    else 
    {   gzscale = 0.0; }                                    // this sim is flat
    gzscale = 1.0 / (zmax-zmin);
   
}
//
//
//  elevstojson -- encode terrain elevations for a sim as JSON
//
//  Returns 65 values, not 64
//
string elevstojson(float scale, float offset, float waterlev, string regionname, vector regioncorner)
{   string jsn;
    float x; float y;
    for (x = 0; x<=SIMSIZE; x += INTERVAL)
    {   list elevs = [];
        for (y=0; y<=SIMSIZE; y += INTERVAL)
        {   float xtest = bound(x,TINY, SIMSIZE-TINY);              // avoid off-sim roundoff errors
            float ytest = bound(y,TINY, SIMSIZE-TINY);
            vector testpos = <xtest,ytest,0> - llGetPos();
            //  Prevent asking for off-sim position
            float z = llGround(testpos);
            z = (z-offset)*scale;                                   // scale into 0..1
            integer zint  = llFloor(z*256);
            if (zint < 0) { zint = 0; }
            if (zint > 255) { zint = 255; }
            elevs += zint;
        }
        string s = "\"" + list2hex(elevs) + "\"";                   // enclose hex string in quotes for JSON
        if (x > 0) { s = ",\n" + s; }                               // add comma for non-first line
        jsn += s;                                                   // accumulate data for one X row
    }
    //  Return elevation data as one JSON string
    //  Make lines of hex into separate output lines as JSON because SL mail sender breaks lines at arbitrary points.
    return(llList2Json(JSON_OBJECT, 
        ["region",regionname, "scale",scale, "offset",offset, "waterlev", waterlev,
            "regioncoords",llList2Json(JSON_ARRAY,[llFloor(regioncorner.x/SIMSIZE),llFloor(regioncorner.y/SIMSIZE)]),
            "elevs", ("["+  jsn + "]")]));
}

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
    if (gLogFailCount % 1 == 0)
    {   llOwnerSay("Logging request to " + gLogURL + " failed. Status " + (string) status);} // mention every Nth fail
    gLogFailCount++;
    return(FALSE);
}


default
{
    //  On touch, log to server.
    touch_start(integer total_number)
    {
        llOwnerSay("Reading elevations for region " + llGetRegionName());
        float waterlev = llWater(llGetPos());
        elevscale(); 
        string s = elevstojson(gzscale,gzoffset, waterlev, llGetRegionName(), llGetRegionCorner());             // need to rescale for some high mountains
        string subject = "Elevations for region " + llGetRegionName();  // email message
        llOwnerSay(subject);
        logstring(s);
    }
}



