//  
//  Logger to server
//
//  Animats
//  March 2018
//
//  Use llMessageLinked(LINK_THIS, 0, msg, "LOG") to send a log message to the server endpoint.
//
//  Configuration constants must be included.
//
//  LOGURL - API endpoint for logging
//  AUTHTOKENNAME - name for auth token
//  AUTHTOKENVALUE - random value for auth token (security-critical)
//
#include "logauthinfo.lsl"                              // Do not put this file on public github
#include "logger.lsl"

default
{
    state_entry()
    {
        loginit(LOGURL, AUTHTOKENNAME, AUTHTOKENVALUE);         // set up logging
    }
    link_message(integer Sender, integer Number, string s, key k) // This script is in the object too.
    {
        if (k == "LOG")
        {   logstring(s); }
    }
    
    http_response( key request_id, integer status, list metadata, string body )
    {   logresponse(request_id, status, metadata, body); }

}
