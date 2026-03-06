//  Sim impostor upload to database.
//  At this point, the impostor objects exist in the asset server.
//  This tells the database about them.
//
//  Constants
string VERSION = "1.0";
//  Max chars to send at one time. Avoid stack/heap collision
integer MAX_BYTES_SENT = 10000;
//  Service endpoint
string gImpostorURL = "https://www.animats.info/actions/uploadimpostor.fcgi";
//  State
integer gPass = 0;  // We start in pass 0
//  Start at this number next time
integer gStart = 0;
//  True if everything sent for this pass
integer gPassDone = FALSE;
//  Key from last HTTP request
key gLastRequest = "";
//  Total cnt to send, asked once
integer gCnt = 0;
//  Uploading if true
integer gUploading = FALSE;
//  Total asset count
integer gAssetCount = 0;

//  Sends a POST to an API endpoint to update impostor database.
key uploadimpostors(string s)                    
{   s = llStringTrim(s,STRING_TRIM);        // trim string for consistency
    list params = [HTTP_METHOD, "POST"];    // send with auth token
    string url = gImpostorURL + "?version=" + VERSION;
    key status = llHTTPRequest(url, params, s); // make HTTP request
    //////llOwnerSay("HTTP request: " + (string) params); // ***TEMP***
    return status;
}

//  Parse one impostor name
//  DECODE PREFIX_x_y_sx_sy_sz_offset_waterheight_lod_vizgroup_hash
string parse_impostor_data(string name, list fields, key uuid) {
    //  Extract 11 fields from asset name
    string prefix = llList2String(fields,0);
    integer x = (integer) llList2String(fields, 1);
    integer y = (integer) llList2String(fields, 2);
    integer sx = (integer) llList2String(fields, 3);
    integer sy = (integer) llList2String(fields, 4);
    float sz = (float) llList2String(fields, 5);
    float elevation_offset = (float) llList2String(fields, 6);
    integer lod = (integer)llList2String(fields, 7);
    integer viz_group = (integer)llList2String(fields, 8);
    float water_height = (float) llList2String(fields, 9);
    string region_hash = llList2String(fields, 10);
    //  Assemble data
    string region_loc = llList2Json(JSON_ARRAY, [x, y]);
    string scale = llList2Json(JSON_ARRAY, [sx, sy, sz]);
    string region_size = llList2Json(JSON_ARRAY, [sx, sy]);
    string impostor_data = llList2Json(JSON_OBJECT, 
         ["comment", "Generated from sculpt texture UUIDS fetched from inventory",
        "asset_name", name,
        "prefix", prefix,
        "asset_hash", region_hash,
        "region_loc", region_loc,
        "region_size", region_size,
        "grid", llGetEnv("grid"), 
        "elevation_offset", elevation_offset ,
        "scale", scale,
        "water_height", water_height,
        "asset_uuid", uuid,
        "impostor_lod", lod,
        "viz_group", viz_group
    ]);
    return impostor_data;
}

//  Dump all texture UUIDs starting from this value.
//  Returns value to use for start next time.
integer upload_texture_uuid_block(integer start, integer cnt, integer pass) 
{
    llOwnerSay("Starting UUID block at " + (string)start + ", pass " + (string)pass);

    integer n;
    string impostors = "";
    //  Need to make two passes. Textures first.
    //  No mesh support yet.
    string desired_prefix = "RT0";
    if (pass == 1)
    {   desired_prefix = "RS"; 
    }
    llOwnerSay("Uploading " + desired_prefix);
    for (n=start; n<cnt; n++)
    {
        string name = llGetInventoryName(INVENTORY_TEXTURE, n);
        key uuid = llGetInventoryKey(name);
        list fields = llParseStringKeepNulls(name, ["_"], []);       
        string prefix = llList2String(fields,0);
        if (prefix == desired_prefix) 
        {
            gAssetCount++;
            string impostor_data = parse_impostor_data(name, fields, uuid);
            if (impostors == "")
            {   impostors = impostors + "\n" + impostor_data;
            } else {
                impostors = impostors + ",\n" + impostor_data;
            }
            if (llStringLength(impostors) > MAX_BYTES_SENT)
            {
                impostors = "[\n" + impostors + "\n]\n";
                llOwnerSay((string) (llStringLength(impostors)) + " bytes of impostors being sent to server.");
                uploadimpostors(impostors);
                impostors = "";
                return (n+1);
            }
        }
    }
    //  Always send, even if empty, so we get a completion.
    impostors = "[\n" + impostors + "\n]\n";
    llOwnerSay((string) (llStringLength(impostors)) + " bytes of impostors being sent to server.");
    uploadimpostors(impostors);
    return (-1);    // EOF
}

//  upload_texture_uuid_block()
//  Called once at start, then on each HTTP reply.
//  Runs through all the UUIDs twice, once for each pass.
upload_next_texture_uuid_block()
{
    llOwnerSay("Upload next block, pass " + (string) gPass + " starting from #" + (string) gStart);
    if (gPassDone) {
        if (gPass != 0) {
            llOwnerSay("Pass 2 complete. Total assets uploaded: " + (string) gAssetCount + ". Done.");
            gUploading = FALSE;
            return;
        } else {
            llOwnerSay("Pass 1 complete.");
            gPass = 1;
            gStart = 0;
        }
    }
    //  Do the upload.
    integer nextStart = upload_texture_uuid_block(gStart, gCnt, gPass);
    if (nextStart < 0) {
        gPassDone = TRUE;
    } else {
        gStart = nextStart;
    }
}

//  Start the upload process
upload_all_texture_uuids() {
    if (gUploading)
    {   llOwnerSay("BUSY - uploading");
        return;
    }
    gUploading = TRUE;
    gPass = 0;
    gStart = 0;
    gCnt = llGetInventoryNumber(INVENTORY_TEXTURE);
    gPassDone = FALSE;
    upload_next_texture_uuid_block();
}

default
{
    touch_start(integer total_number)
    {
        upload_all_texture_uuids();
    }
    
    //  Response from upload.
    http_response(key HTTPRequestID, integer status, list metadata, string body) 
    {      
        //////llOwnerSay("HTTP response: " + body);
        if (status == 200) 
        {    upload_next_texture_uuid_block();
        } else {
            llOwnerSay("ERROR " + (string) status + ": " + body);
            gUploading = FALSE;
        }
    }
}

