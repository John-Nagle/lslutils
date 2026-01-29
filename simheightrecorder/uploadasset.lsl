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

//  Sends a POST to an API endpoint to update impostor database.
key uploadimpostors(string s)                    
{   s = llStringTrim(s,STRING_TRIM);        // trim string for consistency
    list params = [HTTP_METHOD, "POST"];    // send with auth token
    string url = gImpostorURL + "?version=" + VERSION;
    return(llHTTPRequest(url, params, s)); // make HTTP request
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
        "region_hash", region_hash,
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

dump_all_texture_uuids()
{
    integer cnt = llGetInventoryNumber(INVENTORY_TEXTURE);
    llOwnerSay((string) cnt + " textures.");
    integer n;
    string impostors = "";
    integer asset_count = 0;
    //  Need to make two passes. Textures first.
    //  No mesh support yet.
    integer pass;
    for (pass=0; pass<2; pass++) 
    {
        string desired_prefix = "RT0";
        if (pass == 1)
        {   desired_prefix = "RS"; 
        }
        llOwnerSay("Uploading " + desired_prefix);
        for (n=0; n<cnt; n++)
        {
            string name = llGetInventoryName(INVENTORY_TEXTURE, n);
            key uuid = llGetInventoryKey(name);
            list fields = llParseStringKeepNulls(name, ["_"], []);       
            string prefix = llList2String(fields,0);
            if (prefix == desired_prefix) 
            {
                asset_count++;
                string impostor_data = parse_impostor_data(name, fields, uuid);
                if (impostors == "")
                {   impostors = impostors + "\n" + impostor_data;
                } else {
                    impostors = impostors + ",\n" + impostor_data;
                }
                if (llStringLength(impostors) > MAX_BYTES_SENT)
                {
                    impostors = "[\n" + impostors + "\n]\n";
                    llOwnerSay((string) (llStringLength(impostors)) + " bytes of impostors sent to server");
                    uploadimpostors(impostors);
                    impostors = "";
                }
            }
        }
    }
    if (impostors != "")
    {   impostors = "[\n" + impostors + "\n]\n";
        uploadimpostors(impostors);
    }
    llOwnerSay((string) asset_count + " assets sent to server.");
}

default
{
    state_entry()
    {
    }

    touch_start(integer total_number)
    {
        dump_all_texture_uuids();
    }
}

