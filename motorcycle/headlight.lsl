//  Turn headlight on and off with bike
integer     DIR_STOP = 100;
integer     DIR_START = 101;

float GLOW_VALUE = 0.15;                          // just a little glow
integer GLOW_FACE = ALL_SIDES;                      // need to find front face

default
{    
    link_message(integer sender_num, integer num, string msg, key id)
    {   list params  = llGetPrimitiveParams([PRIM_POINT_LIGHT]);    // get params to modify
        ////llOwnerSay("Headlight: " + (string)num + " params: " + llList2CSV(params));
        if (num == DIR_STOP)                        // stopping, turn light off
        {   params = [PRIM_POINT_LIGHT] + llListReplaceList(params, [0.0],2,2);       // light off
            llSetLinkPrimitiveParamsFast(LINK_THIS, params);    // set light params
            llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_GLOW, GLOW_FACE, 0.0]); // glow off
        } else if (num == DIR_START)                // starting, turn light on
        {   params = [PRIM_POINT_LIGHT] + llListReplaceList(params, [1.0],2,2);       // light on
            llSetLinkPrimitiveParamsFast(LINK_THIS, params);    // set light params
            llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_GLOW, GLOW_FACE, GLOW_VALUE]); // glow on
        }
    }
}

