integer gOn;                                    
vector gStartPos;                               
rotation gStartRot;                             
integer gDirection = 0;                         
////vector gPusherRelPos;                           
////integer PUSHERPRIMLINK = 2;                     
////vector PUSHERRETRACT = <0,0,-0.30>;             
float CYCLETIME = 0.5;
float MOTIONDIST; 
float TEXCYCLETIME;                             
float TEXSCALEX = 0.5;                          
float STOPTIME = 10.0;   
float SENSORDIST = 50.0;                // turn on if this close  
vector BOUNDINGBOXERROR = <0.05,0.05,0.05>; // resykt from llGetBoundingBox is oversized by this much                     

vector STEPMOVE = <-0.0,-0.35,0.23>;


stop_belt()
{
    llSetTimerEvent(0.0);                       
    llSetKeyframedMotion([],[]);                
    gOn = FALSE;
    llSleep(CYCLETIME*5.0);                     
    llSetPos(gStartPos);                        
    llSetRot(gStartRot);
}                 

start_belt(integer direction)                   
{
    if (gOn) { return; } 
    if (direction == 0) { return; }                           
    vector motionDir = STEPMOVE;    
    motionDir = motionDir * gStartRot;          
    
    if (direction > 0)                          
    {   ////llSetLinkPrimitiveParams(PUSHERPRIMLINK, [PRIM_POS_LOCAL, gPusherRelPos]); 
        llSetKeyframedMotion([motionDir, CYCLETIME],[KFM_DATA,KFM_TRANSLATION, KFM_MODE,KFM_LOOP]); 
    }
    else                                        
    {                                                   
        ////llSetLinkPrimitiveParams(PUSHERPRIMLINK, [PRIM_POS_LOCAL, gPusherRelPos + PUSHERRETRACT]); 
        ////llSetKeyframedMotion([motionDir, CYCLETIME],[KFM_DATA,KFM_TRANSLATION, KFM_MODE, KFM_FORWARD]);
        ////llSleep(CYCLETIME*2.0);                 
        llSetKeyframedMotion([-motionDir, CYCLETIME],[KFM_DATA,KFM_TRANSLATION, KFM_MODE,KFM_LOOP]);
    }
    gOn = TRUE;                                 
    llSetTimerEvent(STOPTIME);                  
}

init()
{   
    MOTIONDIST = CYCLETIME*2.0;
    MOTIONDIST = CYCLETIME*1.0; 
    TEXSCALEX = MOTIONDIST*0.5;
    TEXSCALEX = 0.5/MOTIONDIST;
    TEXCYCLETIME = 1.0/CYCLETIME;
    llSetKeyframedMotion([],[]);                
    gOn = FALSE;                                
    llSleep(CYCLETIME*2.0);                     
    gStartPos = llGetPos();                     
    gStartRot = llGetRot();
    ////gPusherRelPos = llList2Vector(llGetLinkPrimitiveParams(PUSHERPRIMLINK, [PRIM_POS_LOCAL]),0);
    llSensorRepeat("","",AGENT,SENSORDIST, PI, STOPTIME);    // always be scanning
}

        
default
{
    state_entry()
    {   

        MOTIONDIST = CYCLETIME*2.0;
        MOTIONDIST = CYCLETIME*1.0; 
        TEXSCALEX = MOTIONDIST*0.5;
        TEXSCALEX = 0.5/MOTIONDIST;
        TEXCYCLETIME = 1.0/CYCLETIME;
        llSetKeyframedMotion([],[]);
        llSetLinkPrimitiveParams(LINK_THIS, [PRIM_SCRIPTED_SIT_ONLY, TRUE]); // cannot sit on belt                
        gOn = FALSE;                                
        llSleep(CYCLETIME*2.0);                     
        gStartPos = llGetPos();                     
        gStartRot = llGetRot();
        ////gPusherRelPos = llList2Vector(llGetLinkPrimitiveParams(PUSHERPRIMLINK, [PRIM_POS_LOCAL]),0);
        llSensorRepeat("","",AGENT,SENSORDIST, PI, STOPTIME);                               // always be scanning
    }
    
    sensor(integer num_detected)
    {
        start_belt(gDirection);
    }   
    
    no_sensor()
    {
        stop_belt();          
    }   
    
    on_rez(integer id)
    {   
        ////llOwnerSay("Listening on channel " + (string)id);   // ***TEMP***
        if (id == 0)
        {   llOwnerSay("Rezzed manually for test. No automatic install."); return; }
        integer listenHandle_a = llListen(id, "", "", "");  // must be "DIE" on this channel  
        init();                                 // set up   
    }    
    listen(integer channel, string name, key listenid, string message)
    {   
        list msglist = llJson2List(message);
        ////llOwnerSay("Belt command: " + message); // ***TEMP***
        string cmd = llJsonGetValue(message, ["command"]);    // what are we supposed to do?
        if (cmd == "DIE")
        {   llDie(); return; }                      // controller wants us to go away
        if (cmd == "DIR")                           // ordered to change direction
        {   integer direction = (integer)llJsonGetValue(message, ["direction"]); // get size
            stop_belt();
            gDirection = direction;
            start_belt(gDirection);
            return; 
        }
        if (cmd == "REFPOS")                                // fine-tune step position to match top platform
        {   vector refpt = (vector)llJsonGetValue(message, ["refpt"]);  // point to align to center of top of steps
            //  Steps should already be oriented properly. Now we figure out the offset to align step top with upper platform.
            list bounds = llGetBoundingBox(llGetKey());     // bounds of the steps
            vector lobound = llList2Vector(bounds,0) + BOUNDINGBOXERROR;       // bounds of object
            vector hibound = llList2Vector(bounds,1) - BOUNDINGBOXERROR;
            vector toppt = <(hibound.x + lobound.x)*0.5, lobound.y, hibound.z>; // top pt in object coords
            //  Center check
            llOwnerSay("Bounds: " + llDumpList2String(bounds,","));  // ***TEMP***
            gStartRot = llGetRot();                         // rotation for steps
            gStartPos = refpt - toppt*gStartRot;            // align to make steps match upper platform
            llOwnerSay("Aligning steps to " + (string)gStartPos); // ***TEMP***
            stop_belt();                                    // stop before moving, position to home pos
            ////llSetPos(gStartPos);                            // move there
            return;
        }
        llOwnerSay("Unrecognized command: " + message); // not expected
    }
}

