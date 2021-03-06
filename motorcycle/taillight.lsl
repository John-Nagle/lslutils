//
//  Basic tail light script
//
//  Senses movement
//
//  Animats
//  April 2018
//
//  Configuration
//

vector      STOPRED = <1.0,0,0>;
vector      RUNRED = <0.5,0,0>;
integer     face = ALL_SIDES;                       // which face
//          Message IDs from main script
integer     DIR_STOP = 100;
integer     DIR_START = 101;
//  Constants
#define TIMERPERIOD (0.5)                           // smaller values better looking but higher overhead
#define BRAKINGDECEL (1.0)                          // (m/sec^2) Red if slower than this
#define BRAKINGTHRESH (BRAKINGDECEL*TIMERPERIOD)    // delta of velocity to turn on brake light
#define STOPPEDSPEED (0.05)                         // (m/sec) Red if slower than this


vector axis = <0,1,0>;              // axis of rotation
//  Globals
vector gLastColor = <0,0,0>;                        // off
float  gLastGlow = 0.0;                             // not glowing
float  gLastSpeed = 0.0;                            // last speed(signed)
//  Default state - stand by
default
{
    state_entry()
    {
        llSetTimerEvent(0.0);                       // stop timer
        gLastColor = RUNRED*0.5;                    // dull red
        gLastGlow = 0.0;                            // turned off light
        llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_COLOR, face, gLastColor, 1.0]); // light off
        llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_GLOW, face, gLastGlow]);
    }
    
    on_rez(integer param)
    {
        llResetScript();
    }

    //  Message from master script to tell us to start and stop.
    link_message(integer sender, integer num, string message, key id)
    {  if(num == DIR_START)
        {   state Run;   }                                  // bike has been turned on            
    }
}   

state Run {                                                 // bike is running
    
    state_entry() {
        llSetTimerEvent(TIMERPERIOD); 
    }
    
    //  Message from master script to tell us to start and stop.
    link_message(integer sender, integer num, string message, key id)
    {   if(num == DIR_STOP)                                 // if stopping
        {   state default; }                                // back to default state
    }
    
    timer()                                                 // frequently when running
    {
        //  Compute speed in forward direction. Can be positive or negative.
        vector vehdir = <1.0,0.0,0.0>*llGetRootRotation();  // global direction of vehicle
        float speed = llFabs(vehdir*llGetVel());            // speed in fwd direction
        float glow = 0.2;                                   // default running light
        vector color = RUNRED;
        ////llOwnerSay((string)(speed-gLastSpeed));             // ***TEMP***
        if (speed < STOPPEDSPEED || (speed - gLastSpeed) < -BRAKINGTHRESH) // if stopped or slowing
        {   glow = 1.0; 
            color = STOPRED;
        } 
        if (color != gLastColor || glow != gLastGlow)
        {   llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_COLOR, face, color, 1.0]);
            llSetLinkPrimitiveParamsFast(LINK_THIS, [PRIM_GLOW, face, glow]);
            gLastColor = color;
            gLastGlow = glow;
        }
        gLastSpeed = speed;                             // previous speed
    }
}
