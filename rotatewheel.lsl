//
//  Basic wheel rotation script. 
//
//  Just rotation, no particles.
//  For wheels with actual structure. Rotates the prim, not the texture,
//  but this happens in the viewer, visually, not physically.
//
//  Animats
//  April 2018
//
//  Configuration
//
float       WheelDiameter = 0.30;                   // diameter of wheel
float       TimerPeriod = 0.3;                      // smaller values better looking but higher overhead
float       MaxRate = 10.0;                         // faster than this you can't see it anyway
float       MinAdjust = 0.2;                        // must change by at least this much (fraction)
//          Message IDs from main script
integer     DIR_STOP = 100;
integer     DIR_START = 101;
integer     DIR_NORM = 102;
integer     DIR_LEFT = 103;
integer     DIR_RIGHT = 104;
integer     DIR_FLIGHT = 105;

vector axis = <0,1,0>;              // axis of rotation
//  Globals
float   gLastRate;                                  // previous rate

float abs(float a) { if (a > 0.0) { return(a); }  else { return(-a); }}

//  Default state - stand by
default
{
    state_entry()
    {
        llSetLocalRot(llEuler2Rot(<0, 0, 0.0>));
        llTargetOmega(axis, 0.0, 1.0);              // stop wheel
        gLastRate = 0.0;
        llSetTimerEvent(0.0);                       // stop timer
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
        llSetTimerEvent(TimerPeriod); 
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
        float speed = vehdir*llGetVel();                    // speed in fwd direction
        //  Convert to wheel rotation rate in radians/sec
        float rate = speed / (WheelDiameter*PI);
        if (abs(rate) < 0.01) rate = 0.0;                   // close enough to stopped
        if (rate > MaxRate) rate = MaxRate;                 // bound
        if (rate < -MaxRate) rate = -MaxRate;
        //  Update only if significant change or stopping
        float err = abs(gLastRate - rate);                  // error 
        if ((abs(gLastRate - rate) > MinAdjust*rate) || (gLastRate != 0.0 && rate == 0.0))
        {   llTargetOmega(axis, rate, 1.0);                 // spin wheel visually
            gLastRate = rate;                               // save for next time
        }
    }
}
