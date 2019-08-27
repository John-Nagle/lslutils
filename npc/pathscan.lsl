//
//  patscan.lsl -- component of a path building system
//
//  Part of a system for doing pathfinding in Second Life
//
//  Raycasts for obstacles while character is moving.
//
//  Handles stopping when there's a problem - obstacle or collision.
//  Also handles movement_end messages.
//
//  Communicates with pathexecute.lsl, which got too big, so
//  it had to be split.
//
//  ***UNTESTED***
//
//
//  Animats
//  August, 2019
//
//  License: GPLv3
//
///
#include "npc/assert.lsl"                                   // assert
#include "npc/patherrors.lsl"
#include "npc/mazedefs.lsl"
#include "npc/pathbuildutils.lsl"
//
//  Constants
//
//  ***MOVE TO COMMON FILE***
#define LINKMSGSCANREQUEST  1010                            // message types
#define LINKMSGSCANREPLY    1011
//
//  Idle timer timeout
//
#define PATHEXETIMEOUT      2.0                             // check this often for progress
#define PATHSCANRAYTIME      0.2                            // do a cast ray for obstacles this often
#define PATHMINTURNSECTION  0.5                             // first section of path this length, for a starting turn
#define PATHMOVECHECKSECS   2.0                             // check this often for progress
#define PATHEXELOOKAHEADDIST    10.0                        // (m) distance to look ahead for obstacles while moving
#define PATHEXEMAXCREEP     0.10                            // (m) max positional error allowed after keyframe motion

//
//  Globals
//
integer gPathScanMsgLevel = 0;                              // message level for debug                             
integer gPathScanId = 0;                                    // current path ID
integer gPathScanActive = FALSE;                            // path system is doing something
integer gPathScanFreemem;                                   // amount of free memory left
integer gPathScanTimetick = 0;                              // last time we tested for motion
vector gPathScanLastpos = ZERO_VECTOR;                      // last place we tested for motion
key gPathScanSelfObject;                                    // our own key, needed for raycast test

//  Avatar params
float gPathScanWidth = 1.0;                                  // defaults, overridden by messages
float gPathScanHeight = 1.0;
integer gPathScanChartype = CHARACTER_TYPE_A;

////float gPathScanProbespacing = 0.333;                         // (m) cast ray spacing when looking for obstacles

//
//  Segment storage
//  Just a list of vectors
//
list gKfmSegments = [];                                     // segments being executed by current KFM operation
integer gKfmSegmentCurrent = 0;                             // which segment we are currently on

//
//  pathscaninit -- set up path execute parameters
//
pathscaninit()
{   gPathScanSelfObject = llGetKey();                       // us
    gPathScanFreemem = llGetFreeMemory();   
}

//
//  pathscandone - done here, report to execute task
//
//  Zero status here means ordinary movement end, no problems.
//
pathscandone(integer status, key hitobj)
{   if (gPathScanActive && status != 0)                     // if something bad happened
    {   llSetKeyframedMotion([],[KFM_COMMAND, KFM_CMD_STOP]);// stop whatever is going on
        pathMsg(PATH_MSG_WARN, "Stopped by obstacle " + llKey2Name(hitobj) + " status: " + (string)status);    
    }
    //  Return "scandone" to exec module
    list params = ["reply", "scandone", "status", status, "pathid", gPathScanId, "hitobj", hitobj];
    llMessageLinked(LINK_THIS, LINKMSGSCANREPLY, llList2Json(JSON_OBJECT,params), "");   // Return result to execute task
    gPathScanActive = FALSE;                                // no longer active
}
//
//  pathscanmovementend -- movement has finished, feed in next section if any
//
pathscanmovementend()
{   gPathScanActive = FALSE;                                    // not moving
    gKfmSegments = [];                                          // no current segments
    gPathScanLastpos = ZERO_VECTOR;                             // no last moving pos   
    pathMsg(PATH_MSG_INFO,"Movement end");
    pathscandone(0, "");                                        // normal event
}

//
//  pathobstacleraycast -- check for obstacle ahead
//
//  Cast ray from p to p1.
//
pathobstacleraycast(vector p, vector p1)
{   
    //  One simple ahead ray cast for now.
    list castresult = castray(p, p1, PATHCASTRAYOPTSOBS);
    //  We have to do the whole analysis drill. Ground or walkable, OK. Self, OK.
    //  Anything else is an obstacle
    list castanalysis = pathanalyzecastresult(castresult, FALSE);
    if (castanalysis == []) return;                         // no problem
    if (llGetListLength(castanalysis) == 1)                 // error status
    {   pathscandone(llList2Integer(castanalysis,0), NULL_KEY); return; }  // report error
    key hitobj = llList2Key(castanalysis,0);                // result is [obj, hitpt]
    vector hitpt = llList2Vector(castanalysis,1);
    pathMsg(PATH_MSG_WARN,"Move stopped by obstacle: " + llList2String(llGetObjectDetails(hitobj,[OBJECT_NAME]),0) 
                    + " at " + (string)(hitpt) + " by ray cast from " + (string)p + " to " + (string)p1);
    pathscandone(PATHEXEOBSTRUCTED, hitobj);  // report trouble
}
//
//  pathcheckdynobstacles  -- check for dynamic obstacles encountered while moving.
//
//  Such as other KFM characters, which are not collidable.
//
pathcheckdynobstacles()
{
    //  We need to find out which segment of the path we are currently in.
    float lookaheaddist = PATHEXELOOKAHEADDIST;     // distance to look ahead
    float PATHEXESEGDISTTOL = 0.20;                 // how close to segment to be in it. Keyframe error causes trouble here.
    float HUGE = 99999999999.0;                     // huge number, but INFINITY is bigger
    vector posoffset = <0,0,gPathScanHeight*0.5>;    // pos is at object midpoint, points are at ground
    vector pos = llGetPos() - posoffset;            // where we are now
    integer i;
    integer foundseg = FALSE;
    vector startpos = llList2Vector(gKfmSegments,0);   // start of path
    //  Start at segment where last found the position.  
    //  Stop at end of list, finished lookaheaddist, or no longer moving.
    //  pos and all segment points are at ground level.
    for (i=gKfmSegmentCurrent; i<llGetListLength(gKfmSegments)-1 && lookaheaddist > 0 && gPathScanActive; i++)
    {   vector p0 = llList2Vector(gKfmSegments,i);
        vector p1 = llList2Vector(gKfmSegments,i+1);
        if (!foundseg) 
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHEXESEGDISTTOL); // distance along seg, or INFINITY
            if (distalongseg < HUGE)    
            {   
                gKfmSegmentCurrent = i;                 // advance current segment pos
                foundseg = TRUE;                        // start checking from here                
            }
        }
        if (foundseg)                                   // if pos is in this or a previous segment
        {   float distalongseg = pathdistalongseg(pos, p0, p1, PATHEXESEGDISTTOL); // distance along seg, or NAN
            if (distalongseg < HUGE)                    // point is in capsule around segment
            {   
                float seglength = llVecMag(p0-p1);
                if (distalongseg < 0) { distalongseg = 0; } // bound to segment
                if (distalongseg > seglength) { distalongseg = seglength; }
                float castdist = seglength - distalongseg;  // cast to end of segment
                if (lookaheaddist < castdist) { castdist = lookaheaddist; } // if running out of cast distance
                if  (castdist <= 0) { return; };        // at end
                vector pos2 = pos + llVecNorm(p1-p0)*castdist; // how far to cast            
                pathobstacleraycast(pos+posoffset, pos2+posoffset);          // look ahead horizontally       
                lookaheaddist -= castdist;              // reduce distance ahead
                pos = p1;                               // start of next segment is start of next cast
            }
        }   
    }
    if (!foundseg)
    {   pathMsg(PATH_MSG_ERROR,"Unable to find " + (string)pos + " in " + llDumpList2String(gKfmSegments,",")); } // off the path?
}
//  
//
//  pathscantimer  -- timer event, check progress and do ray casts
//
pathscantimer()
{   if (gPathScanActive && gKfmSegments != [])                   // if we are moving and have a path
    {   pathcheckdynobstacles(); }                              // ray cast for obstacles
    if (gPathScanActive)                                         // if we are supposed to be moving
    {   
        integer now = llGetUnixTime();                          // time now
        if (now - gPathScanTimetick > PATHMOVECHECKSECS)
        {   gPathScanTimetick = now;                            // update last check time
            vector pos = llGetPos();
            if (llVecMag(pos - gPathScanLastpos) > 0.01)         // if moving at all
            {   return; }                                       // OK
            //  No KFM movement. Something has gone wrong. 
            pathscandone(MAZESTATUSKFMSTALL, NULL_KEY);         // stalled
        }
    }
}

//
//  pathscanrequest  -- JSON from path execution
//
//  We get a list of the points the character is currently following. 
//  That tells us what direction to look for obstacles.
//
pathscanrequest(string jsn) 
{   pathMsg(PATH_MSG_INFO,"Path scan request: " + jsn);
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type  
    if (requesttype == "startscan") 
    {   //  Set up for ray casting.
        gPathScanId = (integer)llJsonGetValue(jsn, ["pathid"]);
        gPathScanWidth = (float)llJsonGetValue(jsn,["width"]);
        gPathScanHeight = (float)llJsonGetValue(jsn,["height"]);
        gPathScanMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);
        list ptsstr = llJson2List(llJsonGetValue(jsn, ["points"])); // points, as strings
        gKfmSegments = [];
        integer i;
        integer len = llGetListLength(ptsstr);
        for (i=0; i<len; i++) { gKfmSegments += (vector)llList2String(ptsstr,i); } // convert JSON strings to LSL vectors
        llSetTimerEvent(PATHSCANRAYTIME);                   // switch to fast timer for ray casts for obstructions
    } else if (requesttype == "stopscan")                   // stop scanning
    {   gKfmSegments = [];
        llSetTimerEvent(0.0);                               // shut down and stop timer
    } else {
        pathMsg(PATH_MSG_ERROR,"Path stop rcvd bad msg: " + jsn);
    }
}

//
//  pathscancollision -- collided with something
//
pathscancollision(integer num_detected)
{   
    if (!gPathScanActive) { return; }    // not moving, not our fault
    integer i;
    for (i=0; i<num_detected; i++)
    {   key hitobj = llDetectedKey(i);
        if (hitobj != NULL_KEY)              // null key is land
        {   list details = llGetObjectDetails(hitobj, [OBJECT_PATHFINDING_TYPE]);
            integer pathfindingtype = llList2Integer(details,0);    // get pathfinding type
            if (pathfindingtype != OPT_WALKABLE)                    // hit a non-walkable
            {   pathMsg(PATH_MSG_INFO,"Collided with " + llDetectedName(i));
                pathscandone(PATHEXECOLLISION, llDetectedKey(i)); // stop
                return;
            }
        }
    }
}

//
//  The main program of the scan task.
//
default
{
    state_entry()
    {  pathscaninit();                                // init our KFM system        
    }

    link_message(integer status, integer num, string jsn, key id )
    {   if (num == LINKMSGSCANREQUEST)                           // maze solve result
        {   pathscanrequest(jsn);
        }
    }
    
    timer()
    {   pathscantimer();   }                                      // pass timer event
    
    moving_end()
    {   pathscanmovementend(); }   
    
    collision_start(integer num_detected)
    {   pathscancollision(num_detected); }
}
