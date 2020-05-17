//
//  pathprep.lsl -- components of a path building system
//
//  Part of a system for doing pathfinding in Second Life.
//
//  This is prelminary processing for the planning task.
//
//  Animats
//  June, 2019
//
#include "npc/pathbuildutils.lsl"                           // common functions
#include "npc/pathmazesolvercall.lsl"
#include "npc/pathmovecall.lsl"                             // for recover
#include "npc/mathutils.lsl"
//
//  Constants
//
float MINSEGMENTLENGTH = 0.025; /// 0.10;                   // minimum path segment length (m)
#define MAXREGIONMOVE (1000.0)                              // cap move at 1km
#define NAVMESHFINDTOL (0.10)                               // increment when searching for navmesh

list TRIALOFFSETS = [<-1,0,0>,<1,0,0>,<0,1,0>,<0,-1,0>]; // try coming from any of these directions for start point validation
//
//  Globals
//
float gPathprepSpeed;
float gPathprepTurnspeed;
key gPathprepTarget;
integer gPathprepPathid;
vector gRefPt;                                              // reference point, a region corner, for coordinates
//
//  Stuck detection
//
#define PATHPREPSTALLTIME 300.0                             // stalled if no move in this time
#define PATHPREPSTALLTRIES 20                               // and in this many tries
integer gPathprepLastMovedPathid = 0;                       // last pathid on which character moved
integer gPathprepLastMovedTime = 0;                         // last time moved
vector  gPathprepLastMovedPos = ZERO_VECTOR;                // last position seen moved
vector  gPathprepPrevPos = ZERO_VECTOR;                     // previous position, for navmesh stuck check
//
//  pathstuck -- are we stuck?
//
//  A next to last ditch check for breaking out of stuck conditions after many minutes.
//
integer pathstuck(vector pos)
{   if (llVecMag(pos-gPathprepLastMovedPos) > 0.10)         // if we moved
    {   gPathprepPrevPos = gPathprepLastMovedPos;           // previous place we were good at
        gPathprepLastMovedPos = pos;                        // record good pos
        gPathprepLastMovedTime = llGetUnixTime();           // time
        gPathprepLastMovedPathid = gPathprepPathid;         // path sequence number
        return(FALSE);                                      // not stalled
    }          
    if ((llGetUnixTime() - gPathprepLastMovedTime) < PATHPREPSTALLTIME) { return(FALSE); } // not stalled yet
    integer pathiddiff = gPathprepPathid - gPathprepLastMovedPathid;    // difference in pathid
    if (pathiddiff < PATHPREPSTALLTRIES && pathiddiff > 0) { return(FALSE); } // not enough fails, and pathid didn't wrap around
    return(TRUE);                                           // stuck, time for a recovery
}

//
//  pathdeliversegment -- path planner has a segment to be executed
//
//  Maze segments must be two points. The maze solver will fill in more points.
//  Other segments must be one or more points.
//
//  DUMMY VERSION - only for error messages during prep
//
pathdeliversegment(list path, integer ismaze, integer isdone, integer pathid, integer status)
{   //  Fixed part of the reply. Just add "points" at the end.
    list fixedreplypart = ["reply","path", "pathid", pathid, "status", status, "hitobj", NULL_KEY,
                "target", gPathprepTarget, "speed", gPathprepSpeed, "turnspeed", gPathprepTurnspeed,               // pass speed setting to execution module
                "width", gPathWidth, "height", gPathHeight, "chartype", gPathChartype, 
                "points"];                                  // just add points at the end
    assert(path == []);
    assert(!ismaze);                                        // not during prep
    assert(isdone);                                         // only for the done case with an error
    llMessageLinked(LINK_THIS,MAZEPATHREPLY,
            llList2Json(JSON_OBJECT, ["segmentid", 0] + fixedreplypart + [llList2Json(JSON_ARRAY,[ZERO_VECTOR])]),"");    // send one ZERO_VECTOR segment as an EOF.
}
//
//  dopathturn -- turn in place, synchronous, then cause a callback.
//
//  This can preempt a move, but we stop KFM before doing it, so this is safe against race conditions.
//
dopathturn(float heading, integer pathid)
{
    vector facedir = <llSin(heading), llCos(heading), 0.0>;     // face in programmed direction
    float TURNRATE = 90*DEG_TO_RAD;                             // (deg/sec) turn rate
    float PATH_ROTATION_EASE = 0.5;                             // (0..1) Rotation ease-in/ease out strength
    rotation endrot =  RotFromXAxis(facedir);                   // finish here
    rotation startrot = llGetRot();                             // starting from here
    float turntime = llFabs(llAngleBetween(startrot, endrot)) / TURNRATE;  // how much time to spend turning
    integer steps = llCeil(turntime/0.200 + 0.001);             // number of steps 
    integer i;
    for (i=0; i<= steps; i++)                                   // turn in 200ms steps, which is llSetRot delay
    {   float fract = ((float)i) / steps;                       // fraction of turn (0..1)
        float easefract = easeineaseout(PATH_ROTATION_EASE, fract); // smooth acceleration
        llSetRot(slerp(startrot, endrot, easefract));           // interpolate rotation
    }
    pathdonereply(PATHERRMAZEOK, NULL_KEY, pathid);             // normal completion
}
//
//  isoutsideregion -- true if point is outside region
//
#define REGION_HYSTERESIS_DIST (2.0)                            // get this far out to be sure of region cross
integer isoutsideregion(vector p)
{   return(p.x < -REGION_HYSTERESIS_DIST || p.x > REGION_HYSTERESIS_DIST+REGION_SIZE \
        || p.y < -REGION_HYSTERESIS_DIST || p.y > REGION_HYSTERESIS_DIST+REGION_SIZE);
}
    
//
//  pathtrimmedregionbound  -- trim path just past region boundary
//
//  We cross the region boundary, stop, and then continue via recovery
//
list pathtrimmedregionbound(list pts)
{
    integer i;
    integer len = llGetListLength(pts);
    for (i=1; i<len; i++)
    {   vector p = llList2Vector(pts,i);
        if (isoutsideregion(p))                     // if point is beyond region edge by at least 2m
        {   pathMsg(PATH_MSG_WARN, "Last point of region-trimmed path: " + (string)p); // ***TEMP***
            return(llList2List(pts,0,i));           // keep points up to this off-region one.
        }
    }
    return(pts);                                    // no region cross
}
//
//  Main program of the path pre-planning task
//
default
{
    state_entry()
    {   pathinitutils(); }                                              // library init

    //
    //  Incoming link message - will be a plan job request.
    //    
    link_message(integer status, integer num, string jsn, key id)
    {   if (num == PATHPLANREQUEST)                                     // if request for a planning job
        {   
            //  First, get stopped before we start planning the next move.
            llMessageLinked(LINK_THIS,MAZEPATHSTOP, "",NULL_KEY);       // tell execution system to stop
            integer i = 100;                                            // keep testing for 100 sleeps
            vector startpos = llGetPos();
            float poserr = INFINITY;                                    // position error, if still moving
            do                                                          // until character stops
            {   llSleep(0.3);                                           // allow time for message to execute task to stop KFM
                vector pos = llGetPos();
                poserr = llVecMag(startpos - pos);                      // how far did we move
                startpos = pos;
                pathMsg(PATH_MSG_INFO,"Waiting for character to stop moving.");
            } while (i-- > 0 && poserr > 0.001);                        // until position stabilizes 
            if (i <= 0) { pathMsg(PATH_MSG_WARN, "Character not stopping on command, at " + (string)startpos); }
            //  Common fields
            gPathprepPathid = (integer)llJsonGetValue(jsn,["pathid"]);  // starting new pathid
            string request = llJsonGetValue(jsn,["request"]);           // request type
            if (request == "pathturn")                                  // is this a turn request?
            {   dopathturn((float)llJsonGetValue(jsn,["heading"]),gPathprepPathid);  // do turn
                return;
            }
            assert(request == "pathprep");                              // if not turn, had better be pathprep             
            //  This is a path move request.                     
            //  Starting position and goal position must be on a walkable surface, not at character midpoint. 
            //  We just go down slightly less than half the character height.
            //  We're assuming a box around the character.      
            startpos = llGetPos();                                      // startpos is where we are now
            vector startscale = llGetScale();
            startpos.z = (startpos.z - startscale.z*0.45);              // approximate ground level for start point
            vector goalregioncorner = (vector)llJsonGetValue(jsn,["regioncorner"]); // region of goal point
            vector goal = (vector)llJsonGetValue(jsn,["goal"]);     // get goal point
            gPathprepTarget = (key)llJsonGetValue(jsn,["target"]);  // get target if pursue
            float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
            float testspacing = (float)llJsonGetValue(jsn,["testspacing"]);
            gPathprepSpeed = (float)llJsonGetValue(jsn,["speed"]);
            gPathprepTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]);
            pathMsg(PATH_MSG_INFO,"Path request: " + jsn); 
            jsn = "";                                               // Release string. We are that tight on space.
            //  Call the planner
            pathMsg(PATH_MSG_INFO,"Pathid " + (string)gPathprepPathid + " prepping."); 
            //
            //  Goal adjustment if region crossed.
            vector pos = llGetPos();                                // we are here in current region
            gRefPt = llGetRegionCorner();                           // region corner of current region
            if (goalregioncorner == ZERO_VECTOR) { goalregioncorner = gRefPt; } // use this region if blank
            if (goal != ZERO_VECTOR)
            {   if (goalregioncorner != gRefPt)                     // if this move crosses a region boundary
                {
                    vector regiondiff = goalregioncorner - gRefPt; 
                    if (llVecMag(regiondiff) > MAXREGIONMOVE)       // unlikely
                    {   pathMsg(PATH_MSG_ERROR,"Huge cross-region move from " + (string)(gRefPt+pos) + " to " + (string)(goalregioncorner+goal));
                    }
                    goal += regiondiff;                             // goal is in coordinates of current region
                    pathMsg(PATH_MSG_WARN, "Cross-region path to " + (string)goal); // legit
                }
            }
            //  Are we already there?
            if (pathvecmagxy(pos - goal) < 0.005 && llFabs(pos.z - goal.z) < gPathHeight) // if we are already there
            {
                pathdeliversegment([], FALSE, TRUE, gPathprepPathid, PATHERRMAZEOK);// report immediate success and avoid false stuck detection
                return;
            }
            if (pathstuck(pos))                                     // check for trying over and over and getting nowhere.
            {   
                pathMsg(PATH_MSG_ERROR,"Stuck at " + (string)pos + ". Recovering.");    // err for debug purposes, change to warn later
                pathmoverecover(gPathprepPathid);                   // force return to a previous good point
            }
            //  Quick sanity check - are we in a legit place?            
            vector fullheight = <0,0,gPathHeight>;                  // add this for casts from middle of character
            vector halfheight = fullheight*0.5;   
            vector p = pos-halfheight;                              // position on ground
            integer good = FALSE;                                   // not yet a good start point
            if (pathisinregion(p))                                  // if point is in region
            {   for (i=0; i<llGetListLength(TRIALOFFSETS); i++)     // try coming from all around the start point
                {   if (!good)
                    {   vector refpos = p + llList2Vector(TRIALOFFSETS,i)*gPathWidth; // test at this offset
                        good = pathcheckcelloccupied(refpos, p, TRUE, FALSE) >= 0.0;
                    }
                }
            } else {                                                // if off-region
                good = TRUE;                                        // assume valid dest if off-region, because we can't see off-region
            }
            if (!good)
            {   pathMsg(PATH_MSG_WARN, "Start location is not clear: " + (string)startpos);           // not good, but we can recover
                //  Ask the move task to attempt recovery. This will move to a better place and return a status to the retry system.
                pathmoverecover(gPathprepPathid);                       // attempts recovery and informs retry system   
                return;
            }
            //  Use the system's GetStaticPath to get an initial path
            list pts = pathtrimmedstaticpath(startpos, goal, stopshort, gPathWidth + PATHSTATICTOL);
            pathMsg(PATH_MSG_INFO,"Static path, status " + (string)llList2Integer(pts,-1) + ", "+ (string)llGetListLength(pts) + " points.");  // dump list for debug
            integer status = llList2Integer(pts,-1);                // last item is status
            //  Adjust endpos vertically if having trouble finding the static path.
            //  llStaticPath endpoints need to be within about 0.5m of the navmesh.
            //  Not in a separate function to avoid copying the long list of points again
            if (status == PU_FAILURE_INVALID_GOAL)                  // if trouble at the goal
            {   float offset;                                       // endpoint offset when trying to find navmesh
                for (offset = gPathHeight; offset > -gPathHeight; offset -= NAVMESHFINDTOL) // search downward, += one NPC height
                {   pts = pathtrimmedstaticpath(startpos, goal + <0,0,offset>, stopshort, gPathWidth + PATHSTATICTOL);
                    integer wstatus = llList2Integer(pts,-1);       // get working status
                    if (wstatus == 0)                               // success
                    {   pathMsg(PATH_MSG_WARN,"Endpoint search found navmesh at offset " + (string)offset);
                        status = wstatus;                           // use these points and status
                        offset = -999999;                           // force loop exit (no break keyword in LSL)
                    }
                }
            }
            //  Dealt with invalid goal error, if possible
            if (status != 0 || llGetListLength(pts) < 3)            // if static path fail or we're already at destination
            {   //  Bug check - sometimes llGetStaticPath returns PU_FAILURE_UNREACHABLE when the problem is that the current position is inescapable.
                //  This tends to happen very close to vertical objects like railings.
                //  This gets treated as a PU_FAILURE_INVALID_START, which causes a recover cycle to the last good point.
                if (status == PU_FAILURE_UNREACHABLE && gPathprepPrevPos != ZERO_VECTOR)    // if special case for false bad destination bug in llGetStaticPath
                {   //  Can we go backwards to the previous point? If not, it's a bad navmesh point and we must recover.
                    pts = pathtrimmedstaticpath(startpos, gPathprepPrevPos-halfheight, 0.0, gPathWidth + PATHSTATICTOL); // compute path back to previous good point
                    integer statusback = llList2Integer(pts,-1);
                    if (statusback != 0)                                // can't go back to previous point. Happens with flawed navmesh. We are must recover.
                    {   pathMsg(PATH_MSG_WARN, "Bogus static unreach: " + (string)startpos + " to " + (string)(gPathprepPrevPos-halfheight) + " stat " + (string)statusback); 
                        status = PU_FAILURE_INVALID_START;              // change status to bad starting point
                    }           
                }
                if (llListFindList(PATHRECOVERABLES, [status]) >= 0) 
                {   pathMsg(PATH_MSG_WARN,"Recovering from static path error " + (string)status + " at " + (string)startpos);
                    pathmoverecover(gPathprepPathid);
                    return;
                }
                pathdeliversegment([], FALSE, TRUE, gPathprepPathid, status);// report error
                return;
            }
            //  Got path. Do the path prep work.
            pts = [startpos] + llList2List(pts,0,-2);                   // drop status from end of points list
            ////assert(llVecMag(llList2Vector(pts,0) - startpos) < 0.01); // first point of static path must always be where we are
            //  llGetStaticPath produces a list of points which may contain duplicate points or ultra short segments.
            //  These cause trouble later when we need a vector between points, so they are deleted here in initial path cleanup.
            pts = pathclean(pts);                                       // remove dups and ultra short segments
            //  Region crossing works by generating a path that has one point in the next region. As soon as the
            //  NPC crosses the region boundary, move will detect it and report an error. Retry, starting in the
            //  new region, will continue from there.
            pts = pathtrimmedregionbound(pts);                          // trim just past region boundary
            //  llGetStaticPath returns points on the navmesh, which is a rough approximation of the visible surface.
            //  The navmesh may be a meter or two above or below the visible surface. So the Z heights are adjusted
            //  based on vertical llCastRay calls.
            ////pathMsg(PATH_MSG_WARN,"Cleaned: " +  llDumpList2String(pts,","));                       // ***TEMP***
            pts = pathptstowalkable(pts, gPathHeight);                    // project points onto walkable surface
            ////pathMsg(PATH_MSG_INFO,"Walkables");                     // ***TEMP***
            integer len = llGetListLength(pts);                            // update number of points after cleanup
            if (len < 2)
            {   
                pathdeliversegment([], FALSE, TRUE, gPathprepPathid, PATHERRMAZENOPTS);        // empty set of points, no maze, done.
                return;                                             // empty list
            }
            vector startposerr = llList2Vector(pts,0) - startpos;                  // should not change first point, except in Z
            ////assert(llVecMag(<startposerr.x,startposerr.y,0.0>) < 0.01);        // first point must always be where we are
            //  Check that path does not go through a keep-out area.
            //  Going off-region blindly is allowed if the endpoint is off-region.
            //  This is somewhat risky, but the user has to specify an off-region destination to allow an off-region move.
            integer singleregion = pathisinregion(goal);                    // is endpoint off region?
            for (i=0; i<len; i++)                                           // for all points on path
            {   vector pt = llList2Vector(pts,i);                           // point being tested
                if (pathisinregion(pt))                                     // if path is in region
                {   if (singleregion && !pathvaliddest(pt))                 // if cannot go there
                    {
                        pathMsg(PATH_MSG_WARN, "Prohibited point: " + (string)pt);  // can't go there, don't even try
                        pathdeliversegment([],FALSE,TRUE,gPathprepPathid, PATHERRPROHIBITED); // empty set of points, no maze, done.   
                        return;                                         // not allowed
                    }
                }
            }         
            //  We have a valid static path. Send it to the main planner.
            pathMsg(PATH_MSG_INFO,"Path check for obstacles. Segments: " + (string)len); 
            llMessageLinked(LINK_THIS, PATHPLANPREPPED, llList2Json(JSON_OBJECT,
                ["target",gPathprepTarget, "goal", goal, "stopshort", stopshort, "testspacing", testspacing,
                "speed", gPathprepSpeed, "turnspeed", gPathprepTurnspeed,
                "pathid", gPathprepPathid, 
                "refpt", gRefPt, "points", llList2Json(JSON_ARRAY,pts)]),"");
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn);                                       // initialize params
        } else if (num == DEBUG_MSGLEV_BROADCAST)               // set debug message level for this task
        {   debugMsgLevelSet(jsn);
        }
    }
}

