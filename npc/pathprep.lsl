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
#include "npc/mazesolvercall.lsl"
#include "npc/pathmovecall.lsl"                             // for recover
//
//  Constants
//
float MINSEGMENTLENGTH = 0.025; /// 0.10;                   // minimum path segment length (m)

#define PATHPLANMINMEM  3000                                // less than this, quit early and retry
list TRIALOFFSETS = [<-1,0,0>,<1,0,0>,<0,1,0>,<0,-1,0>]; // try coming from any of these directions for start point validation
//  Globals
//
float gPathprepSpeed;
float gPathprepTurnspeed;
integer gPathprepChartype;
key gPathprepTarget;
float gPathprepWidth;
float gPathprepHeight;
integer gPathprepPathid;

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
                "width", gPathprepWidth, "height", gPathprepHeight, "chartype", gPathprepChartype, "msglev", gPathMsgLevel,
                "points"];                                  // just add points at the end
    assert(path == []);
    assert(!ismaze);                                        // not during prep
    assert(isdone);                                         // only for the done case with an error
    llMessageLinked(LINK_THIS,MAZEPATHREPLY,
            llList2Json(JSON_OBJECT, ["segmentid", 0] + fixedreplypart + [llList2Json(JSON_ARRAY,[ZERO_VECTOR])]),"");    // send one ZERO_VECTOR segment as an EOF.
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
    link_message(integer status, integer num, string jsonstr, key id)
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
            //  Starting position and goal position must be on a walkable surface, not at character midpoint. 
            //  We just go down slightly less than half the character height.
            //  We're assuming a box around the character.      
            startpos = llGetPos();                                      // startpos is where we are now
            vector startscale = llGetScale();
            startpos.z = (startpos.z - startscale.z*0.45);              // approximate ground level for start point
            vector goal = (vector)llJsonGetValue(jsonstr,["goal"]);     // get goal point
            gPathprepTarget = (key)llJsonGetValue(jsonstr,["target"]);  // get target if pursue
            gPathprepWidth = (float)llJsonGetValue(jsonstr,["width"]);
            gPathprepHeight = (float)llJsonGetValue(jsonstr,["height"]);
            float stopshort = (float)llJsonGetValue(jsonstr,["stopshort"]);
            gPathprepChartype = (integer)llJsonGetValue(jsonstr,["chartype"]); // usually CHARACTER_TYPE_A, humanoid
            float testspacing = (float)llJsonGetValue(jsonstr,["testspacing"]);
            gPathprepPathid = (integer)llJsonGetValue(jsonstr,["pathid"]);
            gPathMsgLevel = (integer)llJsonGetValue(jsonstr,["msglev"]);
            gPathprepSpeed = (float)llJsonGetValue(jsonstr,["speed"]);
            gPathprepTurnspeed = (float)llJsonGetValue(jsonstr,["turnspeed"]);
            pathMsg(PATH_MSG_INFO,"Path request: " + jsonstr); 
            jsonstr = "";                                               // Release string. We are that tight on space.
            //  Call the planner
            pathMsg(PATH_MSG_INFO,"Pathid " + (string)gPathprepPathid + " prepping."); 
            //  Start a new planning cycle
            //  Quick sanity check - are we in a legit place?            
            vector pos = llGetPos();                                // we are here
            vector fullheight = <0,0,gPathprepHeight>;              // add this for casts from middle of character
            vector halfheight = fullheight*0.5;   
            vector p = pos-halfheight;                              // position on ground
#ifdef OBSOLETE
            vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
            if (obstacleraycastvert(p+fullheight,p-mazedepthmargin) < 0)  // use exactly the same test as in pathmove           
#endif // OBSOLETE
            integer good = FALSE;                                   // not yet a good start point
            for (i=0; i<llGetListLength(TRIALOFFSETS); i++)         // try coming from all around the start point
            {   if (!good)
                {   vector refpos = p + llList2Vector(TRIALOFFSETS,i)*gPathprepWidth; // test at this offset
                    good = pathcheckcelloccupied(refpos, p, gPathprepWidth, gPathprepHeight, gPathprepChartype, TRUE, FALSE) >= 0.0;
                }
            }
            if (!good)
            {   pathMsg(PATH_MSG_WARN, "Start location is not clear: " + (string)startpos);           // not good, but we can recover
                //  Ask the move task to attempt recovery. This will move to a better place and return a status to the retry system.
                pathmoverecover(gPathprepPathid);                       // attempts recovery and informs retry system   
                return;
            }
            //  Use the system's GetStaticPath to get an initial path
            list pts = pathtrimmedstaticpath(startpos, goal, stopshort, gPathprepWidth + PATHSTATICTOL, gPathprepChartype);
            integer len = llGetListLength(pts);                          
            ////pathMsg(PATH_MSG_INFO,"Static path, status " + (string)llList2Integer(gPts,-1) + ", "+ (string)llGetListLength(gPts) + 
            ////    " pts: " + llDumpList2String(pts,","));             // dump list for debug
            pathMsg(PATH_MSG_INFO,"Static path, status " + (string)llList2Integer(pts,-1) + ", "+ (string)len + " points.");  // dump list for debug
            integer status = llList2Integer(pts,-1);                // last item is status
            if (status != 0 || len < 3)            // if static path fail or we're already at destination
            {   pathdeliversegment([], FALSE, TRUE, gPathprepPathid, status);// report error
                return;
            }
            //  Got path. Do the path prep work.
            pts = llList2List(pts,0,-2);                          // drop status from end of points list
            ////pathMsg(PATH_MSG_INFO,"Static planned");                // ***TEMP***
            pts = pathclean(pts);                                 // remove dups and ultra short segments
            ////pathMsg(PATH_MSG_INFO,"Cleaned");                       // ***TEMP***
            pts = pathptstowalkable(pts, gPathHeight);                    // project points onto walkable surface
            ////pathMsg(PATH_MSG_INFO,"Walkables");                     // ***TEMP***
            len = llGetListLength(pts);                            // update number of points after cleanup
            if (len < 2)
            {   
                pathdeliversegment([], FALSE, TRUE, gPathprepPathid, MAZESTATUSNOPTS);        // empty set of points, no maze, done.
                return;                                             // empty list
            }
            //  We have a valid static path. Send it to the main planner.
            pathMsg(PATH_MSG_INFO,"Path check for obstacles. Segments: " + (string)len); 
            llMessageLinked(LINK_THIS, PATHPLANPREPPED, llList2Json(JSON_OBJECT,
                ["target",gPathprepTarget, "goal", goal, "stopshort", stopshort, "width", gPathprepWidth, "height", gPathprepHeight, 
                "chartype", gPathprepChartype, "testspacing", testspacing,
                "speed", gPathprepSpeed, "turnspeed", gPathprepTurnspeed,
                "pathid", gPathprepPathid, "msglev", gPathMsgLevel, "points", llList2Json(JSON_ARRAY,pts)]),"");
        }   
    }
}

