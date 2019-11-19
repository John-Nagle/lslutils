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
////integer gPathChartype;
key gPathprepTarget;
////float gPathWidth;
////float gPathHeight;
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
                "width", gPathWidth, "height", gPathHeight, "chartype", gPathChartype, "msglev", gPathMsgLevel,
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
            //  Starting position and goal position must be on a walkable surface, not at character midpoint. 
            //  We just go down slightly less than half the character height.
            //  We're assuming a box around the character.      
            startpos = llGetPos();                                      // startpos is where we are now
            vector startscale = llGetScale();
            startpos.z = (startpos.z - startscale.z*0.45);              // approximate ground level for start point
            vector goal = (vector)llJsonGetValue(jsn,["goal"]);     // get goal point
            gPathprepTarget = (key)llJsonGetValue(jsn,["target"]);  // get target if pursue
            ////gPathWidth = (float)llJsonGetValue(jsn,["width"]);
            ////gPathHeight = (float)llJsonGetValue(jsn,["height"]);
            float stopshort = (float)llJsonGetValue(jsn,["stopshort"]);
            ////gPathChartype = (integer)llJsonGetValue(jsn,["chartype"]); // usually CHARACTER_TYPE_A, humanoid
            float testspacing = (float)llJsonGetValue(jsn,["testspacing"]);
            gPathprepPathid = (integer)llJsonGetValue(jsn,["pathid"]);
            gPathMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);
            gPathprepSpeed = (float)llJsonGetValue(jsn,["speed"]);
            gPathprepTurnspeed = (float)llJsonGetValue(jsn,["turnspeed"]);
            pathMsg(PATH_MSG_INFO,"Path request: " + jsn); 
            jsn = "";                                               // Release string. We are that tight on space.
            //  Call the planner
            pathMsg(PATH_MSG_INFO,"Pathid " + (string)gPathprepPathid + " prepping."); 
            //  Start a new planning cycle
            //  Quick sanity check - are we in a legit place?            
            vector pos = llGetPos();                                // we are here
            vector fullheight = <0,0,gPathHeight>;              // add this for casts from middle of character
            vector halfheight = fullheight*0.5;   
            vector p = pos-halfheight;                              // position on ground
#ifdef OBSOLETE
            vector mazedepthmargin = <0,0,MAZEBELOWGNDTOL>;         // subtract this for bottom end of ray cast
            if (obstacleraycastvert(p+fullheight,p-mazedepthmargin) < 0)  // use exactly the same test as in pathmove           
#endif // OBSOLETE
            integer good = FALSE;                                   // not yet a good start point
            for (i=0; i<llGetListLength(TRIALOFFSETS); i++)         // try coming from all around the start point
            {   if (!good)
                {   vector refpos = p + llList2Vector(TRIALOFFSETS,i)*gPathWidth; // test at this offset
                    good = pathcheckcelloccupied(refpos, p, TRUE, FALSE) >= 0.0;
                }
            }
            if (!good)
            {   pathMsg(PATH_MSG_WARN, "Start location is not clear: " + (string)startpos);           // not good, but we can recover
                //  Ask the move task to attempt recovery. This will move to a better place and return a status to the retry system.
                pathmoverecover(gPathprepPathid);                       // attempts recovery and informs retry system   
                return;
            }
            //  Use the system's GetStaticPath to get an initial path
            list pts = pathtrimmedstaticpath(startpos, goal, stopshort, gPathWidth + PATHSTATICTOL);
            integer len = llGetListLength(pts);                          
            ////pathMsg(PATH_MSG_INFO,"Static path, status " + (string)llList2Integer(gPts,-1) + ", "+ (string)llGetListLength(gPts) + 
            ////    " pts: " + llDumpList2String(pts,","));             // dump list for debug
            pathMsg(PATH_MSG_INFO,"Static path, status " + (string)llList2Integer(pts,-1) + ", "+ (string)len + " points.");  // dump list for debug
            integer status = llList2Integer(pts,-1);                // last item is status
            if (status != 0 || len < 3)            // if static path fail or we're already at destination
            {   if (llListFindList(PATHRECOVERABLES, [status]) >= 0) 
                {   pathMsg(PATH_MSG_WARN,"Recovering from static path error " + (string)status + " at " + (string)startpos);
                    pathmoverecover(gPathprepPathid);
                    return;
                }
                pathdeliversegment([], FALSE, TRUE, gPathprepPathid, status);// report error
                return;
            }
            //  Got path. Do the path prep work.
            pts = [startpos] + llList2List(pts,0,-2);                   // drop status from end of points list
#ifdef OBSOLETE
            ////pathMsg(PATH_MSG_INFO,"Static planned");                // ***TEMP***
            vector startposerr0 = llList2Vector(pts,0) - startpos;                  // should not change first point, except in Z
            if (llVecMag(<startposerr0.x,startposerr0.y,0.0>) > 0.01)            // ***TEMP***
            {   pathMsg(PATH_MSG_ERROR, "First point wrong 1, was: " + (string)llList2Vector(pts,0) + " should be " + (string)startpos); }
#endif // OBSOLETE

            ////assert(llVecMag(llList2Vector(pts,0) - startpos) < 0.01); // first point of static path must always be where we are
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
            vector startposerr = llList2Vector(pts,0) - startpos;                  // should not change first point, except in Z
#ifdef OBSOLETE
            if (llVecMag(<startposerr.x,startposerr.y,0.0>) > 0.01)            // ***TEMP***
            {   pathMsg(PATH_MSG_ERROR, "First point wrong 2, was: " + (string)llList2Vector(pts,0) + " should be " + (string)startpos); }
#endif // OBSOLETE
            ////assert(llVecMag(<startposerr.x,startposerr.y,0.0>) < 0.01);        // first point must always be where we are
            //  Check that path does not go through a keep-out area.
            for (i=0; i<len; i++)
            {   vector pt = llList2Vector(pts,i);                   // point being tested
                if (!pathvaliddest(pt))                             // if cannot go there
                {
                    pathMsg(PATH_MSG_WARN, "Prohibited point: " + (string)pt);  // can't go there, don't even try
                    pathdeliversegment([],FALSE,TRUE,gPathprepPathid, PATHERRPROHIBITED); // empty set of points, no maze, done.   
                    return;                                         // not allowed
                }
            }         
            //  We have a valid static path. Send it to the main planner.
            pathMsg(PATH_MSG_INFO,"Path check for obstacles. Segments: " + (string)len); 
            llMessageLinked(LINK_THIS, PATHPLANPREPPED, llList2Json(JSON_OBJECT,
                ["target",gPathprepTarget, "goal", goal, "stopshort", stopshort, "testspacing", testspacing,
                "speed", gPathprepSpeed, "turnspeed", gPathprepTurnspeed,
                "pathid", gPathprepPathid, "points", llList2Json(JSON_ARRAY,pts)]),"");
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                                      // initialize params
  
    }
}

