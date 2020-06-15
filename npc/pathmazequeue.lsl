//
//  pathmazequeue.lsl
//
//  Queues requests for the maze solver.
//
//  The maze solver is compute-bound for long periods. Long enough
//  that other events overflow its event queue. So we have to put it in a prim
//  by itself to limit its message intake of irrelevant messages.
//
//  This script goes in the prim with the other path... scripts.
//  The maze solver goes in a prim by itself.
//
//  Animats
//  June, 2020
//
//  ***MORE*** Something to check if we don't have enough time left to do another worst case maze?
//
//
#include "npc/pathbuildutils.lsl"
#include "npc/pathmazedefs.lsl"

//
//  Globals
//
integer gMazePrim = LINK_ALL_OTHERS;                    // we don't know which prim has the maze solver yet
list gMazeQueue = [];                                   // [pathid, jsn, pathid, jsn...]
integer gMazeLastPathid = -1;                           // last task we started
integer gMazeLastSegid = -1;
integer gMazeLastStarttime = -1;                        // time last maze solve started
integer gMazeLongestTime = -1;                          // longest maze solve time

//
//  queuemazesolve -- queue new maze solve request
//
//  If this request is for a different path than any
//  on the list, those requests are stale and can be removed.
//  Stale maze solve requests are from a failed planning cycle
//  that looked ahead. No need to do them.
//
queuemazesolve(integer pathid, string jsn)
{
    integer done = FALSE;
    while (gMazeQueue != [] && !done)                   // get rid of stale requests
    {
        if (llList2Integer(gMazeQueue,0) != pathid)     // if stale request
        {   gMazeQueue = llDeleteSubList(gMazeQueue,0,1); // remove from head of list  
        } else {
            done = TRUE;                                // no more stale requests to remove
        }  
    }
    gMazeQueue += [pathid,jsn];                         // add new maze solve to end of queue
    ////llOwnerSay("Queue: " + llDumpList2String(gMazeQueue,","));  // ***TEMP***
    startnextmazesolve();                               // start the first solve on the queue if idle
}

//
//  mazesolvecancel -- do not start maze solve, just report an error
//
mazesolvecancel(vector refpt, integer pathid, integer segid, integer status)
{
    //  Create a dummy maze solve result and send it to path execution just to transmit the status.
    //  We also get this message here and it advances the queue.
    llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT,
    ["reply", "mazesolve", "pathid", pathid, "segmentid", segid, "status", status, 
     "pos", ZERO_VECTOR, "rot", ZERO_ROTATION, "cellsize", 0.0,
     "refpt",refpt, "points",llList2Json(JSON_ARRAY,[])]),"");
}
//
//  startnextmazesolve -- start the next maze solve
//
startnextmazesolve() 
{   
    if (gMazeQueue == []) { return; }                   // nothing queued 
    if (gMazeLastPathid >= 0) { return; }               // maze solver is busy
    string jsn = llList2String(gMazeQueue,1);           // get first JSON string
    gMazeQueue = llDeleteSubList(gMazeQueue,0,1);       // remove from head of list
    gMazeLastPathid = (integer) llJsonGetValue(jsn, ["pathid"]);    // what maze solver is doing now
    gMazeLastSegid = (integer)llJsonGetValue(jsn,["segmentid"]);
    integer starttime = (integer)llJsonGetValue(jsn,["starttime"]); // time entire path solve started
    gMazeLastStarttime = llGetUnixTime();               // timestamp
    integer timeleft = PATHCALLSTALLTIME - (gMazeLastStarttime - starttime);  // time left for this maze solve
    if (timeleft < MAZETIMELIMIT)                       // if not enough time left for maze solve
    {   vector refpt = (vector)llJsonGetValue(jsn,["refpt"]);       // region corner to which points are relative
        pathMsg(PATH_MSG_WARN, "Not enough time left for another maze solve: " + (string)timeleft + "s.");
        mazesolvecancel(refpt, gMazeLastPathid,gMazeLastSegid, PATHERRNOMAZETIME);   // fails
        return;
    }
    pathMsg(PATH_MSG_WARN, "Sending to maze solver: " + jsn);  
    llMessageLinked(gMazePrim, MAZESOLVEREQUEST, jsn, "");  // send to maze solver
}
//
//  mazesolverdone -- maze solver has finished a maze.
//
//  pathassemble also receives and uses this message.
//
//  We expect this completion to match the request we last sent.
//  Maze queue to maze solver is strictly synchronous.
//
mazesolverdone(integer pathid, integer segid, integer status)
{
    //  
    if (pathid != gMazeLastPathid || segid != gMazeLastSegid)       // if tasks are out of sync, debug
    {
        pathMsg(PATH_MSG_WARN, "Maze solver finished wrong maze: expected pathid " +
            (string)gMazeLastPathid + " segid " + (string)gMazeLastSegid + " but rcvd pathid " + (string)pathid + " segid " + (string)segid);
        return;
    }
    ////assert(pathid == gMazeLastPathid);                              // maze solver working on wrong maze
    ////assert(segid == gMazeLastSegid);                                // should not happen
    gMazeLastPathid = -1;                                           // idle
    gMazeLastSegid = -1;
    integer elapsed = llGetUnixTime() - gMazeLastStarttime;         // elapsed time, secs 
    if (elapsed > gMazeLongestTime)                                 // if new longest time
    {   gMazeLongestTime = elapsed; }
    if (status != 0)                                                // if this maze solve failed
    {   pathMsg(PATH_MSG_WARN,"Maze solve failed, status " + (string) status + " after " + (string) elapsed + "s, slowest " + (string)gMazeLongestTime + "s");
        integer done = FALSE;
        while (gMazeQueue != [] && !done)                           // get rid of other requests on this pathid
        {
            if (llList2Integer(gMazeQueue,0) == pathid)             // if stale request
            {   gMazeQueue = llDeleteSubList(gMazeQueue,0,1);       // remove from head of list
                pathMsg(PATH_MSG_WARN, "Maze solve failed, dropping later request for same pathid " + (string)pathid);  
            } else {
                done = TRUE;                                        // no more stale requests to remove
            }  
        }
    }
    startnextmazesolve();                                           // start the maze solver                
}   


//
//  The main program of the maze queue task
//
default
{

    state_entry()
    {   pathinitutils(); }                              // library init
   
    link_message( integer sender_num, integer num, string jsn, key id )
    {   if (num == MAZEQUEUEREQUEST)
        {   //  Solve maze
            //
            //  Format:
            //  { "request" : "mazesolve",  "pathid" : INTEGER, "segmentid": INTEGER,
            //      "regioncorner" : VECTOR, "pos": VECTOR, "rot" : QUATERNION, "cellsize": FLOAT, "probespacing" : FLOAT, 
            //      "sizex", INTEGER, "sizey", INTEGER, 
            //      "startx" : INTEGER, "starty" : INTEGER, "endx" : INTEGER, "endy" : INTEGER, "starttime": "INTEGER" }
            //      
            //  "regioncorner", "pos" and "rot" identify the coordinates of the CENTER of the (0,0) cell of the maze grid.
            //  Ray casts are calculated accordingly.
            //  "cellsize" is the edge length of each square cell.
            //  "probespacing" is the spacing between llCastRay probes.
            //  "height" and "radius" define the avatar's capsule. 
            //  
            pathMsg(PATH_MSG_NOTE,"Request to maze queue: " + jsn);            // verbose mode
            assert(gPathWidth > 0);                                 // must be initialized properly
            integer status = 0;                                     // so far, so good
            string requesttype = llJsonGetValue(jsn,["request"]);   // request type
            if (requesttype != "mazesolve") { return; }              // ignore, not our msg
            integer pathid = (integer) llJsonGetValue(jsn, ["pathid"]); 
            queuemazesolve(pathid, jsn);                // queue this request
        } 
        else if (num == MAZESOLVERREPLY)                // just snooping on maze solves to see if we should send the next request
        {   integer pathid = (integer) llJsonGetValue(jsn, ["pathid"]); 
            integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
            integer status = (integer) llJsonGetValue(jsn, ["status"]);
            string prim = llJsonGetValue(jsn,["prim"]); // prim from which message was sent
            if (prim != JSON_INVALID)                   // if present, really from maze solver
            {   gMazePrim = (integer)prim;              // Get maze solver's prim number for later messaging
                mazesolverdone(pathid, segmentid, status);  // maze solver is done, can send another request  
            }
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn);                        // initialize globals (width, height, etc.)
        } else if (num == DEBUG_MSGLEV_BROADCAST)       // set debug message level for this task
        {   debugMsgLevelSet(jsn);
        }
    }
}

