//
//  mazesolver.lsl
//
//  Finds reasonable path through a grid of squares with obstacles.
//
//  Animats
//  June, 2019
//
//  The algorithm is based on one from Wikipedia:
//
//  https://en.wikipedia.org/wiki/Maze_solving_algorithm//Maze-routing_algorithm
//
//  That doesn't work as published; see the Wikipedia talk page.
//
//  This is guaranteed to produce a path if one exists, but it may be non-optimal.
//  All paths in this module are rectangular - horizontal and vertical only.
//
//  The basic approach is to head for the goal, and if there's an obstacle, follow
//  the edge of the obstacle until there's a good route to the goal.
//
//  Paths from this need tightening up afterwards.
//  "mazeoptimizeroute" does the first stage of that.
//  A polishing pass will be required outside this module so that the
//  paths are not so rectangular. 
//
//  Memory consumption is a problem. This code needs to be in a script of its own,
//  communicating by messages, to avoid stack/heap collisions in this 64K world.
//
//  The data for each cell is:
//  - barrier - 1 bit, obstacle present
//  - examined - 1 bit, obstacle presence tested
//
//  These are packed into 2 bits, which are packed 16 per 32 bit word
//  (LSL being a 32-bit system), which are stored in a LSL list.
//  Timing tests indicate that the cost of updating an LSL list is constant up to size 128;
//  then it starts to increase linearly.  So a single list is good enough for anything up
//  to 45x45 cells. 
//
//  TODO:
//  1. Add checking for getting close to space limits, and return failure before a stack/heap collision. [DONE]
//  2. Add backup counter to detect runaways now that collinear point optimization is in. 
//
//
#include "npc/pathbuildutils.lsl"
#include "npc/mazedefs.lsl"

//  Constants
//
#define MAZEBARRIER (0x1)                                   // must be low bit
#define MAZEEXAMINED (0x2)

#define MAZEINVALIDPT (0xffffffff)                          // invalid point value in old point storage

#define MAZEMINMEM (2000)                                   // make sure we have this much memory left
//   Wall follow sides
#define MAZEWALLONLEFT  1
#define MAZEWALLONRIGHT (-1)
list EDGEFOLLOWDIRX = [1,0,-1,0];
list EDGEFOLLOWDIRY = [0, 1, 0, -1];

list MAZEEDGEFOLLOWDXTAB = [1,0,-1,0];
list MAZEEDGEFOLLOWDYTAB = [0,1,0,-1];
#define MAZEEDGEFOLLOWDX(n) llList2Integer(MAZEEDGEFOLLOWDXTAB,(n))
#define MAZEEDGEFOLLOWDY(n) llList2Integer(MAZEEDGEFOLLOWDYTAB,(n))

#define MAZEPRINTVERBOSE(s) { pathMsg(PATH_MSG_DEBUG,(s)); }


//
//   mazemd -- rectangular "Manhattan" distance
//
integer mazemd(integer p0x, integer p0y, integer p1x, integer p1y)
{   return llAbs(p1x-p0x) + llAbs(p1y-p0y); }
    
//
//   mazeclipto1 -- clip to range -1, 1
//
integer mazeclipto1(integer n)
{
    if (n > 0) { return(1); }
    if (n < 0) { return(-1); }
    return(0);
}
//
//   mazeinline  -- true if points are in a line
//
integer mazeinline(integer x0, integer y0,integer x1,integer y1, integer x2, integer y2) 
{    return ((x0 == x1 && x1 == x2) ||
       (y0 == y1 && y1 == y2));
}
       
//
//   mazepointssame  -- true if points are identical
//
integer mazepointssame(integer x0, integer y0, integer x1, integer y1) 
{   return(x0 == x1 && y0 == y1); }
//
//   listreplacelist -- a builtin in LSL
//
list listreplacelist(list src, list dst, integer start, integer end) 
{   assert(start >= 0);                          // no funny end-relative stuff
    assert(end >= 0);
    return(llListReplaceList(src, dst, start, end));
    ////return src[0:start] + dst + src[end+1:]     // LSL compatibility
}
//
//
//   Mazegraph
//
//   Globals for LSL
//
//  All these are initialized at the beginning of mazesolve.
//
list gMazePath = [];                // the path being generated
list gMazeCells = [];               // maze cell bits, see mazecellget
integer gMazeX;                     // current working point
integer gMazeY;                     // current working point
float   gMazeZ;                     // current working point, world coords
integer gMazeMdbest;                // best point found
integer gMazeXsize;                 // maze dimensions
integer gMazeYsize;
integer gMazeStartX;                // start position 
integer gMazeStartY;
float   gMazeStartZ;                // Z of start position, world coords          
integer gMazeEndX;                  // end position 
integer gMazeEndY; 
float   gMazeEndZ;                  // end Z position
integer gMazeStatus;                // status code - MAZESTATUS...
vector  gP0;                        // beginning of maze, for checking only
vector  gP1;                        // end of maze, for checking only
       

//
//   Maze graph functions
//

//
//   Maze cell storage - 2 bits per cell from 2D maze array
//
integer mazecellget(integer x, integer y)   
{
    assert(x >= 0 && x < gMazeXsize);           // subscript check
    assert(y >= 0 && y < gMazeYsize);
    integer cellix = y*gMazeYsize + x;                   // index into cells
    integer listix = (integer)(cellix / 16);
    integer bitix = (cellix % 16) * 2;
    return ((llList2Integer(gMazeCells,listix) >> bitix) & 0x3);  // 2 bits only
}

//
//  mazecellset -- store into 2D maze array
//    
mazecellset(integer x, integer y, integer newval) 
{   assert(x >= 0 && x < gMazeXsize);           // subscript check
    assert(y >= 0 && y < gMazeYsize);
    assert(newval <= 0x3);                      // only 2 bits
    integer cellix = y*gMazeYsize + x;          // index into cells
    integer listix = (integer)(cellix / 16);          // word index
    integer bitix = (cellix % 16) * 2;          // bit index within word
    integer w = llList2Integer(gMazeCells,listix);             // word to update
    w = (w & (~(0x3<<bitix)))| (newval<<bitix); // insert into word
    gMazeCells = llListReplaceList(gMazeCells,[w],listix, listix); // replace word
    ////gMazeCells[listix] = w;                     // insert word
}

//
//  mazesolve  -- find a path through a maze
//         
list mazesolve(integer xsize, integer ysize, integer startx, integer starty, float startz, integer endx, integer endy, float endz)
{   
    gMazeStatus = 0;                        // OK so far
    gMazeXsize = xsize;                     // set size of map
    gMazeYsize = ysize;
    gMazeCells = [];
    while (llGetListLength(gMazeCells) < (xsize*ysize+15)/16)        // allocate cell list
    {    gMazeCells = gMazeCells + [0]; }   // fill list with zeroes 
    gMazeX = startx;                        // start
    gMazeY = starty;
    gMazeZ = startz;                        // world coords
    gMazeStartX = startx;                   // start
    gMazeStartY = starty;
    gMazeStartZ = startz;
    gMazeEndX = endx;                       // destination
    gMazeEndY = endy;
    gMazeEndZ = endz;
    gMazeMdbest = gMazeXsize+gMazeYsize+1;  // best dist to target init
    gMazePath = [];                         // accumulated path
    mazeaddtopath();                        // add initial point
#ifdef MARKERS                                              // debug markers which appear in world
    {   //  Show start and end points of maze as green discs.
        vector p = mazecelltopoint(startx, starty);
        p.z = startz;
        placesegmentmarker(MARKERLINE, p, p+<0.01,0,0>, gMazeWidth, TRANSGREEN, 0.20);// place a temporary line on the ground in-world.
        p = mazecelltopoint(endx, endy);
        p.z = endz;
        placesegmentmarker(MARKERLINE, p, p+<0.01,0,0>, gMazeWidth, TRANSGREEN, 0.20);// place a temporary line on the ground in-world.
    }
#endif // MARKERS 
    //   Outer loop - shortcuts || wall following
    MAZEPRINTVERBOSE("Start maze solve.");
    while (gMazeX != gMazeEndX || gMazeY != gMazeEndY)  // while not at dest
    {   pathMsg(PATH_MSG_INFO,"Maze solve at (" + (string)gMazeX + "," + (string)gMazeY + "," + (string)gMazeZ + ")");
        if (gMazeStatus)                                    // if something went wrong
        {   MAZEPRINTVERBOSE("Maze solver failed: status " + (string)gMazeStatus + " at (" + (string)gMazeX + "," + (string)gMazeY + ")");
            gMazeCells = [];                                // release memory
            gMazePath = [];
            return([]);
        }
        //  Overlong list check
        if (llGetListLength(gMazePath) > gMazeXsize*gMazeYsize*4) 
        {   gMazeCells = [];
            gMazePath = [];
            return([]);                    // we are in an undetected loop
        }
        integer shortcutavail = mazeexistsproductivepath(gMazeX, gMazeY, gMazeZ);     // if a shortcut is available
        if (gMazeStatus)                                    // if something went wrong checking cells
        {   MAZEPRINTVERBOSE("Maze solver failed: status " + (string)gMazeStatus + " at (" + (string)gMazeX + "," + (string)gMazeY + ")");
            gMazeCells = [];                                // release memory
            gMazePath = [];
            return([]);
        }
        if (shortcutavail)                                                          // if a shortcut is available
        {   DEBUGPRINT1("Maze productive path at (" + (string)gMazeX + "," + (string)gMazeY + ")");
            mazetakeproductivepath();       // use it
            gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY);
            ////////gMazeMdbest = gMazeMdbest -1 
            assert(gMazeMdbest >= 0);
        } else {
            integer direction = mazepickside();                                 // direction for following right wall
            ////integer sidelr = MAZEWALLONRIGHT;                                   // always returns dir for following right wall
            //   Inner loop - wall following
            integer followstartx = gMazeX;
            integer followstarty = gMazeY;
            float followstartz = gMazeZ;
            integer followstartdir = direction;
            MAZEPRINTVERBOSE("Starting wall follow at " + (string)followstartx + "," + (string)followstarty + ",  direction " + (string)direction + ", mdist = " + (string)gMazeMdbest);
            integer livea = TRUE;                                               // more to do on path a
            integer liveb = TRUE;                                               // more to do on path b
            integer founduseful = FALSE;                                        // found a useful path
            ///integer initialdira = -1;
            ///integer initialdirb = -1;
            list patha = [followstartx, followstarty, followstartz, followstartdir];          // start conditions, follow one side
            list pathb = [followstartx, followstarty, followstartz, (followstartdir + 2) % 4];    // the other way
            integer xa = -1;                                                    // x,y,z,dir for follower A
            integer ya = -1;
            float za = -1.0;
            integer dira = -1;
            integer xb = -1;                                                    // x,y,z,dir for follower B
            integer yb = -1;
            float zb = -1.0;
            integer dirb = -1;
            while (gMazeStatus == 0 && (!founduseful) && (livea || liveb))      // if more following required
            {   
                // Advance each path one cell
                if (livea)                                                      // if path A still live
                {   patha = mazewallfollow(patha, MAZEWALLONRIGHT);                      // follow one wall
                    DEBUGPRINT1("Path A: " + llDumpList2String(patha,","));
                    DEBUGPRINT1("Path A in: " + mazerouteasstring(llListReplaceList(patha, [], -4,-1))); // ***TEMP***
                    xa = llList2Integer(patha,-4);                       // get X and Y from path list
                    ya = llList2Integer(patha,-3);
                    za = llList2Float(patha, -2);
                    dira = llList2Integer(patha,-1);                     // direction
                    if (gMazeStatus == 0 && (xa == gMazeEndX) && (ya == gMazeEndY))    // reached final goal
                    {   list goodpath = gMazePath + llListReplaceList(patha, [], -4,-1);   // get good path
                        gMazeCells = [];
                        gMazePath = [];
                        DEBUGPRINT1("Path A reached goal: " + mazerouteasstring(llListReplaceList(patha, [], -4,-1)));
                        return(goodpath);
                    }
                    if ((xa != followstartx || ya != followstarty) && mazeexistusefulpath(xa,ya,za))  // if useful shortcut, time to stop wall following
                    {   list goodpath = gMazePath + llListReplaceList(patha, [], -4,-1);   // get good path
                        gMazePath = goodpath;                                   // add to accumulated path
                        gMazeX = xa;                                            // update position
                        gMazeY = ya;
                        gMazeZ = za;
                        founduseful = TRUE;                                     // force exit
                        DEBUGPRINT1("Path A useful: " + mazerouteasstring(llListReplaceList(patha, [], -4,-1)));
                    }
                    if (xa == followstartx && ya == followstarty && dira == followstartdir) 
                    {   DEBUGPRINT1("Path A stuck."); livea = FALSE; }          // in a loop wall following, stuck 
                    if (liveb && xa == xb && ya == yb && dira == (dirb + 2) % 4)// followers have met head on
                    {   DEBUGPRINT1("Path A hit path B"); livea = FALSE; liveb = FALSE; }      // force quit
                    if (mazecellbadz(xa, ya, za)) { livea = FALSE; }            // check for unreasonable Z value near goal

                }
                if (liveb && !founduseful)                                      // if path B still live and no solution found
                {   pathb = mazewallfollow(pathb, -MAZEWALLONRIGHT);                     // follow other wall
                    DEBUGPRINT1("Path B: " + llDumpList2String(pathb,","));
                    DEBUGPRINT1("Path B in: " + mazerouteasstring(llListReplaceList(pathb, [], -4,-1))); // ***TEMP***
                    xb = llList2Integer(pathb,-4);                       // get X and Y from path list
                    yb = llList2Integer(pathb,-3);
                    zb = llList2Float(pathb,-2);
                    dirb = llList2Integer(pathb,-1);                     // direction
                    if (gMazeStatus == 0 && (xb == gMazeEndX) && (yb == gMazeEndY))    // reached final goal
                    {   list goodpath = gMazePath + llListReplaceList(pathb, [], -4,-1);   // get good path
                        gMazeCells = [];
                        gMazePath = [];
                        DEBUGPRINT1("Path B reached goal: " + mazerouteasstring(llListReplaceList(pathb, [], -4,-1)));
                        return(goodpath);
                    }
                    if ((xb != followstartx || yb != followstarty) && mazeexistusefulpath(xb,yb,zb))    // if useful shortcut, time to stop wall following
                    {   list goodpath = gMazePath + llListReplaceList(pathb, [], -4,-1);   // get good path
                        gMazePath = goodpath;                                   // add to accmulated path
                        gMazeX = xb;                                             // update position
                        gMazeY = yb;
                        gMazeZ = zb;
                        founduseful = TRUE;                                     // force exit
                        DEBUGPRINT1("Path B useful: " + mazerouteasstring(llListReplaceList(pathb, [], -4,-1)));
                    }
                    if (xb == followstartx && yb == followstarty && dirb == (followstartdir + 2) % 4)
                    {   DEBUGPRINT1("Path B stuck"); liveb = FALSE; } // in a loop wall following, stuck
                    if (xa == followstartx && ya == followstarty && dira == followstartdir) 
                    {   DEBUGPRINT1("Path A stuck."); livea = FALSE; }          // in a loop wall following, stuck ***MAY FAIL - CHECK***
                    if (livea && xa == xb && ya == yb && dira == (dirb + 2) % 4) // followers have met head on
                    {   DEBUGPRINT1("Path A hit path B"); livea = FALSE; liveb = FALSE; }      // force quit
                    if (mazecellbadz(xb, yb, zb)) { liveb = FALSE; }             // check for unreasonable Z value near goal

                }           
                //  Termination conditions
                //  Consider adding check for paths collided from opposite directions. This is just a speedup, though.
            }
            if (!founduseful)                                                       // stopped following, but no result
            {   gMazePath = [];                                                     // failed, release memory and return
                gMazeCells = [];
                pathMsg(PATH_MSG_WARN,"No maze solution. Status: " + (string)gMazeStatus);
                return([]);                                                         // no path possible
            }

            DEBUGPRINT1("Finished wall following at (" + (string)gMazeX + "," + (string)gMazeY + ")");
        }
    }
    MAZEPRINTVERBOSE("Solved maze");
    list path = gMazePath;
    gMazeCells = [];                        // release memory, we need it
    gMazePath = [];
    return(path); 
}

//
//  mazeaddtopath -- add current position to path
//
//  Optimizes out most collinear points. This is just to reduce storage. We're tight on memory here.
//                    
mazeaddtopath() 
{   
    gMazePath = mazeaddpttolist(gMazePath, gMazeX, gMazeY, gMazeZ);        // use common fn
}

//
//  mazeaddpttolist -- add a point to a path. Returns list.
//
//  Collinear and duplicate points are removed to save memory.
//  A final cleanup takes place later.
//
//  No use of global variables, to allow use on multiple paths in parallel.
//  Path is one integer per x,y format.
//
list mazeaddpttolist(list path, integer x, integer y, float z)
{
    DEBUGPRINT1("(" + (string)x + "," + (string)y + "," + (string)z + ")");
    //  Memory check
    if (pathneedmem(MAZEMINMEM))
    {   gMazeStatus = MAZESTATUSNOMEM; }            // out of memory, will abort
    //  Short list check
    integer val = mazepathconstruct(x, y, z, gMazePos.z);    // current point as one integer
    integer length = llGetListLength(path);
    if (length > 0) 
    {   if (llList2Integer(path,-1) == val) { DEBUGPRINT1("Dup pt."); return(path); }}   // new point is dup, ignore.
    if (length >= 3)                            // if at least 3 points
    {   integer prev0 = llList2Integer(path,-2);
        integer prev1 = llList2Integer(path,-1);
        //  Check for collinear points.
        if (mazeinline(mazepathx(prev0), mazepathy(prev0),
                mazepathx(prev1), mazepathy(prev1),
                x, y))  
        {   DEBUGPRINT1("Collinear pt"); return(llListReplaceList(path,[val],-1,-1)); } // new point replaces prev point 
    } 
    //  No optimizations, just add new point
    {   return(path + [val]); }                 // no optimization
}

//
//  mazecellbadz -- maze cell is close to end, but too far in Z to be valid.
//  
//  This catches problems like being under stairs and such.
//  It's more of a back up measure; it doesn't create a full 3D maze solver.
//
#define MAZERMAXSLOPE 2.0                                       // largest allowed slope
integer mazecellbadz(integer x, integer y, float z)
{   float md = gMazeCellSize*(float)mazemd(x, y, gMazeEndX, gMazeEndY);   // Manhattan distance to end
    float zdiff = llFabs(z - gMazeEndZ);                        // Z move distance to end
    if (md < gMazeCellSize) { md = gMazeCellSize*0.5; }         // prevent divide by zero when close
    integer toofar = (zdiff / md) > MAZERMAXSLOPE;              // too far in Z if huge slope to end of maze
    if (!toofar) { return(FALSE); }                             // OK, within reasonable range
    pathMsg(PATH_MSG_WARN,"Close to end of maze at (" + (string)x + "," + (string)y + ") zdiff: " + (string)zdiff);
    return(TRUE);                                               // on wrong level in Z. Bad.
}

//
//  mazetestcell
//
//  Returns -1 if occupied cell.
//  Returns Z value if not occupied.
//
//  We have to test from a known-empty cell, because cast ray won't detect
//  an obstacle it is inside.
//                                                       
float mazetestcell(integer fromx, integer fromy, float fromz, integer x, integer y)
{
    if (x < 0 || x >= gMazeXsize || y < 0 || y >= gMazeYsize)  // if off grid
    {    return(-1.0);      }                     // treat as occupied
    integer v = mazecellget(x,y);
    if (v & MAZEEXAMINED) 
    {   DEBUGPRINT1("Checked cell (" + (string)x + "," + (string)y + ") : " + (string)(v & MAZEBARRIER));
        if (v & MAZEBARRIER) { return(-1.0); }  // occupied/
        //  Already tested, not occupied, and we need Z. 
        vector p0 = mazecelltopoint(fromx, fromy);          // centers of the start and end test cells
        p0.z = fromz;                                       // provide Z of previous point
        vector p1 = mazecelltopoint(x,y);                   // X and Y of next point, Z currently unknown.
        float z = pathcheckcellz(p0, p1, gMazeWidth, gMazeHeight); // get Z depth of cell
        if (z < 0)
        {   gMazeStatus = MAZESTATUSCELLCHANGED;            // status of cell changed
            pathMsg(PATH_MSG_WARN, "Maze cell (" + (string)x + "," + (string)y + ") was empty, now occupied");
        }
        return(z);                                          // return Z value for cell. Occupancy already checked.
    }
    //  This cell is not in the bitmap yet. Must do the expensive test.
    integer barrier = FALSE;                    // no barrier yet
    float z = -1.0;                             // assume barrier    
    //  Special cases for start and end of maze
    if (x == gMazeStartX && y == gMazeStartY) // special case where start pt is assumed clear
    {   z = gMazeStartZ; }                                  // needed to get character out of very tight spots.
    else if (x == gMazeEndX && y == gMazeEndY) // special case where end pt is assumed clear
    {   z = gMazeEndZ;                                      // needed to get character out of very tight spots.
        if (llFabs(z-fromz) > gMazeHeight*0.5)              // if final move to maze end has a big jump in Z.
        {   gMazeStatus = MAZESTATUSBADZ; return(-1.0); }   // fail
    }
    else
    {   //  Have to test the cell.
        vector p0 = mazecelltopoint(fromx, fromy);          // centers of the start and end test cells
        p0.z = fromz;                                       // provide Z of previous point
        vector p1 = mazecelltopoint(x,y);                   // X and Y of next point, Z currently unknown.
        z = mazecheckcelloccupied(p0, p1, gMazeCellSize, gMazeHeight, gMazeChartype, FALSE);    // test whether cell occupied, assuming prev cell was OK
        barrier = z < 0;                                    // negative Z means obstacle.
#ifdef MARKERS                                              // debug markers which appear in world
        rotation color = TRANSYELLOW;                       // yellow if unoccupied
        if (barrier) { color = TRANSRED; }                  // red if occupied
        p1.z = p0.z;                                        // use Z from p0 if cell check failed. So all fails are horizontal markers.
        if (z > 0) { p1.z = z; }                            // use actual Z from cell check if successful
        placesegmentmarker(MARKERLINE, p0, p1, gMazeWidth, color, 0.20);// place a temporary line on the ground in-world.
#endif // MARKERS
    }
    v = MAZEEXAMINED | barrier;
    mazecellset(x,y,v);                                     // update cells checked
    if (barrier && (x == gMazeEndX) && (y == gMazeEndY))    // if the end cell is blocked
    {   gMazeStatus = MAZESTATUSBADEND; }                   // force abort, maze is unsolveable
    DEBUGPRINT1("Tested cell (" + (string)x + "," + (string)y + ") : " + (string)(z));
    return(z);                                              // return -1 if obstacle, else Z heigth
}
 
//
//  mazeexistsproductivepath -- true if a productive path exists
//
//  A productive path is one that leads to the goal and isn't blocked
//     
integer mazeexistsproductivepath(integer x, integer y, float z)
{
    integer dx = gMazeEndX - x;
    integer dy = gMazeEndY - y;
    dx = mazeclipto1(dx);
    dy = mazeclipto1(dy);
    if (dx != 0) 
    {    float nextz = mazetestcell(x, y, z, x + dx, y); // test if cell in productive direction is clear    
         if (nextz > 0) { return(TRUE); }
    }
    if (dy != 0) 
    {    float nextz = mazetestcell(x, y, z, x, y + dy); // test if cell in productive direction is clear
         if (nextz > 0) { return(TRUE); }
    }
    return(FALSE);
}
//
//  mazeexistusefulpath -- path is both productive and better than best existing distance
//
integer mazeexistusefulpath(integer x, integer y, float z)
{
    if (mazemd(x, y, gMazeEndX, gMazeEndY) >= gMazeMdbest) { return(FALSE); }
    return(mazeexistsproductivepath(x,y,z)); 
}

//
//  mazetakeproductive path -- follow productive path one cell, or return 0
//
integer mazetakeproductivepath()
{
    integer dx = gMazeEndX - gMazeX;
    integer dy = gMazeEndY - gMazeY;
    integer clippeddx = mazeclipto1(dx);
    integer clippeddy = mazeclipto1(dy);
    assert(dx != 0 || dy != 0);              // error to call this at dest
    //    Try X dir first if more direct towards goal
    if (llAbs(dx) > llAbs(dy) && clippeddx) 
    {   float newz = mazetestcell(gMazeX, gMazeY, gMazeZ, gMazeX + clippeddx, gMazeY);
        if (newz > 0)                                   // if not obstructed
        {   gMazeX += clippeddx;                        // advance in desired dir
            gMazeZ = newz;                              // using Z value just obtained
            mazeaddtopath();
            return(TRUE);
        }
    }
    //   Then try Y    
    if (clippeddy) 
    {   float newz = mazetestcell(gMazeX, gMazeY, gMazeZ, gMazeX, gMazeY + clippeddy);
        if (newz > 0)
        {   gMazeY += clippeddy;                        // advance in desired dir
            gMazeZ = newz;                              // using Z value just obtained
            mazeaddtopath();
            return(TRUE); 
        }
    }
    //   Then X, regardless of whether abs(dx) > abs(dy)
    if (clippeddx)
    {   float newz = mazetestcell(gMazeX, gMazeY, gMazeZ, gMazeX + clippeddx, gMazeY);
        if (newz > 0) 
        {   gMazeX += clippeddx;                       // advance in desired dir
            gMazeZ = newz;                              // using Z value just obtained
            mazeaddtopath();
            return(TRUE);
        } 
    }                           // success
    DEBUGPRINT1("Take productive path failed");
    return(FALSE);
}                                               // hit wall, stop
//
//  mazepickside
//
//    Which side of the wall to follow?  Doesn't matter. We will follow both.
//
//  Always returns direction for following the right wall.
//
//       
integer mazepickside()
{
    integer direction;
    integer dx = gMazeEndX - gMazeX;
    integer dy = gMazeEndY - gMazeY;
    assert(dx != 0 || dy != 0);              // error to call this at dest
    integer clippeddx = mazeclipto1(dx);
    integer clippeddy = mazeclipto1(dy);
    if (llAbs(dx) > llAbs(dy))                    // better to move in X
    {    clippeddy = 0; } 
    else
    {    clippeddx = 0; }
    assert(mazetestcell(gMazeX, gMazeY, gMazeZ, gMazeX + clippeddx, gMazeY + clippeddy) < 0); // must have hit a wall
    //   4 cases
    if (clippeddx == 1)                      // obstacle is in +X dir
    {   
        direction = 1;
    } else if (clippeddx == -1) 
    {  
        direction = 3;              
    } else if (clippeddy == 1 )                  // obstacle is in +Y dir
    {  
        direction = 2;
    } else if (clippeddy == -1)                   // obstacle is in -Y dir
    {  
        direction = 0;
    } else {
        assert(FALSE);                       // should never get here
    }
    DEBUGPRINT("At (%d,%d) picked side %d, direction %d for wall follow." % (gMazeX, gMazeY, MAZEWALLONRIGHT, direction));
    return(direction);
}


//
//  mazewallfollow -- Follow wall from current point. Single move per call.
//        
//    Wall following rules:
//    Always blocked on follow side. Algorithm error if not.
//        
//    If blocked ahead && not blocked opposite follow side, inside corner
//            turn away from follow side. No move.
//    If blocked ahead && blocked opposite follow side, dead end
//            turn twice to reverse direction, no move.
//    If not blocked ahead && blocked on follow side 1 ahead, 
//            advance straight.
//    If not blocked ahead && not blocked on follow side 1 ahead, outside corner,
//            advance straight, 
//           turn towards follow side, 
//            advance straight.
//            
//    "sidelr" is 1 for left, -1 for right
//    "direction" is 0 for +X, 1 for +Y, 2 for -X, 3 for -Y
//
//  No use of global variables. Returns
//  [pt, pt, ... , x, y, z, direction]   
//
//  and "params" uses the same format
//
//  Z is updated from the appropriate call to mazetestcell.
//
list mazewallfollow(list params, integer sidelr)
{
    integer x = llList2Integer(params,-4);
    integer y = llList2Integer(params,-3);
    float z = llList2Float(params,-2);
    integer direction = llList2Integer(params,-1);
    list path = llListReplaceList(params,[],-4,-1); // remove non-path items.
    pathMsg(PATH_MSG_INFO,"Following wall at (" + (string)x + "," + (string)y + "," + (string)z + ")" 
        + " side " + (string)sidelr + " direction " + (string) direction 
        + "  md: " + (string)mazemd(x, y, gMazeEndX, gMazeEndY) + " mdbest: " + (string)gMazeMdbest);
    integer dx = MAZEEDGEFOLLOWDX(direction);
    integer dy = MAZEEDGEFOLLOWDY(direction);
    integer dxsame = MAZEEDGEFOLLOWDX(((direction + sidelr) + 4) % 4); // if not blocked ahead
    integer dysame = MAZEEDGEFOLLOWDY(((direction + sidelr) + 4) % 4); 
    float followedsidez = mazetestcell(x, y, z, x + dxsame, y+dysame);
    assert(followedsidez < 0);                              // must be next to obstacle on wall-following side
    float aheadz = mazetestcell(x, y, z, x + dx, y + dy);   // Z value at x+dx, y+dy, or -1
    if (aheadz < 0)                                         // if blocked ahead
    {   integer dxopposite = MAZEEDGEFOLLOWDX(((direction - sidelr) + 4) % 4);
        integer dyopposite = MAZEEDGEFOLLOWDY(((direction - sidelr) + 4) % 4);
        float oppositez = mazetestcell(x, y, z, x + dxopposite, y + dyopposite);
        if (oppositez < 0)                                  // blocked ahead, and blocked on followed side - dead end.
        {   DEBUGPRINT1("Dead end");
            direction = (direction + 2) % 4;         // dead end, reverse direction
        } else {
            DEBUGPRINT1("Inside corner");
            direction = (direction - sidelr + 4) % 4;      // inside corner, turn
        }
    } else {                                        // not blocked ahead, can go straight or do an outside corner.
        assert(dxsame == 0 || dysame == 0);
        float sameaheadz = mazetestcell(x + dx, y + dy, aheadz, x + dx + dxsame, y + dy + dysame);
        if (sameaheadz < 0)                         // straight, not outside corner
        {   DEBUGPRINT1("Straight");
            x += dx;                                // move ahead 1
            y += dy;
            z = aheadz;                             // use Z value for (x+dx, y+dy)
            path = mazeaddpttolist(path,x,y,z);
        } else {                                    // outside corner
            DEBUGPRINT1("Outside corner");
            x += dx;                                // move ahead 1
            y += dy;
            z = aheadz;                             // use Z value for (x+dx, y+dy)
            path = mazeaddpttolist(path,x,y,z);
            //   Need to check for a productive path. May be time to stop wall following
            integer md = mazemd(x, y, gMazeEndX, gMazeEndY);
            if (md < gMazeMdbest && mazeexistsproductivepath(x,y,z))
            {
                DEBUGPRINT1("Outside corner led to a productive path halfway through");
                return(path + [x, y, z, direction]);
            }
            direction = (direction + sidelr + 4) % 4;    // turn in direction
            x += dxsame;                        // move around corner
            y += dysame;
            z = sameaheadz;                         // use Z value for (x+dx+dxsame, y+dy+dysame)
            path = mazeaddpttolist(path,x,y,z);
        }
    }
    return(path + [x, y, z, direction]);           // return path plus state
} 
#ifdef OBSOLETE
//
//  mazereplyjson -- construct result as JSON
//
//  Format:
//  { "reply" : "mazesolve" , "status" : INTEGER, "serial", INTEGER", points: [pt,pt,...]}
//
//  Points are packed with x and y in one integer.
//
//  "status" is 0 if successful, nonzero if a problem.
//  "serial" is the serial number from the query, to match reply with request.
//
//  Only used for testing.
//
string mazereplyjson(integer status, integer pathid, integer segmentid, list path)
{   return (llList2Json(JSON_OBJECT, ["reply", "mazesolve", "status", status, "pathid", pathid, "segmentid", segmentid,
        "points", llList2Json(JSON_ARRAY, path)]));
}
#endif // OBSOLETE

vector gMazePos;                                // location of maze in SL world space
rotation gMazeRot;                              // rotation of maze in SL world space
float gMazeCellSize;                            // size of cell in world
float gMazeProbeSpacing;                        // probe spacing for llCastRay
float gMazeHeight;                              // character height
float gMazeWidth;                               // character diameter
integer gMazeChartype;                          // character type (static path)
key gMazeHitobj;                                // obstacle which caused the maze solve to start
//
//  mazerequestjson -- request a maze solve via JSON
//
//  Format:
//  { "request" : "mazesolve",  "msglevG" : INTEGER, "serial": INTEGER,
//      "regioncorner" : VECTOR, "pos": VECTOR, "rot" : QUATERNION, "cellsize": FLOAT, "probespacing" : FLOAT, 
//      "width" : FLOAT, "height" : FLOAT, chartype: INTEGER,
//      "sizex", INTEGER, "sizey", INTEGER, 
//      "startx" : INTEGER, "starty" : INTEGER, "endx" : INTEGER, "endy" : INTEGER }
//      
//  "regioncorner", "pos" and "rot" identify the coordinates of the CENTER of the (0,0) cell of the maze grid.
//  Ray casts are calculated accordingly.
//  "cellsize" is the edge length of each square cell.
//  "probespacing" is the spacing between llCastRay probes.
//  "height" and "radius" define the avatar's capsule. 
//  
mazerequestjson(string jsn, key id) 
{   gPathMsgLevel = (integer)llJsonGetValue(jsn,["msglev"]);// so first message will print
    pathMsg(PATH_MSG_INFO,"Request to maze solver: " + jsn);            // verbose mode
    integer status = 0;                                     // so far, so good
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type
    if (requesttype != "mazesolve") { return; }              // ignore, not our msg
    integer pathid = (integer) llJsonGetValue(jsn, ["pathid"]); 
    integer segmentid = (integer)llJsonGetValue(jsn,["segmentid"]);
    vector regioncorner = (vector)llJsonGetValue(jsn,["regioncorner"]);
    gMazePos = (vector)llJsonGetValue(jsn,["pos"]);
    gMazeRot = (rotation)llJsonGetValue(jsn,["rot"]);
    gMazeCellSize = (float)llJsonGetValue(jsn,["cellsize"]);
    gMazeProbeSpacing = (float)llJsonGetValue(jsn,["probespacing"]);
    gMazeHeight = (float)llJsonGetValue(jsn,["height"]);
    gMazeWidth = (float)llJsonGetValue(jsn,["width"]);
    gMazeChartype = (integer)llJsonGetValue(jsn,["chartype"]);
    gMazeHitobj = (key)llJsonGetValue(jsn,["hitobj"]);
    integer sizex = (integer)llJsonGetValue(jsn,["sizex"]);
    integer sizey = (integer)llJsonGetValue(jsn,["sizey"]);
    integer startx = (integer)llJsonGetValue(jsn,["startx"]);
    integer starty = (integer)llJsonGetValue(jsn,["starty"]);
    float startz = (float)llJsonGetValue(jsn,["startz"]);           // elevation in world coords
    integer endx = (integer)llJsonGetValue(jsn,["endx"]);
    integer endy = (integer)llJsonGetValue(jsn,["endy"]);
    float endz = (float)llJsonGetValue(jsn,["endz"]);           // elevation in world coords
    vector gp0 = (vector)llJsonGetValue(jsn,["p0"]);            // for checking only
    vector gp1 = (vector)llJsonGetValue(jsn,["p1"]);            // for checking only
    if (sizex < 3 || sizex > MAZEMAXSIZE || sizey < 3 || sizey > MAZEMAXSIZE) { status = MAZESTATUSBADSIZE; } // too big
    list path = [];
    if (status == 0)                                    // if params sane enough to start
    {   path = mazesolve(sizex, sizey, startx, starty, startz, endx, endy, endz); // solve the maze
        gMazeCells = [];                                // release memory before building JSON to save space.
        gMazePath = [];
        if (llGetListLength(path) == 0 || gMazeStatus != 0)       // failed to find a path
        {   path = [];                                  // clear path
            status = gMazeStatus;                       // failed for known reason, report
            if (status == 0) { status = MAZESTATUSNOFIND; }  // generic no-find status
        } else {
            ////path = mazeoptimizeroute(path);             // do simple optimizations
        } 
    }
    {   pathMsg(PATH_MSG_WARN,"Maze solver finished, pathid " + (string)pathid + ", seg " + (string)segmentid + 
        ". Free mem: " + (string)llGetFreeMemory()); 
        pathMsg(PATH_MSG_INFO,"Maze route: " + mazerouteasstring(path));    // detailed debug
    } 
    //  Send reply                  
    llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT, ["reply", "mazesolve", "pathid", pathid, "segmentid", segmentid, "status", status,
        "hitobj",gMazeHitobj,
        "pos", gMazePos, "rot", gMazeRot, "cellsize", gMazeCellSize,
        "p0",gp0, "p1",gp1,                                 // for checking only
        "points", llList2Json(JSON_ARRAY, path)]),"");
}

//
//  mazecelltopoint -- convert maze coordinates to point in world space
//
vector mazecelltopoint(integer x, integer y)
{   return(mazecellto3d(x, y, gMazeCellSize, gMazePos, gMazeRot)); }
                
//
//   Test-only code
//

//
//  mazerouteasstring -- display route a string
//
//  Dump a route, which has X && Y encoded into one value
// 
string mazerouteasstring(list route)
{
    string s = "";
    integer length = llGetListLength(route);
    integer i;
    for (i=0; i<length; i++)
    {   integer val = llList2Integer(route,i);
        integer x = mazepathx(val);
        integer y = mazepathy(val);
        ////s = s + ("(%d,%d) " % (x,y))
        s = s + "(" + (string)x + "," + (string)y + ") ";
    }
    return(s);
}


//
//  The main program of the maze solver task
//
//
default
{

    state_entry()
    {   pathinitutils(); }                              // library init
   
    link_message( integer sender_num, integer num, string jsn, key id )
    {   if (num == MAZESOLVEREQUEST)
        {   mazerequestjson(jsn, id);                   // solve maze
        } else if (num == PATHPARAMSINIT)
        {   pathinitparams(jsn); }                      // initialize params
    }

}

