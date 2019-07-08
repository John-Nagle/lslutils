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
//  1. Add checking for getting close to space limits, and return failure before a stack/heap collision.
//  2. Add backup counter to detect runaways now that collinear point optimization is in. 
//
//
#include "npc/pathbuildutils.lsl"

//  Constants
//
#define MAZEBARRIER (0x1)                                   // must be low bit
#define MAZEEXAMINED (0x2)

#define MAZEINVALIDPT (0xffffffff)                          // invalid point value in old point storage

#define MAZEMINMEM (4096)                                   // make sure we have this much memory left
//   Wall follow sides
#define MAZEWALLONLEFT  1
#define MAZEWALLONRIGHT (-1)
list EDGEFOLLOWDIRX = [1,0,-1,0];
list EDGEFOLLOWDIRY = [0, 1, 0, -1];

list MAZEEDGEFOLLOWDXTAB = [1,0,-1,0];
list MAZEEDGEFOLLOWDYTAB = [0,1,0,-1];
#define MAZEEDGEFOLLOWDX(n) llList2Integer(MAZEEDGEFOLLOWDXTAB,(n))
#define MAZEEDGEFOLLOWDY(n) llList2Integer(MAZEEDGEFOLLOWDYTAB,(n))

#define MAZEPRINTVERBOSE(s) { if (verbose) { llOwnerSay((s)); } }
////#define DEBUG
#ifdef DEBUG
#define DEBUGPRINT(s) // Nothing for now
#define DEBUGPRINT1(s) llOwnerSay(s)
////#define assert(exp) // Nothing for now
#define assert(exp) { if (!(exp)) { llOwnerSay("Assertion failed at " + __SHORTFILE__ + " line " + (string) __LINE__); panic(); }}
#else // not debugging
#define DEBUGPRINT(s) {}
#define DEBUGPRINT1(s) {}
#define assert(exp) {}
#endif // DEBUG

//
//  abs -- absolute value, integer
//
integer abs(integer n)
{   if (n < 0) { return(-n); }
    return(n);
}
//
//   mazemd -- rectangular "Manhattan" distance
//
integer mazemd(integer p0x, integer p0y, integer p1x, integer p1y)
{   return abs(p1x-p0x) + abs(p1y-p0y); }
    
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
integer gMazeMdbest;                // best point found
integer gMazeXsize;                 // maze dimensions
integer gMazeYsize;
integer gMazeStartX;                // start position 
integer gMazeStartY;           
integer gMazeEndX;                  // end position 
integer gMazeEndY; 
integer gMazePrev0;                 // previous point, invalid value
integer gMazePrev1;                 // previous point, invalid value
integer gMazeStatus;                // status code - MAZESTATUS...
integer gMazeVerbose;               // verbose mode
       

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
#ifndef mazepathx        
//
//   Maze path storage - X && Y in one 32-bit value
//
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value
#endif // mazepathx


//
//  mazesolve  -- find a path through a maze
//         
list mazesolve(integer xsize, integer ysize, integer startx, integer starty, integer endx, integer endy, integer verbose)
{   gMazeVerbose = verbose;
    gMazeStatus = 0;                        // OK so far
    gMazeXsize = xsize;                     // set size of map
    gMazeYsize = ysize;
    gMazeCells = [];
    while (llGetListLength(gMazeCells) < (xsize*ysize+15)/16)        // allocate cell list
    {    gMazeCells = gMazeCells + [0]; }   // fill list with zeroes 
    gMazeX = startx;                        // start
    gMazeY = starty;
    gMazeStartX = startx;                   // start
    gMazeStartY = starty;
    gMazeEndX = endx;                       // destination
    gMazeEndY = endy;
    gMazeMdbest = gMazeXsize+gMazeYsize+1;  // best dist to target init
    gMazePath = [];                         // accumulated path
    gMazePrev0 = MAZEINVALIDPT;             // previous point, invalid value
    gMazePrev1 = MAZEINVALIDPT;             // previous point out, invalid value
    mazeaddtopath();                        // add initial point
    //   Outer loop - shortcuts || wall following
    MAZEPRINTVERBOSE("Start maze solve.");
    while (gMazeX != gMazeEndX || gMazeY != gMazeEndY)  // while not at dest
    {   MAZEPRINTVERBOSE("Maze solve at (" + (string)gMazeX + "," + (string)gMazeY + ")");
        if (gMazeStatus)                                    // if something went wrong
        {   if (verbose)
            {   llOwnerSay("Maze solver failed: status " + (string)gMazeStatus + " at (" 
                + (string)gMazeX + "," + (string)gMazeY + ")");
            }
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
        if (mazeexistsproductivepath(gMazeX, gMazeY))     // if a shortcut is available
        {   MAZEPRINTVERBOSE("Maze productive path at (" + (string)gMazeX + "," + (string)gMazeY + ")");
            mazetakeproductivepath();       // use it
            gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY);
            ////////gMazeMdbest = gMazeMdbest -1 
            assert(gMazeMdbest >= 0);
        } else {
            ////////gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)
            ////sidelr, direction = mazepickside()        // follow left || right?
            ////list picksideinfo = mazepickside();
            ////integer sidelr = llList2Integer(picksideinfo,0);
            ////integer direction= llList2Integer(picksideinfo,1);
            integer direction = mazepickside();                                 // direction for following right wall
            integer sidelr = MAZEWALLONRIGHT;                                   // always returns dir for following right wall
            //   Inner loop - wall following
            integer followstartx = gMazeX;
            integer followstarty = gMazeY;
            integer followstartdir = direction;
            MAZEPRINTVERBOSE("Starting wall follow at " + (string)followstartx + "," + (string)followstarty + ",  direction " + (string)direction + ", mdist = " + (string)gMazeMdbest);
            integer livea = TRUE;                                               // more to do on path a
            integer liveb = TRUE;                                               // more to do on path b
            integer founduseful = FALSE;                                        // found a useful path
            list patha = [followstartx, followstarty, followstartdir];          // start conditions, follow one side
            list pathb = [followstartx, followstarty, (followstartdir + 2) % 4];    // the other way
            while (gMazeStatus == 0 && (!founduseful) && (livea || liveb))      // if more following required
            {   
                // Advance each path one cell
                if (livea)                                                      // if path A still live
                {   patha = mazewallfollow(patha, sidelr);                      // follow one wall
                    DEBUGPRINT1("Path A: " + llDumpList2String(patha,","));
                    DEBUGPRINT1("Path A in: " + mazerouteasstring(llListReplaceList(patha, [], -3,-1))); // ***TEMP***
                    integer x = llList2Integer(patha,-3);                       // get X and Y from path list
                    integer y = llList2Integer(patha,-2);
                    integer dir = llList2Integer(patha,-1);                     // direction
                    if (gMazeStatus == 0 && (x == gMazeEndX) && (y == gMazeEndY))    // reached final goal
                    {   list goodpath = gMazePath + llListReplaceList(patha, [], -3,-1);   // get good path
                        gMazeCells = [];
                        gMazePath = [];
                        DEBUGPRINT1("Path A reached goal: " + mazerouteasstring(llListReplaceList(pathb, [], -3,-1)));
                        return(goodpath);
                    }
                    if ((x != followstartx || y != followstarty) && mazeexistusefulpath(x,y))  // if useful shortcut, time to stop wall following
                    {   list goodpath = gMazePath + llListReplaceList(patha, [], -3,-1);   // get good path
                        gMazePath = goodpath;                                   // add to accumulated path
                        gMazeX = x;                                             // update position
                        gMazeY = y;
                        founduseful = TRUE;                                     // force exit
                        DEBUGPRINT1("Path A useful: " + mazerouteasstring(llListReplaceList(patha, [], -3,-1)));
                    }
                    if (x == followstartx && y == followstarty && dir == followstartdir) 
                    {   DEBUGPRINT1("Path A stuck."); livea = FALSE; }          // in a loop wall following, stuck ***MAY FAIL - CHECK***
                }
                if (liveb && !founduseful)                                      // if path B still live and no solution found
                {   pathb = mazewallfollow(pathb, -sidelr);                     // follow other wall
                    DEBUGPRINT1("Path B: " + llDumpList2String(pathb,","));
                    DEBUGPRINT1("Path B in: " + mazerouteasstring(llListReplaceList(pathb, [], -3,-1))); // ***TEMP***
                    integer x = llList2Integer(pathb,-3);                       // get X and Y from path list
                    integer y = llList2Integer(pathb,-2);
                    integer dir = llList2Integer(pathb,-1);                     // direction
                    if (gMazeStatus == 0 && (x == gMazeEndX) && (y == gMazeEndY))    // reached final goal
                    {   list goodpath = gMazePath + llListReplaceList(pathb, [], -3,-1);   // get good path
                        gMazeCells = [];
                        gMazePath = [];
                        DEBUGPRINT1("Path B reached goal: " + mazerouteasstring(llListReplaceList(pathb, [], -3,-1)));
                        return(goodpath);
                    }
                    if ((x != followstartx || y != followstarty) && mazeexistusefulpath(x,y))    // if useful shortcut, time to stop wall following
                    {   list goodpath = gMazePath + llListReplaceList(pathb, [], -3,-1);   // get good path
                        gMazePath = goodpath;                                   // add to accmulated path
                        gMazeX = x;                                             // update position
                        gMazeY = y;
                        founduseful = TRUE;                                     // force exit
                        DEBUGPRINT1("Path B useful: " + mazerouteasstring(llListReplaceList(pathb, [], -3,-1)));
                    }
                    if (x == followstartx && y == followstarty && dir == (followstartdir + 2) % 4)
                    {   DEBUGPRINT1("Path B stuck"); liveb = FALSE; } // in a loop wall following, stuck
                }           
                //  Termination conditions
                //  Consider adding check for paths collided from opposite directions. This is just a speedup, though.
            }
            if (!founduseful)                                                       // stopped following, but no result
            {   gMazePath = [];                                                     // failed, release memory and return
                gMazeCells = [];
                DEBUGPRINT1("No solution. Status: " + (string)gMazeStatus);
                return([]);                                                         // no path possible
            }

            DEBUGPRINT1("Finished wall following at (" + (string)gMazeX + "," + (string)gMazeY + ")");
        }
    }
    MAZEPRINTVERBOSE("Solved maze");
    if (gMazePrev1 != MAZEINVALIDPT) { gMazePath += [gMazePrev1]; } // ***OBSOLETE***
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
    gMazePath = mazeaddpttolist(gMazePath, gMazeX, gMazeY);        // use common fn
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
list mazeaddpttolist(list path, integer x, integer y)
{
    DEBUGPRINT1("(" + (string)x + "," + (string)y + ")");
    //  Memory check
    if (llGetFreeMemory() < MAZEMINMEM)             // if in danger of stack/heap collision crash
    {   gMazeStatus = MAZESTATUSNOMEM; }            // out of memory, will abort
    //  Short list check
    integer val = mazepathval(x, y);                // current point as one integer
    integer length = llGetListLength(path);
    if (length > 0) 
    {   if (llList2Integer(path,-1) == val) { DEBUGPRINT1("Dup pt."); return(path); }}   // new point is dup, ignore.
    if (length >= 3)                            // if at least 3 points
    {   integer gMazePrev0 = llList2Integer(path,-2);
        integer gMazePrev1 = llList2Integer(path,-1);
        //  Check for collinear points.
        if (mazeinline(mazepathx(gMazePrev0), mazepathy(gMazePrev0),
                mazepathx(gMazePrev1), mazepathy(gMazePrev1),
                x, y))  
        {   DEBUGPRINT1("Collinear pt"); return(llListReplaceList(path,[val],-1,-1)); } // new point replaces prev point 
    } 
    //  No optimizations, just add new point
    {   return(path + [val]); }                 // no optimization
}

//
//  mazetestcell
//
//  Returns 1 if occupied cell.
//
//  We have to test from a known-empty cell, because cast ray won't detect
//  an obstacle it is inside.
//                                                       
integer mazetestcell(integer fromx, integer fromy, integer x, integer y)
{
    DEBUGPRINT("Testcell (%d,%d)" % (x,y));       // ***TEMP***
    if (x < 0 || x >= gMazeXsize || y < 0 || y >= gMazeYsize)  // if off grid
    {    return(1);      }                      // treat as occupied
    integer v = mazecellget(x,y);
    if (v & MAZEEXAMINED) 
    {   return(v & MAZEBARRIER); }              // already have this one
    integer barrier = gBarrierFn(fromx, fromy, x,y); // check this location
    v = MAZEEXAMINED | barrier;
    mazecellset(x,y,v);                         // update cells checked
    if (barrier && (x == gMazeEndX) && (y == gMazeEndY))    // if the end cell is blocked
    {   gMazeStatus = MAZESTATUSBADEND; }       // force abort, maze is unsolveable
    return(barrier);                            // return 1 if obstacle
}
 
//
//  mazeexistsproductivepath -- true if a productive path exists
//
//  A productive path is one that leads to the goal and isn't blocked
//     
integer mazeexistsproductivepath(integer x, integer y)
{
    integer dx = gMazeEndX - x;
    integer dy = gMazeEndY - y;
    dx = mazeclipto1(dx);
    dy = mazeclipto1(dy);
    if (dx != 0) 
    {    integer productive = !mazetestcell(x, y, x + dx, y); // test if cell in productive direction is clear
         if (productive) { return(TRUE); }
    }
    if (dy != 0) 
    {    integer productive = !mazetestcell(x, y, x, y + dy); // test if cell in productive direction is clear
         if (productive) { return(TRUE); }
    }
    return(FALSE);
}
//
//  mazeexistusefulpath -- path is both productive and better than best existing distance
//
integer mazeexistusefulpath(integer x, integer y)
{
    if (mazemd(x, y, gMazeEndX, gMazeEndY) >= gMazeMdbest) { return(FALSE); }
    return(mazeexistsproductivepath(x,y)); 
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
    if (abs(dx) > abs(dy) && clippeddx) 
    {
        if (!mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY)) 
        {   gMazeX += clippeddx;                      // advance in desired dir
            mazeaddtopath();
            return(1);
        }
    }
    //   Then try Y    
    if (clippeddy) 
    {   if (!mazetestcell(gMazeX, gMazeY, gMazeX, gMazeY + clippeddy))
        {   gMazeY += clippeddy;                       // advance in desired dir
            mazeaddtopath();
            return(1); 
        }
    }
    //   Then X, regardless of whether abs(dx) > abs(dy)
    if (clippeddx)
    {   if (!mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY)) 
        {   gMazeX += clippeddx;                       // advance in desired dir
            mazeaddtopath();
            return(1);
        } 
    }                           // success
    DEBUGPRINT1("Take productive path failed");
    return(0);
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
    ////integer sidelr;
    integer direction;
    integer dx = gMazeEndX - gMazeX;
    integer dy = gMazeEndY - gMazeY;
    assert(dx != 0 || dy != 0);              // error to call this at dest
    integer clippeddx = mazeclipto1(dx);
    integer clippeddy = mazeclipto1(dy);
    if (abs(dx) > abs(dy))                    // better to move in X
    {    clippeddy = 0; } 
    else
    {    clippeddx = 0; }
    assert(mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY + clippeddy)); // must have hit a wall
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
    ////sidelr = MAZEWALLONRIGHT;
    DEBUGPRINT("At (%d,%d) picked side %d, direction %d for wall follow." % (gMazeX, gMazeY, MAZEWALLONRIGHT, direction));
    return(direction);
    ////return([sidelr, direction]);
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
//  [pt, pt, ... , x, y, direction]   
//
//  and "params" uses the same format
//
list mazewallfollow(list params, integer sidelr)
{
    integer x = llList2Integer(params,-3);
    integer y = llList2Integer(params,-2);
    integer direction = llList2Integer(params,-1);
    list path = llListReplaceList(params,[],-3,-1); // remove non-path items.
    DEBUGPRINT1("Following wall at (" + (string)x + "," + (string)y + ")" + " side " + (string)sidelr + " direction " + (string) direction + " md " + (string)mazemd(x, y, 
            gMazeEndX, gMazeEndY));
    integer dx = MAZEEDGEFOLLOWDX(direction);
    integer dy = MAZEEDGEFOLLOWDY(direction);
    integer dxsame = MAZEEDGEFOLLOWDX(((direction + sidelr) + 4) % 4); // if not blocked ahead
    integer dysame = MAZEEDGEFOLLOWDY(((direction + sidelr) + 4) % 4); 
    integer followedside = mazetestcell(x, y, x + dxsame, y+dysame);
    if (!followedside) 
    {
        DEBUGPRINT("***ERROR*** followedside not blocked. dx,dy: (%d,%d)  dxsame,dysame: (%d,%d) sidelr %d direction %d" %
                (dx,dy, dxsame,dysame, sidelr,direction));
        assert(followedside);                            // must be next to obstacle
    }
    integer blockedahead = mazetestcell(x, y, x + dx, y + dy);
    if (blockedahead)
    {   integer dxopposite = MAZEEDGEFOLLOWDX(((direction - sidelr) + 4) % 4);
        integer dyopposite = MAZEEDGEFOLLOWDY(((direction - sidelr) + 4) % 4);
        integer blockedopposite = mazetestcell(x, y, x + dxopposite, y + dyopposite);
        if (blockedopposite) 
        {   DEBUGPRINT1("Dead end");
            direction = (direction + 2) % 4;         // dead end, reverse direction
        } else {
            DEBUGPRINT1("Inside corner");
            direction = (direction - sidelr + 4) % 4;      // inside corner, turn
        }
    } else {
        assert(dxsame == 0 || dysame == 0);
        integer blockedsameahead = mazetestcell(x + dx, y + dy, x + dx + dxsame, y + dy + dysame);
        if (blockedsameahead)                       // straight, not outside corner
        {   DEBUGPRINT1("Straight");
            x += dx;                            // move ahead 1
            y += dy;
            path = mazeaddpttolist(path,x,y);
        } else {                                     // outside corner
            DEBUGPRINT1("Outside corner");
            x += dx;                            // move ahead 1
            y += dy;
            path = mazeaddpttolist(path,x,y);
            //   Need to check for a productive path. May be time to stop wall following
            integer md = mazemd(x, y, gMazeEndX, gMazeEndY);
            if (md < gMazeMdbest && mazeexistsproductivepath(x,y))
            {
                DEBUGPRINT1("Outside corner led to a productive path halfway through");
                return(path + [x, y, direction]);
            }
            direction = (direction + sidelr + 4) % 4;    // turn in direction
            x += dxsame;                        // move around corner
            y += dysame;
            path = mazeaddpttolist(path,x,y);
        }
    }
    return(path + [x, y, direction]);           // return path plus state
} 

//
//  mazelinebarrier -- Does the line between the two points, inclusive, hit a barrier?
//
//  Assumes the start point is not on a barrier cell.
//  Because we have to work forward from an empty cell so that llCastRay will work.
//        
integer mazelinebarrier(integer x0, integer y0, integer x1, integer y1) 
{   DEBUGPRINT("Maze test barrier: (%d,%d),(%d,%d)" % (x0,y0,x1,y1))
    if (x0 == x1)                          // vertical line
    {   assert(y0 != y1);                  // must not be zero length
        if (y0 > y1)                       // sort
        {   integer temp = y0;
            y0 = y1;
            y1 = temp;
        }
        assert(y1 > y0);
        integer y;
        for (y=y0; y<y1; y++)
        ////for y in range(y0,y1) :            // test each segment
        {   if (mazetestcell(x0, y, x0, y+1)) { return(TRUE); }}              // hit barrier
        ////return(FALSE);
    } else {
        assert(y0 == y1);
        assert(x0 != x1);
        if (x0 > x1)                       // sort
        {   integer temp = x0;
            x0 = x1;
            x1 = temp;
        }
        assert(x1 > x0);
        ////for x in range(x0,x1) :
        integer x;
        for (x=x0; x<x1; x++)
        {   if (mazetestcell(x, y0, x+1, y0)) { return(TRUE); }}               // hit barrier
        ////return(FALSE);
    }
    return(FALSE);                          // no obstacle found
}         
           
//
//    Locally optimize route.
//        
//    The incoming route should have corners only, && represent only horizontal && vertical lines.
//    Optimizing the route looks at groups of 4 points. If the two turns are both the same, then try
//    to eliminate one of the points by moving the line between the two middle points.
//    
//    O(n)        
//
//       
list mazeoptimizeroute(list route) 
{
    integer n = 0;
    //   Advance through route. On each iteration, either the route gets shorter, or n gets
    //   larger, so this should always terminate.
    while (n < llGetListLength(route)-3)                         // advancing through route
    {   integer p0val = llList2Integer(route,n);                            // get next four points
        integer p1val = llList2Integer(route,n+1);                            // get next four points
        integer p2val = llList2Integer(route,n+2);                            // get next four points
        integer p3val = llList2Integer(route,n+3);                            // get next four points
        integer p0x = mazepathx(p0val);
        integer p0y = mazepathy(p0val);
        integer p1x = mazepathx(p1val);
        integer p1y = mazepathy(p1val);
        integer p2x = mazepathx(p2val);
        integer p2y = mazepathy(p2val);
        integer p3x = mazepathx(p3val);
        integer p3y = mazepathy(p3val);
        ////DEBUGPRINT("%d: (%d,%d) (%d,%d) (%d,%d) (%d,%d)" % (n, p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y)) // ***TEMP***

        //   Remove collinear redundant points. The redundant point may not be
        //   between the endpoints, but that's OK. It's just removing a move to
        //   a dead end && back.
        if (p0x == p1x && p0y == p1y)             // redundant point
        {
            ////////DEBUGPRINT("Removing redundant point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+1, n+1);
            if (n > 0) { n = n - 1; }               // back up 1, may have created new redundant group                
            jump continue;
        }
        if (p1x == p2x && p1y == p2y)              // redundant point
        {   ////////DEBUGPRINT("Removing redundant point %d from %s" % (n+2, str(route)))
            route = listreplacelist(route, [], n+2, n+2);
            if (n > 0) { n = n - 1; }               // back up 1, may have created new redundant group                
            jump continue;
        }
        if (mazeinline(p0x,p0y,p1x,p1y,p2x,p2y)) 
        {   ////////DEBUGPRINT("Removing collinear point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+1, n+1);
            if (n > 0) { n = n - 1; }               // back up 1, may have created new redundant group                
            jump continue;
        }
        if (mazeinline(p1x,p1y,p2x,p2y,p3x,p3y))
        {   ////////DEBUGPRINT("Removing collinear point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+2, n+2);
            if (n > 0) { n = n - 1; }               // back up 1, may have created new redundant group                
            jump continue;
        }                
        if (p1x == p2x)                             // if vertical middle segment
        {   //   End segments must be horizontal
            assert(p0y == p1y);
            assert(p2y == p3y);
            //   Is this C-shaped?
            if (!((p0x > p1x) == (p2x < p3x)))   // no, not C-shaped
            {   n = n + 1;
                jump continue;
            }
            //   Find shorter arm of C
            integer armlena = p0x-p1x;
            integer armlenb = p3x-p2x;
            if (abs(armlena) > abs(armlenb))         // second arm is shorter
            {
                //   We will try to move middle segment to align with p0y, ignoring p1y
                if (mazelinebarrier(p3x, p0y, p3x, p3y))  // if blocked
                {   n = n + 1;
                    jump continue;
                }
                //   We can get rid of p1 && replace p2
                route = listreplacelist(route, [mazepathval(p3x,p0y)], n+1, n+2); // remove p1
                DEBUGPRINT("Vertical middle segment shortened at p1: %d: (%d,%d)" % (n+1,p3x,p0y))
                jump continue;
            } else {
                //   We will try to move middle segment to align with p3y, ignoring p2y
                if (mazelinebarrier(p0x, p0y, p0x, p3y))  // if blocked
                {   n = n + 1;
                    jump continue;
                }
                //   We can get rid of p2 && replace p1
                route = listreplacelist(route, [mazepathval(p0x, p3y)], n+1, n+2); // remove p2
                DEBUGPRINT("Vertical middle segment shortened at p2: %d: (%d,%d)" % (n+1,p0x,p3y))
                jump continue;                       
            }     
        } else {                                     // if horizontal middle segment
            assert(p1y == p2y);
            //   End segments must be vertical
            assert(p0x == p1x);
            assert(p2x == p3x);
            //   Is this C-shaped?
            if (! ((p0y > p1y) == (p2y < p3y)))    // no, not C-shaped
            {   n = n + 1;
                jump continue;
            }
            //   Find shorter arm of C
            integer armlena = p0y-p1y; 
            integer armlenb = p3y-p2y;
            if (abs(armlena) > abs(armlenb))         // second arm is shorter
            {
                //   We will try to move middle segment to align with p3y
                if (mazelinebarrier(p0x, p3y, p3x, p3y))  // if blocked
                {   n = n + 1;
                    jump continue;
                }
                //   We can get rid of p1 && p2 && replace with new point
                route = listreplacelist(route, [mazepathval(p1x, p3y)], n+1, n+2); // replace p1 && p2
                DEBUGPRINT("Horizontal middle segment shortened at p1: %d: (%d,%d)" % (n+1,p1x,p3y))
                jump continue;
            } else {
                //   We will try to move middle segment to align with p0y
                if (mazelinebarrier(p0x, p0y, p3x, p0y))  // if blocked
                {   n = n + 1;
                    jump continue;
                }
                //   We can get rid of p1 && p2 && replace with new point
                route = listreplacelist(route, [mazepathval(p2x,p0y)], n+1, n+2); // replace p1 && p2 with new point
                DEBUGPRINT("Horizontal middle segment shortened at p2: %d: (%d,%d)" % (n+1,p2x,p0y))
                jump continue;
            } 
        }
    @continue;                                          // the first "jump" I have had to code in decades
    }
    return(route);                                   // condensed route                      
}

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
string mazereplyjson(integer status, integer serial, list path)
{   return (llList2Json(JSON_OBJECT, ["reply", "mazesolve", "status", status, "serial", serial,
        "points", llList2Json(JSON_ARRAY, path)]));
}

vector gMazePos;                                // location of maze in SL world space
rotation gMazeRot;                              // rotation of maze in SL world space
float gMazeCellSize;                            // size of cell in world
float gMazeProbeSpacing;                        // probe spacing for llCastRay
float gMazeHeight;                              // character height
float gMazeWidth;                               // character diameter
//
//  mazerequestjson -- request a maze solve via JSON
//
//  Format:
//  { "request" : "mazesolve",  "verbose" : INTEGER, "serial": INTEGER,
//      "regioncorner" : VECTOR, "pos": VECTOR, "rot" : QUATERNION, "cellsize": FLOAT, "probespacing" : FLOAT, 
//      "width" : FLOAT, "height" : FLOAT, 
//      "sizex", INTEGER, "sizey", INTEGER, 
//      "startx" : INTEGER, "starty" : INTEGER, "endx" : INTEGER, "endy" : INTEGER }
//      
//  "regioncorner", "pos" and "rot" identify the coordinates of the CENTER of the (0,0) cell of the maze grid.
//  Ray casts are calculated accordingly.
//  "cellsize" is the edge length of each square cell.
//  "probespacing" is the spacing between llCastRay probes.
//  "height" and "radius" define the avatar's capsule. 
//  
mazerequestjson(integer sender_num, integer num, string jsn, key id) 
{   if (num != MAZESOLVEREQUEST) { return; }                // message not for us
    integer status = 0;                                     // so far, so good
    string requesttype = llJsonGetValue(jsn,["request"]);   // request type
    if (requesttype != "mazesolve") { return; }              // ignore, not our msg
    string serial = llJsonGetValue(jsn, ["serial"]); 
    integer verbose = (integer)llJsonGetValue(jsn,["verbose"]);
    vector regioncorner = (vector)llJsonGetValue(jsn,["regioncorner"]);
    gMazePos = (vector)llJsonGetValue(jsn,["pos"]);
    gMazeRot = (rotation)llJsonGetValue(jsn,["rot"]);
    gMazeCellSize = (float)llJsonGetValue(jsn,["cellsize"]);
    gMazeProbeSpacing = (float)llJsonGetValue(jsn,["probespacing"]);
    gMazeHeight = (float)llJsonGetValue(jsn,["height"]);
    gMazeWidth = (float)llJsonGetValue(jsn,["width"]);
    integer sizex = (integer)llJsonGetValue(jsn,["sizex"]);
    integer sizey = (integer)llJsonGetValue(jsn,["sizey"]);
    integer startx = (integer)llJsonGetValue(jsn,["startx"]);
    integer starty = (integer)llJsonGetValue(jsn,["starty"]);
    integer endx = (integer)llJsonGetValue(jsn,["endx"]);
    integer endy = (integer)llJsonGetValue(jsn,["endy"]);
    if (verbose) 
    {   llOwnerSay("Request to maze solver: " + jsn); }            // verbose mode
    if (sizex < 3 || sizex > MAZEMAXSIZE || sizey < 3 || sizey > MAZEMAXSIZE) { status = 2; } // too big
    list path = [];
    if (status == 0)                                    // if params sane enough to start
    {   path = mazesolve(sizex, sizey, startx, starty, endx, endy, verbose); // solve the maze
        if (llGetListLength(path) == 0)                 // failed to find a path
        {   path = [];                                  // clear path
            status = 1;
        }  
    }
    if (verbose) 
    {   llOwnerSay("Maze solver done. Free memory " + (string)llGetFreeMemory()); } 
    //  Send reply                  
    llMessageLinked(LINK_THIS, MAZESOLVERREPLY, llList2Json(JSON_OBJECT, ["reply", "mazesolve", "serial", serial, "status", status,
        "points", llList2Json(JSON_ARRAY, path)]),"");
}

//
//  mazebarrierfn -- barrier test in 3D space
//
//  ***NEED TO ADD A STRAIGHT DOWN RAYCAST TO CHECK FOR WALKABLE UNDERNEATH***
//
integer mazebarrierfn(integer prevx, integer prevy, integer x, integer y)
{   
    vector p0 = mazecelltopoint(prevx, prevy);          // centers of the start and end test cells
    vector p1 = mazecelltopoint(x,y);
    vector direction = llVecNorm(p1-p0);                // direction
    p1 = p1 + direction*(gMazeCellSize*0.5);            // extend to edge of cell
    p0 = p0 - direction*(gMazeCellSize*0.5);            // extend to edge of cell
    float dist = castbeam(p0, p1, gMazeWidth, gMazeHeight, gMazeProbeSpacing, FALSE, [RC_REJECT_TYPES,RC_REJECT_LAND]);
    return(dist != INFINITY);                           // returns true if obstacle
}
//
//  mazecelltopoint -- convert maze coordinates to point in world space
//
vector mazecelltopoint(integer x, integer y)
{      return(gMazePos + (<x,y,0>*gMazeCellSize) * gMazeRot);  }
                
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
#ifdef NOTYET   
def mazedump(route, finalroute) :
    """
    Debug dump
    """
    DEBUGPRINT("Graph && path.")
    //   Horizontal scale
    units = ""
    tens = ""
    for i in range(gMazeXsize) :
        units += str(i % 10)
        tens += str((int(i / 10)) % 10)  
    DEBUGPRINT("     " + tens)                                   
    DEBUGPRINT("     " + units)            
    DEBUGPRINT("    " + ("█" * (gMazeXsize+2)))                 // top/bottom wall
    //   Dump maze as a little picture
    //   █ - barrier
    //   • - path
    //   ◦ - examined, not on path
    //   S - start
    //   E - end
    for i in range(gMazeYsize-1,-1,-1) :
        s = ""
        for j in range(gMazeXsize) :
            barrier = testdata[j][i] & MAZEBARRIER
            examined = testdata[j][i] & MAZEEXAMINED
            ch = " "
            if examined :
                ch = "◦"
            if barrier > 0 :
                ch = "█"
            else :
                if mazepathval(j,i) in route :                    
                    ch = "•"
                if mazepathval(j,i) in finalroute :
                    ch = "◉"
            if i == gMazeStartY && j == gMazeStartX :
                ch = "S"
            if i == gMazeEndY && j == gMazeEndX : 
                ch = "E"                                            
            s = s + ch
        s = "█" + s + "█"   // show outer walls
        DEBUGPRINT("%4d%s" % (i,s))
    DEBUGPRINT("    " + ("█" * (gMazeXsize+2)))                 // top/bottom wall
    DEBUGPRINT("     " + tens)                                   
    DEBUGPRINT("     " + units)            

 


def checkreachability(xsize, ysize, xstart, ystart, xend, yend, barrierpairs) :
    """
    Check if end is reachable from start.
    This is an inefficient flood fill. Doesn't generate a route.
    But it's simple.  
    """
    barrier = numpy.full((xsize, ysize),0)                  // barrier array
    marked = numpy.full((xsize, ysize), 0)                  // marked by flood fill
    //   Mark barrier
    for (x,y) in barrierpairs :
        barrier[x][y] = 1
    //   Flood from one pixel
    def flood(x,y) :
        if (x < 0 || x >= xsize || y < 0 || y >= ysize) :
            return False                                         // off grid
        if barrier[x][y] == 0 && marked[x][y] == 0 :       // if floodable
            marked[x][y] = 1                                // mark it
            return True
    //   Flood all
    marked[xstart, ystart] = 1                              // mark start point
    changed = True
    while changed :
        changed = False                                     // something must change to continue
        for x in range(xsize) :                             // for all cells
            for y in range(ysize) :
                if marked[x][y] :
                    changed = changed || flood(x+1,y)       // flood adjacent pixels
                    changed = changed || flood(x-1,y)   
                    changed = changed || flood(x,y+1)   
                    changed = changed || flood(x,y-1) 
    //   Done flooding
    reached = marked[xend, yend]
    return reached    
                      
        
def unittestrandom1(xsize, ysize) :
    DENSITY = 0.3
    startx = random.randrange(xsize)
    starty = random.randrange(ysize)
    endx = random.randrange(xsize)
    endy = random.randrange(ysize)
    if (startx == endx && starty == endy) :
        DEBUGPRINT("Start && end at same place, skip")
        return
    barrierpairs = generaterandombarrier(xsize, ysize, startx, starty, endx, endy, int(xsize*ysize*DENSITY))
    def barrierfn(prevx, prevy, ix, iy) :   // closure for barrier test fn
        return (ix, iy) in barrierpairs
    mazeinit(xsize, ysize)
    result = mazesolve(startx, starty, endx, endy, barrierfn)
    DEBUGPRINT ("route", routeasstring(result))
    DEBUGPRINT ("cost", llGetListLength(result))
    mazedump(result,[])   
    result2 = mazeroutecornersonly(result)
    DEBUGPRINT("Corners only:" + routeasstring(result2))
    result3 = mazeoptimizeroute(result2)
    DEBUGPRINT("Optimized: " + routeasstring(result3))
    mazedump(result, result3)
    reachable = checkreachability(xsize, ysize, startx, starty, endx, endy, barrierpairs)
    pathfound = llGetListLength(result) > 0
    DEBUGPRINT("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          // fail if disagree

    
def unittestrandom(xsize, ysize, iters) :
    for n in range(iters) :
        unittestrandom1(xsize,ysize)
        DEBUGPRINT("Test %d completed." % (n,))     
     
def generaterandombarrier(xsize, ysize, startx, starty, endx, endy, cnt) :
    """
    Generate a lame random maze. Just random dots.
    """
    pts = []
    for i in range(cnt) :
        pnt = (random.randrange(xsize), random.randrange(ysize))
        if pnt == (startx,starty) || pnt == (endx, endy):       // start && end point must be free
            continue
        if not pnt in pts :
            pts.append(pnt)
    DEBUGPRINT("Random barrier: " + str(pts)) 
    DEBUGPRINT("Start, end: (%d,%d) (%d,%d) " % (startx, starty, endx, endy)) 
    return pts 
             
//   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]
BARRIERBLOCKER = [(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(10,8),(11,8)]
BARRIERCENTER = [(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(4,9),(5,9),(6,9)]

//   This one causes trouble with the termination condition
BARRIERSTUCK = [(1, 4), (5, 5), (10, 11), (3, 11), (0, 5), (9, 7), (4, 1), (5, 9), (3, 1), (6, 6), (11, 10), 
   (5, 10), (4, 9), (4, 2), (10, 8), (6, 4), (1, 7), (11, 6), (11, 9), (9, 8), (3, 9), (8, 1), (10, 4), 
   (3, 0), (2, 10), (5, 1), (7, 10), (7, 8), (6, 0), (5, 11), (2, 8), (11, 8), (6, 2), (11, 4), (10, 0), 
   (9, 3), (3, 6), (10, 5), (0, 9), (0, 10), (5, 2), (7, 11), (7, 0), (2, 2), (0, 2), (4, 8), (2, 6), 
   (6, 7), (7, 5), (4, 4), (3, 8), (1, 10), (10, 1), (3, 7), (6, 5), (4, 11), (1, 9), (9, 6), (4, 10),
   (1, 0), (10, 10), (9, 2)]

//   This one was not solved.   
BARRIERFAIL1 = [(11, 1), (8, 6), (3, 1), (10, 10), (6, 10), (10, 3), (9, 9), (5, 7), (8, 0), (3, 8),
(2, 2), (11, 4), (8, 4), (9, 4), (9, 5), (10, 11), (11, 3), (9, 11), (0, 5), (1, 7), (0, 1), (9, 10),
(6, 9), (10, 7), (1, 2), (5, 10), (5, 1), (6, 0), (10, 0), (8, 1), (5, 3), (2, 10), (0, 3), (10, 9),
(6, 4), (3, 10), (10, 5), (9, 2), (11, 0), (4, 6), (11, 5), (6, 7), (1, 9), (1, 6), (8, 10), (8, 5),
(10, 4), (8, 7), (1, 5), (4, 8), (6, 8), (3, 11), (2, 4), (7, 3), (0, 9)]

BARRIERFAIL2 = [(9, 1), (9, 3), (1, 8), (6, 5), (5, 8), (10, 11), (0, 6), (7, 10), (3, 4),
(10, 5), (1, 2), (8, 10), (6, 1), (2, 7), (3, 2), (6, 11), (0, 4), (11, 5), (2, 6), (8, 7), 
(8, 3), (2, 4), (5, 7), (4, 8), (6, 10), (1, 0), (8, 2), (5, 3), (11, 4), (5, 2), 
(2, 0), (2, 5), (9, 11), (11, 2), (4, 2), (10, 2), (9, 9), (1, 10), (9, 0), (8, 1), 
(7, 8), (1, 6), (0, 5), (10, 9), (8, 5), (0, 8), (4, 6), (4, 11), (11, 7), (10, 0), 
(5, 4), (3, 9), (4, 7), (0, 11)]

BARRIERFAIL3 =  [(5, 3), (3, 1), (11, 7), (11, 1), (10, 6), (0, 3), (9, 9), (0, 9),
(9, 3), (4, 5), (9, 1), (9, 11), (4, 0), (10, 0), (11, 3), (2, 5), (1, 11), (9, 4),
(0, 6), (7, 5), (10, 3), (3, 5), (1, 10), (9, 0), (2, 3), (8, 2), (10, 7), (1, 5),
(2, 1), (7, 7), (0, 2), (8, 1), (11, 8), (3, 9), (0, 1), (11, 4), (7, 4), (1, 4), (11, 2), (7, 2), (9, 5), (1, 7), (2, 4), (10, 9), (10, 11), (6, 7), (3, 4), (0, 5), (5, 0), (4, 7), (2, 7), (2, 11), (11, 9), (6, 4), (7, 0), (0, 10), (4, 1),
(2, 0), (9, 8)]

BARRIERFAIL4 =  [(1, 9), (11, 2), (8, 1), (4, 5), (6, 7), (7, 6), (10, 3), (7, 3),
(7, 2), (2, 9), (6, 3), (9, 5), (9, 11), (4, 0), (11, 7), (9, 6), (1, 10), (5, 7),
(7, 1), (2, 6), (11, 4), (5, 3), (4, 10), (9, 7), (3, 11), (6, 6), (1, 8), (11, 1),
(8, 9), (3, 7), (3, 10), (0, 4), (10, 4), (4, 7), (3, 2), (4, 9), (9, 8), (0, 10),
(3, 8), (4, 3), (6, 2), (10, 10), (7, 7), (5, 2), (7, 11), (0, 11), (11, 0), (2, 0),
(7, 0), (0, 5), (1, 3), (6, 1)]

BARRIERFAIL5 =  [(0, 3), (3, 11), (10, 7), (8, 7), (9, 3), (10, 8), (9, 0),
(1, 1), (3, 3), (1, 11), (2, 10), (0, 7), (2, 11), (3, 4), (2, 0), (9, 11),
(7, 3), (4, 1), (4, 6), (0, 4), (10, 5), (4, 10), (5, 8), (5, 11), (11, 8),
(6, 7), (5, 3), (7, 1), (2, 3), (4, 8), (0, 6), (6, 0), (2, 9), (6, 6),
(1, 8), (3, 2), (8, 11), (4, 0), (10, 1), (4, 2), (8, 10), (5, 1), (11, 5),
(5, 0), (5, 10), (1, 0), (4, 5), (8, 3), (11, 4), (5, 4), (3, 5), (6, 4),
(9, 8), (0, 10), (9, 1), (6, 3), (6, 8), (3, 10)]




def runtest(xsize, ysize, barrierpairs, msg) :
    def barrierfn(prevx, prevy, ix, iy) :   // closure for barrier test fn
        return (ix, iy) in barrierpairs
    DEBUGPRINT("Test name: " + msg)
    mazeinit(xsize, ysize)
    result = mazesolve(0, 0, xsize-1, ysize-1, barrierfn)
    DEBUGPRINT ("route", result)
    DEBUGPRINT ("cost", llGetListLength(result))
    mazedump(result,[])
    reachable = checkreachability(xsize, ysize, 0, 0, xsize-1, ysize-1, barrierpairs)
    pathfound = llGetListLength(result) > 0
    DEBUGPRINT("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          // fail if disagree
    assert(reachable == pathfound)          // fail if disagree
    result2 = mazeroutecornersonly(result)
    DEBUGPRINT("Corners only:" + routeasstring(result2))
    result3 = mazeoptimizeroute(result2)
    DEBUGPRINT("Optimized: " + routeasstring(result3))
    mazedump(result, result3)

    DEBUGPRINT("End test: " + msg) 
    
def test() :
    runtest(12,12,BARRIERDEF1+BARRIERCENTER, "Barrier in center")
    runtest(12,12,BARRIERDEF1+BARRIERBLOCKER, "Blocked")
    ////////return // ***TEMP***
    runtest(12,12,BARRIERSTUCK, "Barrier stuck")
    runtest(12,12,BARRIERFAIL1, "Fail 1")
    runtest(12,12,BARRIERFAIL2, "Fail 2")
    runtest(12,12,BARRIERFAIL3, "Fail 3")
    runtest(12,12,BARRIERFAIL4, "Fail 4")
    runtest(12,12,BARRIERFAIL5, "Fail 5")

    unittestrandom(41,41,1000)
   
    
 
if __name__=="__main__":
    test()
#endif // NOTYET
