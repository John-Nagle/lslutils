//
//   mazesolver.lsl
//
//   Finds reasonable path through a grid of squares with obstacles.
//
//   Intermediate step to an implementation in LSL.
//
//
//   Animats
//   June, 2019
//
//   The algorithm is from Wikipedia:
//
//   https://en.wikipedia.org/wiki/Maze_solving_algorithm//Maze-routing_algorithm
//
//   This is guaranteed to produce a path if one exists, but it may be non-optimal.
//
//   The basic approach is to head for the goal, && if there's an obstacle, follow
//   the edge of the obstacle until there's a good route to the goal.
//
//   The main problem is keeping this from looping endlessly. If it hits the
//   start point 3 times, from all possible directions, there's no solution.
//
//   Paths from this need tightening up afterwards.
// 
//
//   The data for each cell is:
//   - barrier - 1 bit, obstacle present
//   - examined - 1 bit, obstacle presence tested
//
//   These are packed into 2 bits, which are packed 16 per 32 bit word
//   (LSL being a 32-bit system), which are stored in a LSL list.
//   Timing tests indicate that the cost of updating an LSL list is constant up to size 128;
//   then it starts to increase linearly.  So a single list is good enough for anything up
//   to 45x45 cells. 
//
//   Constants
//
#define MAZEBARRIER (0x1)                                   // must be low bit
#define MAZEEXAMINED (0x2)

#define MAZEINVALIDPT (0xffffffff)                          // invalid point value in old point storage


//   Wall follow sides
#define MAZEWALLONLEFT  1
#define MAZEWALLONRIGHT (-1)
list EDGEFOLLOWDIRX = [1,0,-1,0];
list EDGEFOLLOWDIRY = [0, 1, 0, -1];

list MAZEEDGEFOLLOWDXTAB = [1,0,-1,0];
list MAZEEDGEFOLLOWDYTAB = [0,1,0,-1];
#define MAZEEDGEFOLLOWDX(n) llList2Integer(MAZEEDGEFOLLOWDXTAB,(n))
#define MAZEEDGEFOLLOWDY(n) llList2Integer(MAZEEDGEFOLLOWDYTAB,(n))

#ifdef DEBUG
#define DEBUGPRINT(s) // Nothing for now
#define DEBUGPRINT1(s) llOwnerSay(s)
////#define assert(exp) // Nothing for now
#define assert(exp) { if (!(exp)) { llOwnerSay("Assertion failed at __LINE__"); panic(); }}
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
//   Maze path storage - X && Y in one 32-bit value
//
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value

//
//  mazesolve  -- find a path through a maze
//         
list mazesolve(integer xsize, integer ysize, integer startx, integer starty, integer endx, integer endy)
{
    gMazeXsize = xsize;                          // set size of map
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
    while (gMazeX != gMazeEndX || gMazeY != gMazeEndY)  // while not at dest
    {   if (llGetListLength(gMazePath) > gMazeXsize*gMazeYsize*4) 
        {   gMazeCells = [];
            gMazePath = [];
            return([]);                    // we are in an undetected loop
        }
        if (mazeexistsproductivepath())     // if a shortcut is available
        {   mazetakeproductivepath();       // use it
            gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY);
            ////////gMazeMdbest = gMazeMdbest -1 
            assert(gMazeMdbest >= 0);
        } else {
            ////////gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)
            ////sidelr, direction = mazepickside()        // follow left || right?
            list picksideinfo = mazepickside();
            integer sidelr = llList2Integer(picksideinfo,0);
            integer direction= llList2Integer(picksideinfo,1);
            //   Inner loop - wall following
            integer followstartx = gMazeX;
            integer followstarty = gMazeY;
            integer followstartdir = direction;
            ////DEBUGPRINT("Starting wall follow at (%d,%d), direction %d, m.dist = %d" % (followstartx, followstarty, direction, gMazeMdbest))
            DEBUGPRINT1("Starting wall follow at " + (string)followstartx + "," + (string)followstarty + ",  direction " + (string)direction + ", mdist = " + (string)gMazeMdbest);
            while (mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY) >= gMazeMdbest || !mazeexistsproductivepath())
            {   ////DEBUGPRINT1("Wall follow loop");    // ***TEMP***
                if (gMazeX == gMazeEndX && gMazeY == gMazeEndY)  // if at end
                {   if (gMazePrev1 != MAZEINVALIDPT) { gMazePath += [gMazePrev1]; }
                    gMazeCells = [];                        // release memory, we need it
                    list path = gMazePath;
                    gMazePath = [];
                    return(path); 
                }                               // done
                ////DEBUGPRINT1("Calling mazefollowwall");              // ***TEMP***
                direction = mazefollowwall(sidelr, direction);      // follow edge, advance one cell
                if (llGetListLength(gMazePath) > gMazeXsize*gMazeYsize*4) // runaway check
                {   DEBUGPRINT1("***ERROR*** runaway"); 
                    gMazeCells = [];
                    gMazePath = [];
                    return([]);
                }
                //   Termination check - if we are back at the start of following && going in the same direction, no solution
                if (gMazeX == followstartx && gMazeY == followstarty && direction == followstartdir) 
                {
                    DEBUGPRINT1("Back at start of follow. No solution");
                    gMazeCells = [];
                    gMazePath = [];
                    return([]);                                  // fails
                }
            }
            DEBUGPRINT1("Finished wall following.");
        }
    }
    DEBUGPRINT1("Solved maze");
    if (gMazePrev1 != MAZEINVALIDPT) { gMazePath += [gMazePrev1]; }
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
#define MAZECOLLINEAROPT
#ifdef  MAZECOLLINEAROPT
    DEBUGPRINT1("(" + (string)gMazeX + "," + (string)gMazeY + ")");
    integer val = mazepathval(gMazeX, gMazeY);  // current point as one integer
    //  Should we keep pt 1?
    //  Yes, if it's not a duplicate or collinear.
    if ((gMazePrev1 != MAZEINVALIDPT &&                 // must have pt1 stored to keep
            ((!mazeinline(mazepathx(gMazePrev0), mazepathy(gMazePrev0),
                mazepathx(gMazePrev1), mazepathy(gMazePrev1),
                gMazeX, gMazeY)                         // not inline
                && (gMazePrev0 != gMazePrev1)           // prev1 not duplicate point
                && (gMazePrev1 != val)))))              // prev1 not duplicate point
    {   gMazePath = gMazePath + [gMazePrev1];     }     // save useful point
    gMazePrev0 = gMazePrev1;                // we keep two old points
    gMazePrev1 = val;                       // new point always becomes prev point
#else
    gMazePath = gMazePath + [mazepathval(gMazeX, gMazeY)];
    DEBUGPRINT1("(" + (string)gMazeX + "," + (string)gMazeY + ")");
#endif // MAZECOLLINEAROPT
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
    return(barrier);                            // return 1 if obstacle
}
 
//
//  mazeexistsproductivepath -- true if a productive path exists
//
//  A productive path is one that leads to the goal and isn't blocked
//     
integer mazeexistsproductivepath()
{
    integer dx = gMazeEndX - gMazeX;
    integer dy = gMazeEndY - gMazeY;
    dx = mazeclipto1(dx);
    dy = mazeclipto1(dy);
    if (dx != 0) 
    {    integer productive = !mazetestcell(gMazeX, gMazeY, gMazeX + dx, gMazeY); // test if cell in productive direction is clear
         if (productive) { return(TRUE); }
    }
    if (dy != 0) 
    {    integer productive = !mazetestcell(gMazeX, gMazeY, gMazeX, gMazeY + dy); // test if cell in productive direction is clear
         if (productive) { return(TRUE); }
    }
    return(FALSE);
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
//    Which side of the wall to follow? The one that leads toward the goal.
//    Where is the wall? One cell in the direction takkeproductvepath was going.
//
//       
list mazepickside()
{
    ////integer sidelr, direction;
    integer sidelr;
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
    //   8 cases, dumb version
    if (clippeddx == 1)                      // obstacle is in +X dir
    {   if (dy > 0)                        // if want to move in +Y
        {   direction = 1;
            sidelr = MAZEWALLONRIGHT;
        } else {
            direction = 3;
            sidelr = MAZEWALLONLEFT;
        }
    } else if (clippeddx == -1) 
    {   if (dy > 0)
        {   direction = 1;
            sidelr = MAZEWALLONLEFT;
        } else {
            direction = 3;
            sidelr = MAZEWALLONRIGHT;
        }               
    } else if (clippeddy == 1 )                  // obstacle is in +Y dir
    {   if (dx > 0)                       // if want to move in +X
        {   direction = 0;
            sidelr = MAZEWALLONLEFT;             // wall is on left            
        } else {
            direction = 2;
            sidelr = MAZEWALLONRIGHT;
        }
    } else if (clippeddy == -1)                   // obstacle is in -Y dir
    {    if (dx > 0)                        // if want to move in +X
        {    direction = 0;
            sidelr = MAZEWALLONRIGHT;                     // wall is on left
        } else {
            direction = 2;
            sidelr = MAZEWALLONLEFT;
        }
    } else {
        assert(FALSE);                       // should never get here
    }
    DEBUGPRINT("At (%d,%d) picked side %d, direction %d for wall follow." % (gMazeX, gMazeY, sidelr, direction));
    return([sidelr, direction]);
}
//
//  mazefollowwall -- Follow wall from current point. Single move per call
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
integer mazefollowwall(integer sidelr, integer direction)
{
    DEBUGPRINT1("Following wall at (" + (string)gMazeX + "," + (string)gMazeY + ")" + " side " + (string)sidelr + " direction " + (string) direction + " md " + (string)mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY));
    integer dx = MAZEEDGEFOLLOWDX(direction);
    integer dy = MAZEEDGEFOLLOWDY(direction);
    integer dxsame = MAZEEDGEFOLLOWDX(((direction + sidelr) + 4) % 4); // if not blocked ahead
    integer dysame = MAZEEDGEFOLLOWDY(((direction + sidelr) + 4) % 4); 
    integer followedside = mazetestcell(gMazeX, gMazeY, gMazeX + dxsame, gMazeY+dysame);
    if (!followedside) 
    {
        DEBUGPRINT("***ERROR*** followedside not blocked. dx,dy: (%d,%d)  dxsame,dysame: (%d,%d) sidelr %d direction %d" %
                (dx,dy, dxsame,dysame, sidelr,direction));
        assert(followedside);                            // must be next to obstacle
    }
    integer blockedahead = mazetestcell(gMazeX, gMazeY, gMazeX + dx, gMazeY + dy);
    if (blockedahead)
    {   integer dxopposite = MAZEEDGEFOLLOWDX(((direction - sidelr) + 4) % 4);
        integer dyopposite = MAZEEDGEFOLLOWDY(((direction - sidelr) + 4) % 4);
        integer blockedopposite = mazetestcell(gMazeX, gMazeY, gMazeX + dxopposite, gMazeY + dyopposite);
        if (blockedopposite) 
        {   DEBUGPRINT("Dead end");
            direction = (direction + 2) % 4;         // dead end, reverse direction
        } else {
            DEBUGPRINT("Inside corner");
            direction = (direction - sidelr + 4) % 4;      // inside corner, turn
        }
    } else {
        assert(dxsame == 0 || dysame == 0);
        integer blockedsameahead = mazetestcell(gMazeX + dx, gMazeY + dy, gMazeX + dx + dxsame, gMazeY + dy + dysame);
        if (blockedsameahead)                       // straight, not outside corner
        {   DEBUGPRINT("Straight");
            gMazeX += dx;                            // move ahead 1
            gMazeY += dy;
            mazeaddtopath();
        } else {                                     // outside corner
            DEBUGPRINT("Outside corner");
            gMazeX += dx;                            // move ahead 1
            gMazeY += dy;
            mazeaddtopath();
            //   Need to check for a productive path. May be time to stop wall following
            integer md = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY);
            if (md < gMazeMdbest && mazeexistsproductivepath())
            {
                DEBUGPRINT("Outside corner led to a productive path halfway through")
                return(direction);
            }
            direction = (direction + sidelr + 4) % 4;    // turn in direction
            gMazeX += dxsame;                        // move around corner
            gMazeY += dysame;
            mazeaddtopath();
        }
    }
    return(direction);                                // new direction
} 

#ifdef OBSOLETE   // mazeoptimizeroute now does this
    
def mazeroutecornersonly(route) :
    """
    Condense route, only keeping corners
    """
    if (llGetListLength(route) == 0) :                          // empty
        return(route)
    newroute = []
    prev0x = -1
    prev0y = -1
    prev1x = -1
    prev1y = -1
    x = -1
    y = -1
    for n in range(llGetListLength(route)) :
        val = route[n]
        x = mazepathx(val)
        y = mazepathy(val)
        ////////x = route[n][0]
        ////////y = route[n][1]
        if (prev0x >= 0 && (mazeinline(prev0x, prev0y, prev1x, prev1y, x, y) 
            || mazepointssame(prev0x, prev0y, prev1x, prev1y) 
            || mazepointssame(prev1x, prev1y, x,y))) :
            pass
                //   pt 1 is redundant
        else :                                  // need to keep pt 1
            prev0x = prev1x
            prev0y = prev1y
            if prev1x >= 0 :                    // if we have something to output
                newroute.append(mazepathval(prev1x, prev1y))
        prev1x = x
        prev1y = y
    // final point.
    newroute.append(mazepathval(x,y)) 
    return newroute
#endif // OBSOLETE    
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
