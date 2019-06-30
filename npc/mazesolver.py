#
#   mazesolver.py
#
#   Finds reasonable path through a grid of squares with obstacles.
#
#   Intermediate step to an implementation in LSL.
#
#
#   Animats
#   June, 2019
#
#   The algorithm is from Wikipedia:
#
#   https://en.wikipedia.org/wiki/Maze_solving_algorithm#Maze-routing_algorithm
#
#   This is guaranteed to produce a path if one exists, but it may be non-optimal.
#
#   The basic approach is to head for the goal, and if there's an obstacle, follow
#   the edge of the obstacle until there's a good route to the goal.
#
#   The main problem is keeping this from looping endlessly. If it hits the
#   start point 3 times, from all possible directions, there's no solution.
#
#   Paths from this need tightening up afterwards.
# 
#
#   The data for each cell is:
#   - barrier - 1 bit, obstacle present
#   - examined - 1 bit, obstacle presence tested
#
#   These are packed into 2 bits, which are packed 16 per 32 bit word
#   (LSL being a 32-bit system), which are stored in a LSL list.
#   Timing tests indicate that the cost of updating an LSL list is constant up to size 128;
#   then it starts to increase linearly.  So a single list is good enough for anything up
#   to 45x45 cells. 
#
#   BUGS:
#
import numpy
import math
import random
#   Constants
MAZEBARRIER = 0x1                                   # must be low bit
MAZEEXAMINED = 0x2

EDGEFOLLOWDIRS = [(1,0), (0, 1), (-1, 0), (0, -1)]  # edge following dirs to dx and dy

MAZEEDGEFOLLOWDX = [1,0,-1,0]
MAZEEDGEFOLLOWDY = [0,1,0,-1]

#   Wall follow sides
MAZEWALLONLEFT = 1
MAZEWALLONRIGHT = -1

#
#   mazemd -- rectangular "Manhattan" distance
#
def mazemd(p0x, p0y, p1x, p1y) :
    return abs(p1x-p0x) + abs(p1y-p0y)
    
#
#   mazeclipto1 -- clip to range -1, 1
#
def mazeclipto1(n) :
    if n > 0 :
        return 1
    elif n < 0 :
        return -1
    return 0
#
#   mazeinline
#
def mazeinline(x0,y0,x1,y1,x2,y2) :
    return ((x0 == x1 and x1 == x2) or
       (y0 == y1 and y1 == y2))
       
#
#   mazepointssame
#
def mazepointssame(x0,y0,x1,y1) :
    return x0 == x1 and y0 == y1
#
#   listreplacelist
#
def listreplacelist(src, dst, start, end) :
    """
    LSL list update function
    """
    assert(start >= 0)                          # no funny end-relative stuff
    assert(end >= 0)
    return src[0:start] + dst + src[end+1:]     # LSL compatibility
#
#
#   Mazegraph
#
#   Globals for LSL
#

gMazePath = []
gMazeCells = []                     # maze cell bits, see mazecellget
gMazeX = -1
gMazeY = -1
gMazeMdbest = -1


gMazeXsize = -1
gMazeYsize = -1
gMazeStartX = -1 
gMazeStartY = -1 
gMazeEndX = -1 
gMazeEndY = -1
#
#   Python only
gBarrierFn = None
testdata = None




#
#   Maze graph functions
#

#
#   Maze cell storage - 2 bits per cell
#
def mazecellget(x,y) :
    """
    Get from 2D maze array
    """
    assert(x >= 0 and x < gMazeXsize)           # subscript check
    assert(y >= 0 and y < gMazeYsize)
    cellix = y*gMazeYsize + x                   # index into cells
    listix = int(cellix / 16)
    bitix = (cellix % 16) * 2
    return (gMazeCells[listix] >> bitix) & 0x3  # 2 bits only
    
def mazecellset(x,y, newval) :
    """
    Store into 2D maze array
    """
    global gMazeCells
    assert(x >= 0 and x < gMazeXsize)           # subscript check
    assert(y >= 0 and y < gMazeYsize)
    assert(newval <= 0x3)                       # only 2 bits
    cellix = y*gMazeYsize + x                   # index into cells
    listix = int(cellix / 16)                   # word index
    bitix = (cellix % 16) * 2                   # bit index within word
    w = gMazeCells[listix]
    w = (w & (~(0x3<<bitix)))| (newval<<bitix)  # insert into word
    gMazeCells[listix] = w                      # insert word
    while len(gMazeCells) < listix :            # fill out list as needed
        gMazeCells.append(0)
   
def mazeinit(xsize, ysize) :
    global gMazeXsize, gMazeYsize, gMazeCells
    gMazeXsize = xsize                      # set size of map
    gMazeYsize = ysize
    gMazeCells = []
    while len(gMazeCells) < xsize*ysize :   # allocate list
        gMazeCells.append(0)
    global testdata                         # Python only
    testdata = numpy.full((xsize, ysize), 0)
       
def mazesolve(startx, starty, endx, endy, barrierfn) :
    global gMazeX, gMazeY, gMazeStartX, gMazeStartY, gMazeEndX, gMazeEndY, gMazeMdbest, gMazePath
    global gBarrierFn                       # Python only
    gMazeX = startx                         # start
    gMazeY = starty
    gBarrierFn = barrierfn                   # tests cell for blocked
    gMazeStartX = startx                    # start
    gMazeStartY = starty
    gMazeEndX = endx                        # destination
    gMazeEndY = endy
    gMazeMdbest = gMazeXsize+gMazeYsize+1   # best dist to target init
    gMazePath = []                          # accumulated path
    mazeaddtopath()                        # add initial point
    #   Outer loop - shortcuts or wall following
    while (gMazeX != gMazeEndX or gMazeY != gMazeEndY) : # while not at dest
        if (len(gMazePath) > gMazeXsize*gMazeYsize*4) :
            return []                       # we are in an undetected loop
        if (mazeexistsproductivepath()) :  # if a shortcut is available
            mazetakeproductivepath()       # use it
            gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)
            ####gMazeMdbest = gMazeMdbest -1 
            assert(gMazeMdbest >= 0)
        else :
            ####gMazeMdbest = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)
            sidelr, direction = mazepickside()        # follow left or right?
            #   Inner loop - wall following
            followstartx = gMazeX
            followstarty = gMazeY
            followstartdir = direction
            print("Starting wall follow at (%d,%d), direction %d, m.dist = %d" % (followstartx, followstarty, direction, gMazeMdbest))
            while mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY) >= gMazeMdbest or not mazeexistsproductivepath() :
                if (gMazeX == gMazeEndX and gMazeY == gMazeEndY) : # if at end
                    return gMazePath                               # done
                direction = mazefollowwall(sidelr, direction)      # follow edge, advance one cell
                if len(gMazePath) > gMazeXsize*gMazeYsize*4 : # runaway check
                    print("***ERROR*** runaway: " + str(gMazePath)) 
                    return []
                #   Termination check - if we are back at the start of following and going in the same direction, no solution
                if (gMazeX == followstartx and gMazeY == followstarty and direction == followstartdir) :
                    print("Back at start of follow. Stuck")
                    return []                                   # fails
            print("Finished wall following.")
    print("Solved maze")                
    return(gMazePath)
                    
def mazeaddtopath() :
    """
    Add current position to path
    """
    global gMazePath
    gMazePath += [(gMazeX, gMazeY)]
    print("(%d,%d)" % (gMazeX, gMazeY))
    ####assert(not mazetestcell(gMazeX, gMazeY, gMazeX + dx, gMazeY + dy)) # path must not go into an occupied cell

                                                     
def mazetestcell(fromx, fromy, x, y) :
    """
    Returns 1 if occupied cell.
    Makes expensive cast ray tests the first time a cell is checked.
    """
    print("Testcell (%d,%d)" % (x,y))       # ***TEMP***
    if (x < 0 or x >= gMazeXsize or y < 0 or y >= gMazeYsize) : # if off grid
        return 1                            # treat as occupied
    v = mazecellget(x,y)
    assert(v == testdata[x][y])             # this cell
    if (v & MAZEEXAMINED) :
        return v & MAZEBARRIER              # already have this one
    barrier = gBarrierFn(fromx, fromy, x,y) # check this location
    v = MAZEEXAMINED | barrier
    mazecellset(x,y,v)                      # update cells checked
    testdata[x][y] = v                      # update sites checked
    return barrier                          # return 1 if obstacle
        
def mazeexistsproductivepath() :
    """
    True if a productive path exists
    """
    dx = gMazeEndX - gMazeX
    dy = gMazeEndY - gMazeY
    dx = mazeclipto1(dx)
    dy = mazeclipto1(dy)
    if (dx != 0) :
        productive = not mazetestcell(gMazeX, gMazeY, gMazeX + dx, gMazeY) # test if cell in productive direction is clear
        if productive :
            print("Productive path at (%d,%d): %d" % (gMazeX, gMazeY, productive))
            return True
    if (dy != 0) :
        productive = not mazetestcell(gMazeX, gMazeY, gMazeX, gMazeY + dy) # test if cell in productive direction is clear
        if productive :
            print("Productive path at (%d,%d): %d" % (gMazeX, gMazeY, productive))
            return True
    return False
         
def mazetakeproductivepath() :
    """
    Follow productive path or return 0       
    """
    global gMazeX, gMazeY

    dx = gMazeEndX - gMazeX
    dy = gMazeEndY - gMazeY
    clippeddx = mazeclipto1(dx)
    clippeddy = mazeclipto1(dy)
    assert(dx != 0 or dy != 0)              # error to call this at dest
    #    Try X dir first if more direct towards goal
    if abs(dx) > abs(dy) and clippeddx :
        if not mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY) :
            gMazeX += clippeddx                       # advance in desired dir
            mazeaddtopath()
            return 1
    #   Then try Y    
    if clippeddy :
        if not mazetestcell(gMazeX, gMazeY, gMazeX, gMazeY + clippeddy) :
            gMazeY += clippeddy                       # advance in desired dir
            mazeaddtopath()
            return 1 
    #   Then X, regardless of whether abs(dx) > abs(dy)
    if clippeddx :
        if not mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY) :
            gMazeX += clippeddx                       # advance in desired dir
            mazeaddtopath()
            return 1    
                               # success
    print("Take productive path failed")
    return 0                                        # hit wall, stop
        
def mazepickside() :
    """
    Which side of the wall to follow? The one that leads toward
    the goal.
    Where is the wall? One cell in the direction takkeproductvepath was
    going.
    """
    dx = gMazeEndX - gMazeX
    dy = gMazeEndY - gMazeY
    assert(dx != 0 or dy != 0)              # error to call this at dest
    clippeddx = mazeclipto1(dx)
    clippeddy = mazeclipto1(dy)
    if abs(dx) > abs(dy) :                  # better to move in X
        clippeddy = 0 
    else :
        clippeddx = 0
    assert(mazetestcell(gMazeX, gMazeY, gMazeX + clippeddx, gMazeY + clippeddy)) # must have hit a wall
    #   8 cases, dumb version
    if clippeddx == 1 :                     # obstacle is in +X dir
        if (dy > 0) :                       # if want to move in +Y
            direction = 1
            sidelr = MAZEWALLONRIGHT
        else :
            direction = 3
            sidelr = MAZEWALLONLEFT
    elif clippeddx == -1 :
        if (dy > 0) :
            direction = 1
            sidelr = MAZEWALLONLEFT
        else :
            direction = 3
            sidelr = MAZEWALLONRIGHT                
    elif clippeddy == 1 :                   # obstacle is in +Y dir
        if (dx > 0) :                       # if want to move in +X
            direction = 0
            sidelr = MAZEWALLONLEFT             # wall is on left
        else :
            direction = 2
            sidelr = MAZEWALLONRIGHT
    elif clippeddy == -1 :                  # obstacle is in -Y dir
        if (dx > 0) :                       # if want to move in +X
            direction = 0
            sidelr = MAZEWALLONRIGHT                     # wall is on left
        else :
            direction = 2
            sidelr = MAZEWALLONLEFT
    else :
        assert(False)                       # should never get here
    print("At (%d,%d) picked side %d, direction %d for wall follow." % (gMazeX, gMazeY, sidelr, direction))
    return (sidelr, direction)
        
def mazefollowwall(sidelr, direction) :
    """
    Follow wall from current point. Single move per call
        
    Wall following rules:
    Always blocked on follow side. Algorithm error if not.
        
    If blocked ahead and not blocked opposite follow side, inside corner
            turn away from follow side. No move.
    If blocked ahead and blocked opposite follow side, dead end
            turn twice to reverse direction, no move.
    If not blocked ahead and blocked on follow side 1 ahead, 
            advance straight.
    If not blocked ahead and not blocked on follow side 1 ahead, outside corner,
            advance straight, 
            turn towards follow side, 
            advance straight.
            
    "sidelr" is 1 for left, -1 for right
    "direction" is 0 for +X, 1 for +Y, 2 for -X, 3 for -Y

    """
    global gMazeX, gMazeY
    print("Following wall at (%d,%d) side %d direction %d md %d" % 
            (gMazeX, gMazeY, sidelr, direction, mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)))
    dx = MAZEEDGEFOLLOWDX[direction]
    dy = MAZEEDGEFOLLOWDY[direction]
    dxsame = MAZEEDGEFOLLOWDX[((direction + sidelr) + 4) % 4] # if not blocked ahead
    dysame = MAZEEDGEFOLLOWDY[((direction + sidelr) + 4) % 4] 
    followedside = mazetestcell(gMazeX, gMazeY, gMazeX + dxsame, gMazeY+dysame)
    if (not followedside) :
        print("***ERROR*** followedside not blocked. dx,dy: (%d,%d)  dxsame,dysame: (%d,%d) sidelr %d direction %d" %
                (dx,dy, dxsame,dysame, sidelr,direction))
        assert(followedside)                            # must be next to obstacle
    blockedahead = mazetestcell(gMazeX, gMazeY, gMazeX + dx, gMazeY + dy)
    if blockedahead :
        dxopposite = MAZEEDGEFOLLOWDX[((direction - sidelr) + 4) % 4]
        dyopposite = MAZEEDGEFOLLOWDY[((direction - sidelr) + 4) % 4]
        blockedopposite = mazetestcell(gMazeX, gMazeY, gMazeX + dxopposite, gMazeY + dyopposite)
        if blockedopposite :
            print("Dead end")
            direction = (direction + 2) % 4         # dead end, reverse direction
        else :
            print("Inside corner")
            direction = (direction - sidelr + 4) % 4      # inside corner, turn
    else :
        assert(dxsame == 0 or dysame == 0)
        blockedsameahead = mazetestcell(gMazeX + dx, gMazeY + dy, gMazeX + dx + dxsame, gMazeY + dy + dysame);
        if blockedsameahead :                       # straight, not outside corner
            print("Straight")
            gMazeX += dx                            # move ahead 1
            gMazeY += dy
            mazeaddtopath()
        else :                                      # outside corner
            print("Outside corner")
            gMazeX += dx                            # move ahead 1
            gMazeY += dy
            mazeaddtopath()
            #   Need to check for a productive path. May be time to stop wall following
            md = mazemd(gMazeX, gMazeY, gMazeEndX, gMazeEndY)
            if md < gMazeMdbest and mazeexistsproductivepath() :
                print("Outside corner led to a productive path halfway through")
                return direction
            direction = (direction + sidelr + 4) % 4    # turn in direction
            gMazeX += dxsame                        # move around corner
            gMazeY += dysame
            mazeaddtopath() 
    return direction                                # new direction
        
def mazeroutecornersonly(route) :
    """
    Condense route, only keeping corners
    """
    newroute = []
    prev0x = -1
    prev0y = -1
    prev1x = -1
    prev1y = -1
    x = -1
    y = -1
    for n in range(len(route)) :
        x = route[n][0]
        y = route[n][1]
        if (prev0x >= 0 and (mazeinline(prev0x, prev0y, prev1x, prev1y, x, y) 
            or mazepointssame(prev0x, prev0y, prev1x, prev1y) 
            or mazepointssame(prev1x, prev1y, x,y))) :
            pass
                #   pt 1 is redundant
        else :                                  # need to keep pt 1
            prev0x = prev1x
            prev0y = prev1y
            if prev1x >= 0 :                    # if we have something to output
                newroute.append((prev1x, prev1y))
        prev1x = x
        prev1y = y
    # final point.
    newroute.append((x,y)) 
    return newroute
        
def mazelinebarrier(x0, y0, x1, y1) :
    """
    Does the line between the two points, inclusive, hit a barrier?
    """
    print("Maze test barrier: (%d,%d),(%d,%d)" % (x0,y0,x1,y1))
    if (x0 == x1) :                         # vertical line
        assert(y0 != y1)                    # must not be zero length
        if y0 > y1 :                        # sort
            temp = y0
            y0 = y1
            y1 = temp
        assert(y1 > y0)
        for y in range(y0,y1) :             # test each segment
            if mazetestcell(x0, y, x0, y+1) :
                return True                # hit barrier
        return False
    else :
        assert(y0 == y1)
        assert(x0 != x1)
        if x0 > x1 :                        # sort
            temp = x0
            x0 = x1
            x1 = temp
        assert(x1 > x0)
        for x in range(x0,x1) :
            if mazetestcell(x, y0, x+1, y0) :
                return True                # hit barrier
        return False
         
           
        
def mazeoptimizeroute(route) :
    """
    Locally optimize route.
        
    The incoming route should have corners only, and represent only horizontal and vertical lines.
    Optimizing the route looks at groups of 4 points. If the two turns are both the same, then try
    to eliminate one of the points by moving the line between the two middle points.
    
    O(n)        
    """
    n = 0;
    #   Advance throug route. On each iteration, either the route gets shorter, or n gets
    #   larger, so this should always terminate.
    while n < len(route)-3 :                        # advancing through route
        print("%d: %s %s %s %s" % (n, route[n], route[n+1], route[n+2], route[n+3])) # ***TEMP***
        p0x = route[n][0]
        p0y = route[n][1]
        p1x = route[n+1][0]
        p1y = route[n+1][1]
        p2x = route[n+2][0]
        p2y = route[n+2][1]
        p3x = route[n+3][0]
        p3y = route[n+3][1]
        #   Remove collinear redundant points. The redundant point may not be
        #   between the endpoints, but that's OK. It's just removing a move to
        #   a dead end and back.
        if (p0x == p1x and p0y == p1y) :            # redundant point
            ####print("Removing redundant point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+1, n+1)
            if n > 0 :                              # back up 1, may have created new redundant group
                n = n - 1
            continue
        if (p1x == p2x and p1y == p2y) :            # redundant point
            ####print("Removing redundant point %d from %s" % (n+2, str(route)))
            route = listreplacelist(route, [], n+2, n+2)
            if n > 0 :
                n = n - 1
            continue
        if mazeinline(p0x,p0y,p1x,p1y,p2x,p2y) :
            ####print("Removing collinear point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+1, n+1)
            if n > 0 :
                n = n - 1
            continue
        if mazeinline(p1x,p1y,p2x,p2y,p3x,p3y) :
            ####print("Removing collinear point %d from %s" % (n+1, str(route)))
            route = listreplacelist(route, [], n+2, n+2)
            if n > 0 :
                n = n - 1
            continue                
        if (p1x == p2x) :                           # if vertical middle segment
            #   End segments must be horizontal
            assert(p0y == p1y)
            assert(p2y == p3y)
            #   Is this C-shaped?
            if not ((p0x > p1x) == (p2x < p3x)) :   # no, not C-shaped
                n = n + 1
                continue
            #   Find shorter arm of C
            armlena = p0x-p1x 
            armlenb = p3x-p2x
            if abs(armlena) > abs(armlenb) :        # second arm is shorter
                #   We will try to move middle segment to align with p0y, ignoring p1y
                if mazelinebarrier(p3x, p0y, p3x, p3y) : # if blocked
                    n = n + 1
                    continue
                #   We can get rid of p1 and replace p2
                route = listreplacelist(route, [(p3x,p0y)], n+1, n+2) # remove p1
                print("Vertical middle segment shortened at p1: %d: (%d,%d)" % (n+1,p3x,p0y))
                continue
            else :
                #   We will try to move middle segment to align with p3y, ignoring p2y
                if mazelinebarrier(p0x, p0y, p0x, p3y) : # if blocked
                    n = n + 1
                    continue
                #   We can get rid of p2 and replace p1
                route = listreplacelist(route, [(p0x, p3y)], n+1, n+2) # remove p2
                print("Vertical middle segment shortened at p2: %d: (%d,%d)" % (n+1,p0x,p3y))
                continue                       
                    
        else :                                      # if horizontal middle segment
            assert(p1y == p2y)
            #   End segments must be vertical
            assert(p0x == p1x)
            assert(p2x == p3x)
            #   Is this C-shaped?
            if not ((p0y > p1y) == (p2y < p3y)) :   # no, not C-shaped
                n = n + 1
                continue
            #   Find shorter arm of C
            armlena = p0y-p1y 
            armlenb = p3y-p2y
            if abs(armlena) > abs(armlenb) :        # second arm is shorter
                #   We will try to move middle segment to align with p3y
                if mazelinebarrier(p0x, p3y, p3x, p3y) : # if blocked
                    n = n + 1
                    continue
                #   We can get rid of p1 and p2 and replace with new point
                route = listreplacelist(route, [(p1x, p3y)], n+1, n+2) # replace p1 and p2
                print("Horizontal middle segment shortened at p1: %d: (%d,%d)" % (n+1,p1x,p3y))
                continue
            else :
                #   We will try to move middle segment to align with p0y
                if mazelinebarrier(p0x, p0y, p3x, p0y) : # if blocked
                    n = n + 1
                    continue
                #   We can get rid of p1 and p2 and replace with new point
                route = listreplacelist(route, [(p2x,p0y)], n+1, n+2) # replace p1 and p2 with new point
                print("Horizontal middle segment shortened at p2: %d: (%d,%d)" % (n+1,p2x,p0y))
                continue 
    return route                                    # condensed route                      

            
#
#   Test-only code
#        
def mazedump(route, finalroute) :
    """
    Debug dump
    """
    print("Graph and path.")
    #   Horizontal scale
    units = ""
    tens = ""
    for i in range(gMazeXsize) :
        units += str(i % 10)
        tens += str((int(i / 10)) % 10)  
    print("     " + tens)                                   
    print("     " + units)            
    print("    " + ("█" * (gMazeXsize+2)))                 # top/bottom wall
    #   Dump maze as a little picture
    #   █ - barrier
    #   • - path
    #   ◦ - examined, not on path
    #   S - start
    #   E - end
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
                if (j,i) in route :                    
                    ch = "•"
                if (j,i) in finalroute :
                    ch = "◉"
            if i == gMazeStartY and j == gMazeStartX :
                ch = "S"
            if i == gMazeEndY and j == gMazeEndX : 
                ch = "E"                                            
            s = s + ch
        s = "█" + s + "█"   # show outer walls
        print("%4d%s" % (i,s))
    print("    " + ("█" * (gMazeXsize+2)))                 # top/bottom wall
    print("     " + tens)                                   
    print("     " + units)            

 


def checkreachability(xsize, ysize, xstart, ystart, xend, yend, barrierpairs) :
    """
    Check if end is reachable from start.
    This is an inefficient flood fill. Doesn't generate a route.
    But it's simple.  
    """
    barrier = numpy.full((xsize, ysize),0)                  # barrier array
    marked = numpy.full((xsize, ysize), 0)                  # marked by flood fill
    #   Mark barrier
    for (x,y) in barrierpairs :
        barrier[x][y] = 1
    #   Flood from one pixel
    def flood(x,y) :
        if (x < 0 or x >= xsize or y < 0 or y >= ysize) :
            return False                                         # off grid
        if barrier[x][y] == 0 and marked[x][y] == 0 :       # if floodable
            marked[x][y] = 1                                # mark it
            return True
    #   Flood all
    marked[xstart, ystart] = 1                              # mark start point
    changed = True
    while changed :
        changed = False                                     # something must change to continue
        for x in range(xsize) :                             # for all cells
            for y in range(ysize) :
                if marked[x][y] :
                    changed = changed or flood(x+1,y)       # flood adjacent pixels
                    changed = changed or flood(x-1,y)   
                    changed = changed or flood(x,y+1)   
                    changed = changed or flood(x,y-1) 
    #   Done flooding
    reached = marked[xend, yend]
    return reached    
                      
        
def unittestrandom1(xsize, ysize) :
    DENSITY = 0.3
    startx = random.randrange(xsize)
    starty = random.randrange(ysize)
    endx = random.randrange(xsize)
    endy = random.randrange(ysize)
    if (startx == endx and starty == endy) :
        print("Start and end at same place, skip")
        return
    barrierpairs = generaterandombarrier(xsize, ysize, startx, starty, endx, endy, int(xsize*ysize*DENSITY))
    def barrierfn(prevx, prevy, ix, iy) :   # closure for barrier test fn
        return (ix, iy) in barrierpairs
    mazeinit(xsize, ysize)
    result = mazesolve(startx, starty, endx, endy, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    mazedump(result,[])   
    reachable = checkreachability(xsize, ysize, startx, starty, endx, endy, barrierpairs)
    pathfound = len(result) > 0
    print("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          # fail if disagree
    result2 = mazeroutecornersonly(result)
    print("Corners only:" + str(result2))
    result3 = mazeoptimizeroute(result2)
    print("Optimized: " + str(result3))
    mazedump(result, result3)
    
def unittestrandom(xsize, ysize, iters) :
    for n in range(iters) :
        unittestrandom1(xsize,ysize)
        print("Test %d completed." % (n,))     
     
def generaterandombarrier(xsize, ysize, startx, starty, endx, endy, cnt) :
    """
    Generate a lame random maze. Just random dots.
    """
    pts = []
    for i in range(cnt) :
        pnt = (random.randrange(xsize), random.randrange(ysize))
        if pnt == (startx,starty) or pnt == (endx, endy):       # start and end point must be free
            continue
        if not pnt in pts :
            pts.append(pnt)
    print("Random barrier: " + str(pts)) 
    print("Start, end: (%d,%d) (%d,%d) " % (startx, starty, endx, endy)) 
    return pts 
             
#   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]
BARRIERBLOCKER = [(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(10,8),(11,8)]
BARRIERCENTER = [(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(4,9),(5,9),(6,9)]

#   This one causes trouble with the termination condition
BARRIERSTUCK = [(1, 4), (5, 5), (10, 11), (3, 11), (0, 5), (9, 7), (4, 1), (5, 9), (3, 1), (6, 6), (11, 10), 
   (5, 10), (4, 9), (4, 2), (10, 8), (6, 4), (1, 7), (11, 6), (11, 9), (9, 8), (3, 9), (8, 1), (10, 4), 
   (3, 0), (2, 10), (5, 1), (7, 10), (7, 8), (6, 0), (5, 11), (2, 8), (11, 8), (6, 2), (11, 4), (10, 0), 
   (9, 3), (3, 6), (10, 5), (0, 9), (0, 10), (5, 2), (7, 11), (7, 0), (2, 2), (0, 2), (4, 8), (2, 6), 
   (6, 7), (7, 5), (4, 4), (3, 8), (1, 10), (10, 1), (3, 7), (6, 5), (4, 11), (1, 9), (9, 6), (4, 10),
   (1, 0), (10, 10), (9, 2)]

#   This one was not solved.   
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
    def barrierfn(prevx, prevy, ix, iy) :   # closure for barrier test fn
        return (ix, iy) in barrierpairs
    print("Test name: " + msg)
    mazeinit(xsize, ysize)
    result = mazesolve(0, 0, xsize-1, ysize-1, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    mazedump(result,[])
    reachable = checkreachability(xsize, ysize, 0, 0, xsize-1, ysize-1, barrierpairs)
    pathfound = len(result) > 0
    print("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          # fail if disagree
    assert(reachable == pathfound)          # fail if disagree
    result2 = mazeroutecornersonly(result)
    print("Corners only:" + str(result2))
    result3 = mazeoptimizeroute(result2)
    print("Optimized: " + str(result3))
    mazedump(result, result3)

    print("End test: " + msg) 
    
def test() :
    runtest(12,12,BARRIERDEF1+BARRIERCENTER, "Barrier in center")
    runtest(12,12,BARRIERDEF1+BARRIERBLOCKER, "Blocked")
    ####return # ***TEMP***
    runtest(12,12,BARRIERSTUCK, "Barrier stuck")
    runtest(12,12,BARRIERFAIL1, "Fail 1")
    runtest(12,12,BARRIERFAIL2, "Fail 2")
    runtest(12,12,BARRIERFAIL3, "Fail 3")
    runtest(12,12,BARRIERFAIL4, "Fail 4")
    runtest(12,12,BARRIERFAIL5, "Fail 5")

    unittestrandom(41,41,1)
   
    
 
if __name__=="__main__":
    test()

