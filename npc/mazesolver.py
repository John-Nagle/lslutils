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
    return src[0:start] + dst + src[end+1:]   
#
#
#   Mazegraph
#
#   This will have to be done with globals in LSL
#
class Mazegraph(object):
   
    def __init__(self, xsize, ysize) :
        self.gMazeXsize = xsize                                          # set size of map
        self.gMazeYsize = ysize
        self.testdata = numpy.full((xsize, ysize), 0)
       
    def solvemaze(self, startx, starty, endx, endy, barrierfn) :
        self.gMazeX = startx                         # start
        self.gMazeY = starty
        self.barrierfn = barrierfn              # tests cell for blocked
        self.gMazeStartX = startx                    # start
        self.gMazeStartY = starty
        self.gMazeEndX = endx                        # destination
        self.gMazeEndY = endy
        self.gMazeMdbest = self.gMazeXsize+self.gMazeYsize+1   # best dist to target init
        self.gMazePath = []                          # accumulated path
        self.mazeaddtopath()                        # add initial point
        #   Outer loop - shortcuts or wall following
        while (self.gMazeX != self.gMazeEndX or self.gMazeY != self.gMazeEndY) : # while not at dest
            if (len(self.gMazePath) > self.gMazeXsize*self.gMazeYsize*4) :
                return []                       # we are in an undetected loop
            if (self.mazeexistsproductivepath()) :  # if a shortcut is available
                self.mazetakeproductivepath()       # use it
                self.gMazeMdbest = mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY)
                ####self.gMazeMdbest = self.gMazeMdbest -1 
                assert(self.gMazeMdbest >= 0)
            else :
                ####self.gMazeMdbest = mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY)
                sidelr, direction = self.mazepickside()        # follow left or right?
                #   Inner loop - wall following
                followstartx = self.gMazeX
                followstarty = self.gMazeY
                followstartdir = direction
                print("Starting wall follow at (%d,%d), direction %d, m.dist = %d" % (followstartx, followstarty, direction, self.gMazeMdbest))
                while mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY) >= self.gMazeMdbest or not self.mazeexistsproductivepath() :
                    if (self.gMazeX == self.gMazeEndX and self.gMazeY == self.gMazeEndY) : # if at end
                        return self.gMazePath                               # done
                    direction = self.mazefollowwall(sidelr, direction)      # follow edge, advance one cell
                    if len(self.gMazePath) > self.gMazeXsize*self.gMazeYsize*4 : # runaway check
                        print("***ERROR*** runaway: " + str(self.gMazePath)) 
                        return []
                    #   Termination check - if we are back at the start of following and going in the same direction, no solution
                    if (self.gMazeX == followstartx and self.gMazeY == followstarty and direction == followstartdir) :
                        print("Back at start of follow. Stuck")
                        return []                                   # fails
                print("Finished wall following.")
                        
        return(self.gMazePath)
                    
    def mazeaddtopath(self) :
        """
        Add current position to path
        """
        self.gMazePath += [(self.gMazeX, self.gMazeY)]
        print("(%d,%d)" % (self.gMazeX, self.gMazeY))
        ####assert(not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY + dy)) # path must not go into an occupied cell

                                                     
    def mazetestcell(self, fromx, fromy, x, y) :
        """
        Returns 1 if occupied cell
        """
        print("Testcell (%d,%d)" % (x,y))       # ***TEMP***
        if (x < 0 or x >= self.gMazeXsize or y < 0 or y >= self.gMazeYsize) : # if off grid
            return 1                            # treat as occupied
        v = self.testdata[x][y]                 # this cell
        if (v & MAZEEXAMINED) :
            print
            return v & MAZEBARRIER                  # already have this one
        barrier = self.barrierfn(fromx, fromy, x,y)             # check this location
        v = MAZEEXAMINED | barrier
        self.testdata[x][y] = v                 # update sites checked
        return barrier                          # return 1 if obstacle
        
    def mazeexistsproductivepath(self) :
        """
        True if a productive path exists
        """
        dx = self.gMazeEndX - self.gMazeX
        dy = self.gMazeEndY - self.gMazeY
        dx = mazeclipto1(dx)
        dy = mazeclipto1(dy)
        if (dx != 0) :
            productive = not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY) # test if cell in productive direction is clear
            if productive :
                print("Productive path at (%d,%d): %d" % (self.gMazeX, self.gMazeY, productive))
                return True
        if (dy != 0) :
            productive = not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX, self.gMazeY + dy) # test if cell in productive direction is clear
            if productive :
                print("Productive path at (%d,%d): %d" % (self.gMazeX, self.gMazeY, productive))
                return True
        return False
        #    DEAD CODE    
        if abs(dx) > abs(dy) :                  # better to move in X
            dy = 0 
        else :
            dx = 0
        dx = mazeclipto1(dx)
        dy = mazeclipto1(dy)
        assert(dx != 0 or dy != 0)              # error to call this at dest
        assert(dx == 0 or dy == 0)              # must be rectangular move
        productive = not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY + dy) # test if cell in productive direction is clear
        print("Productive path at (%d,%d): %d" % (self.gMazeX, self.gMazeY, productive))
        return productive
         
    def mazetakeproductivepath(self) :
        """
        Follow productive path or return 0       
        """
        dx = self.gMazeEndX - self.gMazeX
        dy = self.gMazeEndY - self.gMazeY
        clippeddx = mazeclipto1(dx)
        clippeddy = mazeclipto1(dy)
        assert(dx != 0 or dy != 0)              # error to call this at dest
        #    Try X dir first if more direct towards goal
        if abs(dx) > abs(dy) and clippeddx :
            if not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + clippeddx, self.gMazeY) :
                self.gMazeX += clippeddx                       # advance in desired dir
                self.mazeaddtopath()
                return 1
        #   Then try Y    
        if clippeddy :
            if not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX, self.gMazeY + clippeddy) :
                self.gMazeY += clippeddy                       # advance in desired dir
                self.mazeaddtopath()
                return 1 
        #   Then X, regardless of whether abs(dx) > abs(dy)
        if clippeddx :
            if not self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + clippeddx, self.gMazeY) :
                self.gMazeX += clippeddx                       # advance in desired dir
                self.mazeaddtopath()
                return 1    
                               # success
        print("Take productive path failed")
        return 0                                        # hit wall, stop
        
    def mazepickside(self) :
        """
        Which side of the wall to follow? The one that leads toward
        the goal.
        Where is the wall? One cell in the direction takkeproductvepath was
        going.
        """
        dx = self.gMazeEndX - self.gMazeX
        dy = self.gMazeEndY - self.gMazeY
        assert(dx != 0 or dy != 0)              # error to call this at dest
        clippeddx = mazeclipto1(dx)
        clippeddy = mazeclipto1(dy)
        if abs(dx) > abs(dy) :                  # better to move in X
            clippeddy = 0 
        else :
            clippeddx = 0
        assert(self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + clippeddx, self.gMazeY + clippeddy)) # must have hit a wall
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
        print("At (%d,%d) picked side %d, direction %d for wall follow." % (self.gMazeX, self.gMazeY, sidelr, direction))
        return (sidelr, direction)
        
    def mazefollowwall(self, sidelr, direction) :
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
        print("Following wall at (%d,%d) side %d direction %d md %d" % 
            (self.gMazeX, self.gMazeY, sidelr, direction, mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY)))
        dx = MAZEEDGEFOLLOWDX[direction]
        dy = MAZEEDGEFOLLOWDY[direction]
        dxsame = MAZEEDGEFOLLOWDX[((direction + sidelr) + 4) % 4] # if not blocked ahead
        dysame = MAZEEDGEFOLLOWDY[((direction + sidelr) + 4) % 4] 
        followedside = self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dxsame, self.gMazeY+dysame)
        if (not followedside) :
            print("***ERROR*** followedside not blocked. dx,dy: (%d,%d)  dxsame,dysame: (%d,%d) sidelr %d direction %d" %
                (dx,dy, dxsame,dysame, sidelr,direction))
            assert(followedside)                            # must be next to obstacle
        blockedahead = self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY + dy)
        if blockedahead :
            dxopposite = MAZEEDGEFOLLOWDX[((direction - sidelr) + 4) % 4]
            dyopposite = MAZEEDGEFOLLOWDY[((direction - sidelr) + 4) % 4]
            blockedopposite = self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dxopposite, self.gMazeY + dyopposite)
            if blockedopposite :
                print("Dead end")
                direction = (direction + 2) % 4         # dead end, reverse direction
            else :
                print("Inside corner")
                direction = (direction - sidelr + 4) % 4      # inside corner, turn
        else :
            ####dxsame = MAZEEDGEFOLLOWDX[(direction - sidelr + 4) % 4] # if not blocked ahead
            ####dysame = MAZEEDGEFOLLOWDY[(direction - sidelr + 4) % 4] 
            assert(dxsame == 0 or dysame == 0)
            blockedsameahead = self.mazetestcell(self.gMazeX + dx, self.gMazeY + dy, self.gMazeX + dx + dxsame, self.gMazeY + dy + dysame);
            if blockedsameahead :                       # straight, not outside corner
                print("Straight")
                self.gMazeX += dx                            # move ahead 1
                self.gMazeY += dy
                self.mazeaddtopath()
            else :                                      # outside corner
                print("Outside corner")
                self.gMazeX += dx                            # move ahead 1
                self.gMazeY += dy
                self.mazeaddtopath()
                #   Need to check for a productive path. May be time to stop wall following
                md = mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY)
                if md < self.gMazeMdbest and self.mazeexistsproductivepath() :
                    print("Outside corner led to a productive path halfway through")
                    return direction
                direction = (direction + sidelr + 4) % 4    # turn in direction
                self.gMazeX += dxsame                        # move around corner
                self.gMazeY += dysame
                self.mazeaddtopath() 
        return direction                                # new direction
        
    def mazeroutecornersonly(self, route) :
        """
        Condense route, only keeping corners
        
        ***BUG: Can keep duplicate points***
        """
        newroute = []
        prev0x = -1
        prev0y = -1
        prev1x = -1
        prev1y = -1
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
        
    def mazelinebarrier(self, x0, y0, x1, y1) :
        """
        Does the line between the two points hit a barrier?
        """
        if (x0 == x1) :                         # vertical line
            assert(y0 != y1)                    # must not be zero length
            if y0 > y1 :                        # sort
                temp = y0
                y0 = y1
                y1 = temp
            for y in range(y0,y1-1) :           # test each segment
                if self.mazetestcell(x0, y, x0, y+1) :
                    return False                # hit barrier
            return True
        else :
            assert(y0 == y1)
            assert(x0 != x1)
            if x0 > x1 :                        # sort
                temp = x0
                x0 = x1
                x1 = temp
            for x in range(x0,x1-1) :
                if self.mazetestcell(x, y0, x+1, y0) :
                    return False                # hit barrier
            return True
         
           
        
    def mazeoptimizeroute(self, route) :
        """
        Optimize route.
        
        The incoming route should have corners only, and represent only horizontal and vertical lines.
        Optimizing the route looks at groups of 4 points. If the two turns are both the same, then try
        to eliminate one of the points by moving the line between the two middle points.        
        """
        n = 0;
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
            if (p0x == p1x and p0y == p1y) :            # redundant point
                print("Removing redundant point %d from %s" % (n+1, str(route)))
                route = listreplacelist(route, [], n+1, n+1)
                continue
            if (p1x == p2x and p1y == p2y) :            # redundant point
                print("Removing redundant point %d from %s" % (n+2, str(route)))
                route = listreplacelist(route, [], n+2, n+2)
                continue
            if mazeinline(p0x,p0y,p1x,p1y,p2x,p2y) :
                print("Removing collinear point %d from %s" % (n+1, str(route)))
                route = listreplacelist(route, [], n+1, n+1)
                continue
            if mazeinline(p1x,p1y,p2x,p2y,p3x,p3y) :
                print("Removing collinear point %d from %s" % (n+1, str(route)))
                route = listreplacelist(route, [], n+2, n+2)
                continue

                
                
            if (p1x == p2x) :                           # if vertical middle segment
                #   End segments must be horizontal
                assert(p0y == p1y)
                assert(p2y == p3y)
                #   Is this C-shaped?
                if not ((p0y > p1y) == (p2y < p3y)) :   # no, not C-shaped
                    n = n + 1
                    continue
                #   Find shorter arm of C
                armlena = p0y-p1y 
                armlenb = p3y-p2y
                if abs(armlena) > abs(armlenb) :        # second arm is shorter
                    #   We will try to move middle segment to align with p0y, ignoring p1y
                    if self.mazelinebarrier(p3x, p0y, p3x, p3y) : # if blocked
                        n = n + 1
                        continue
                    #   We can get rid of p1 and replace p2
                    route = listreplacelist(route, [(p3x,p0y)], n+1, n+2) # remove p1
                    if n > 0 :                          # back up 1
                        n = n - 1
                    continue
                else :
                    #   We will try to move middle segment to align with p3y, ignoring p2y
                    if self.mazelinebarrier(p0x, p0y, p0x, p3y) : # if blocked
                        n = n + 1
                        continue
                    #   We can get rid of p2 and replace p1
                    route = listreplacelist(route, [(p0x, p3y)], n+1, n+2) # remove p2
                    if n > 0 :                          # back up 1
                        n = n - 1                       # to allow further optimization
                    continue                       
                    
            else :                                      # if horizontal middle segment
                assert(p1y == p2y)
                #   End segments must be vertical
                assert(p0x == p1x)
                assert(p2x == p3x)
                #   Is this C-shaped?
                if not ((p0x > p1x) == (p2x < p3x)) :   # no, not C-shaped
                    n = n + 1
                    continue
                #   Find shorter arm of C
                armlena = p0x-p1x 
                armlenb = p3x-p2x
                if abs(armlena) > abs(armlenb) :        # second arm is shorter
                    #   We will try to move middle segment to align with p3y
                    if self.mazelinebarrier(p0x, p3y, p3x, p3y) : # if blocked
                        n = n + 1
                        continue
                    #   We can get rid of p1 and p2 and replace with new point
                    route = listreplacelist(route, [(p1x, p3y)], n+1, n+2) # replace p1 and p2
                    if n > 0 :                          # back up 1
                        n = n - 1
                    continue
                else :
                    #   We will try to move middle segment to align with p0y
                    if self.mazelinebarrier(p0x, p0y, p3x, p0y) : # if blocked
                        n = n + 1
                        continue
                    #   We can get rid of p1 and p2 and replace with new point
                    route = listreplacelist(route, [(p2x,p0y)], n+1, n+2) # replace p1 and p2 with new point
                    if n > 0 :                          # back up 1
                        n = n - 1                       # to allow further optimization
                    continue 
        return route                                    # condensed route                      

            
        
    def mazedump(self, route, finalroute) :
        """
        Debug dump
        """
        print("Graph and path.")
        #   Horizontal scale
        units = ""
        tens = ""
        for i in range(self.gMazeXsize) :
            units += str(i % 10)
            tens += str((int(i / 10)) % 10)  
        print("     " + tens)                                   
        print("     " + units)            
        print("    " + ("█" * (self.gMazeXsize+2)))                 # top/bottom wall
        #   Dump maze as a little picture
        #   █ - barrier
        #   • - path
        #   ◦ - examined, not on path
        #   S - start
        #   E - end
        for i in range(self.gMazeYsize-1,-1,-1) :
            s = ""
            for j in range(self.gMazeXsize) :
                barrier = self.testdata[j][i] & MAZEBARRIER
                examined = self.testdata[j][i] & MAZEEXAMINED
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
                if i == self.gMazeStartY and j == self.gMazeStartX :
                    ch = "S"
                if i == self.gMazeEndY and j == self.gMazeEndX : 
                    ch = "E"                                            
                s = s + ch
            s = "█" + s + "█"   # show outer walls
            print("%4d%s" % (i,s))
        print("    " + ("█" * (self.gMazeXsize+2)))                 # top/bottom wall
        print("     " + tens)                                   
        print("     " + units)            

 
#
#   Test-only code
#

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
    graph = Mazegraph(xsize, ysize)
    result = graph.solvemaze(startx, starty, endx, endy, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.mazedump(result,[])   
    reachable = checkreachability(xsize, ysize, startx, starty, endx, endy, barrierpairs)
    pathfound = len(result) > 0
    print("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          # fail if disagree
    result2 = graph.mazeroutecornersonly(result)
    print("Corners only:" + str(result2))
    result3 = graph.mazeoptimizeroute(result2)
    print("Optimized: " + str(result3))
    graph.mazedump(result, result3)
    
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
    graph = Mazegraph(xsize, ysize)
    result = graph.solvemaze(0, 0, xsize-1, ysize-1, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.mazedump(result,[])
    reachable = checkreachability(xsize, ysize, 0, 0, xsize-1, ysize-1, barrierpairs)
    pathfound = len(result) > 0
    print("Reachable: %r" % (reachable,))
    assert(reachable == pathfound)          # fail if disagree

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

