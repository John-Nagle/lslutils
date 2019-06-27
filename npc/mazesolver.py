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
MAZEWALLONLEFT = -1
MAZEWALLONRIGHT = 1

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
            if (len(self.gMazePath) > self.gMazeXsize*self.gMazeYsize*2) :
                return []                       # we are in an undetected loop
            if (self.mazeexistsproductivepath()) :  # if a shortcut is available
                self.mazetakeproductivepath()       # use it
            else :
                self.gMazeMdbest = mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY)
                sidelr, direction = self.mazepickside()        # follow left or right?
                #   Inner loop - wall following
                followstartx = self.gMazeX
                followstarty = self.gMazeY
                followstartdir = direction
                while mazemd(self.gMazeX, self.gMazeY, self.gMazeEndX, self.gMazeEndY) != self.gMazeMdbest or not self.mazeexistsproductivepath() :
                    direction = self.mazefollowwall(sidelr, direction)      # follow edge, advance one cell
                    if len(self.gMazePath) > self.gMazeXsize*self.gMazeYsize*2 : # runaway check
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
        
        Consider adding check to generate diagonals to reduce path
        cleanup cost.
        """
        dx = self.gMazeEndX - self.gMazeX
        dy = self.gMazeEndY - self.gMazeY
        if abs(dx) > abs(dy) :                  # better to move in X
            dy = 0 
        else :
            dx = 0
        dx = mazeclipto1(dx)
        dy = mazeclipto1(dy)
        assert(dx != 0 or dy != 0)              # error to call this at dest
        assert(dx == 0 or dy == 0)              # must be rectangular move
        if (self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY + dy)) :
            return 0                            # hit wall, stop
        self.gMazeX += dx                            # advance in desired dir
        self.gMazeY += dy
        self.mazeaddtopath()
        return 1                                # success
        
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
            if  (dy > 0) :                      # if want to move in +Y
                direction = 1
                sidelr = MAZEWALLONLEFT 
            else :
                direction = 3
                sidelr = MAZEWALLONRIGHT
        elif clippeddx == -1 :
            if (dy > 0) :
                direction = 1
                sidelr = MAZEWALLONRIGHT
            else :
                direction = 3
                sidelr = MAZEWALLONLEFT                
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
                sidelr = MAZEWALLONLEFT             # wall is on left
            else :
                direction = 2
                sidelr = MAZEWALLONRIGHT
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
        print("Following wall at (%d,%d) side %d direction %d" % (self.gMazeX, self.gMazeY, sidelr, direction))
        dx = MAZEEDGEFOLLOWDX[direction]
        dy = MAZEEDGEFOLLOWDY[direction]
        blockedahead = self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dx, self.gMazeY + dy)
        if blockedahead :
            dxopposite = MAZEEDGEFOLLOWDX[(direction + sidelr) % 4]
            dyopposite = MAZEEDGEFOLLOWDY[(direction + sidelr) % 4]
            blockedopposite = self.mazetestcell(self.gMazeX, self.gMazeY, self.gMazeX + dxopposite, self.gMazeY + dyopposite)
            if blockedopposite :
                print("Dead end")
                direction = (direction + 2) % 4         # dead end, reverse direction
            else :
                print("Inside corner")
                direction = (direction -1 + 4) % 4      # inside corner, turn
        else :
            dxsame = MAZEEDGEFOLLOWDX[(direction - sidelr + 4) % 4] # if not blocked ahead
            dysame = MAZEEDGEFOLLOWDY[(direction - sidelr + 4) % 4] 
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
                direction = (direction - sidelr + 4) % 4    # turn in direction
                self.gMazeX += dxsame                        # move around corner
                self.gMazeY += dysame
                self.mazeaddtopath() 
        return direction                                # new direction   
        
    def mazedump(self, route) :
        """
        Debug dump
        """
        print("Graph and path.")
        #   Horizontal scale
        units = ""
        tens = ""
        for i in range(self.gMazeXsize) :
            units += str(i % 10)
            tens += str((i / 10) % 10)  
        print("    " + units)
        print("    " + tens)                                   
            
        print("    " + ("•" * (self.gMazeXsize+2)))                 # top/bottom wall
        #   Data
        for i in range(self.gMazeYsize) :
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
                s = s + ch
            s = "•" + s + "•"   # show outer walls
            print("%4d%s" % (i,s))
        print("    " + ("•" * (self.gMazeXsize+2)))                 # top/bottom wall

 

def generaterandombarrier(xsize, ysize, cnt) :
    """
    Generate a lame random maze. Just random dots.
    """
    pts = []
    for i in range(cnt) :
        pnt = (random.randrange(xsize), random.randrange(ysize))
        if pnt == (0,0) or pnt == (xsize-1, ysize-1):       # start and end point must be free
            continue
        if not pnt in pts :
            pts.append(pnt)
    print("Random barrier: " + str(pts))  
    return pts 
             
#   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]
BARRIERBLOCKER = [(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(10,8),(11,8)]
BARRIERCENTER = [(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(4,9),(5,9),(6,9)]
BARRIERRANDOM = generaterandombarrier(12,12,72)
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

def runtest(xsize, ysize, barrierpairs) :
    def barrierfn(prevx, prevy, ix, iy) :   # closure for barrier test fn
        return (ix, iy) in barrierpairs
    graph = Mazegraph(xsize, ysize)
    result = graph.solvemaze(0, 0, xsize-1, ysize-1, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.mazedump(result)    
    
 
if __name__=="__main__":
    runtest(12,12,BARRIERDEF1+BARRIERCENTER)
    runtest(12,12,BARRIERDEF1+BARRIERBLOCKER)
    runtest(12,12,BARRIERSTUCK)
    randombarrier = generaterandombarrier(12,12,72)
    runtest(12,12,randombarrier)
    runtest(12,12,BARRIERFAIL1)

