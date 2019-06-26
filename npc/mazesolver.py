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
#   The algorithm is classic, and from Wikipedia. 
#
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

BARRIER = 0x1                                   # must be low bit
EXAMINED = 0x2

EDGEFOLLOWDIRS = [(1,0), (0, 1), (-1, 0), (0, -1)]  # edge following dirs to dx and dy

EDGEFOLLOWDX = [1,0,-1,0]
EDGEFOLLOWDY = [0,1,0,-1]

#   Wall follow sides
WALLONLEFT = -1
WALLONRIGHT = 1

#
#   md -- rectangular "Manhattan" distance
#
def md(p0x, p0y, p1x, p1y) :
    return abs(p1x-p0x) + abs(p1y-p0y)
    
#
#   clipto1 -- clip to range -1, 1
#
def clipto1(n) :
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
        self.xsize = xsize                                          # set size of map
        self.ysize = ysize
        self.testdata = numpy.full((xsize, ysize), 0)
       
    def solvemaze(self, startx, starty, endx, endy, barrierfn) :
        self.x = startx                         # start
        self.y = starty
        self.barrierfn = barrierfn              # tests cell for blocked
        self.startx = startx                    # start
        self.starty = starty
        self.endx = endx                        # destination
        self.endy = endy
        self.mdbest = self.xsize+self.ysize+1   # best dist to target init
        self.path = []                          # accumulated path
        self.stuck = 0                          # not stuck yet
        self.addtopath()                        # add initial point
        #   Outer loop - shortcuts or wall following
        while (self.x != self.endx or self.y != self.endy) : # while not at dest
            if (len(self.path) > self.xsize*self.ysize*2) :
                return []                       # we are in a loop
            if (self.existsproductivepath()) :
                self.takeproductivepath()
            else :
                self.mdbest = md(self.x, self.y, self.endx, self.endy)
                sidelr, direction = self.pickside()        # follow left or right?
                #   Inner loop - wall following
                while md(self.x, self.y, self.endx, self.endy) != self.mdbest or not self.existsproductivepath() :
                    direction = self.followwall(sidelr, direction)      # follow edge, advance one cell
                    assert(len(self.path) <= self.xsize*self.ysize*2)   # runaway check
                    #   Termination check - if we are back at the start and going in the same direction, no solution
                    if (self.x == self.startx and self.y == self.starty) :
                        print("Back at start")
                        self.stuck += 1
                        if (self.stuck > 2) :                           # back at start more than twice, we're stuck.
                            return []                                   # fails
                        
        return(self.path)
                    
    def addtopath(self) :
        """
        Add current position to path
        """
        self.path += [(self.x, self.y)]
        print("(%d,%d)" % (self.x, self.y))
        ####assert(not self.testcell(self.x, self.y, self.x + dx, self.y + dy)) # path must not go into an occupied cell

                                                     
    def testcell(self, fromx, fromy, x, y) :
        """
        Returns 1 if occupied cell
        """
        print("Testcell (%d,%d)" % (x,y))       # ***TEMP***
        if (x < 0 or x >= self.xsize or y < 0 or y >= self.ysize) : # if off grid
            return 1                            # treat as occupied
        v = self.testdata[x][y]                 # this cell
        if (v & EXAMINED) :
            print
            return v & BARRIER                  # already have this one
        barrier = self.barrierfn(fromx, fromy, x,y)             # check this location
        v = EXAMINED | barrier
        self.testdata[x][y] = v                 # update sites checked
        return barrier                          # return 1 if obstacle
        
    def existsproductivepath(self) :
        """
        True if a productive path exists
        """
        dx = self.endx - self.x
        dy = self.endy - self.y
        if abs(dx) > abs(dy) :                  # better to move in X
            dy = 0 
        else :
            dx = 0
        dx = clipto1(dx)
        dy = clipto1(dy)
        assert(dx != 0 or dy != 0)              # error to call this at dest
        assert(dx == 0 or dy == 0)              # must be rectangular move
        productive = not self.testcell(self.x, self.y, self.x + dx, self.y + dy) # test if cell in productive direction is clear
        print("Productive path at (%d,%d): %d" % (self.x, self.y, productive))
        return productive
         
    def takeproductivepath(self) :
        """
        Follow productive path or return 0
        """
        dx = self.endx - self.x
        dy = self.endy - self.y
        if abs(dx) > abs(dy) :                  # better to move in X
            dy = 0 
        else :
            dx = 0
        dx = clipto1(dx)
        dy = clipto1(dy)
        assert(dx != 0 or dy != 0)              # error to call this at dest
        assert(dx == 0 or dy == 0)              # must be rectangular move
        if (self.testcell(self.x, self.y, self.x + dx, self.y + dy)) :
            return 0                            # hit wall, stop
        self.x += dx                            # advance in desired dir
        self.y += dy
        self.addtopath()
        return 1                                # success
        
    def pickside(self) :
        """
        Which side of the wall to follow? The one that leads toward
        the goal.
        Where is the wall? One cell in the direction takkeproductvepath was
        going.
        """
        dx = self.endx - self.x
        dy = self.endy - self.y
        assert(dx != 0 or dy != 0)              # error to call this at dest
        clippeddx = clipto1(dx)
        clippeddy = clipto1(dy)
        if abs(dx) > abs(dy) :                  # better to move in X
            clippeddy = 0 
        else :
            clippeddx = 0
        assert(self.testcell(self.x, self.y, self.x + clippeddx, self.y + clippeddy)) # must have hit a wall
        #   8 cases, dumb version
        if clippeddx == 1 :                     # obstacle is in +X dir
            if  (dy > 0) :                      # if want to move in +Y
                direction = 1
                sidelr = WALLONLEFT 
            else :
                direction = 3
                sidelr = WALLONRIGHT
        elif clippeddx == -1 :
            if (dy > 0) :
                direction = 1
                sidelr = WALLONRIGHT
            else :
                direction = 3
                sidelr = WALLONLEFT                
        elif clippeddy == 1 :                   # obstacle is in +Y dir
            if (dx > 0) :                       # if want to move in +X
                direction = 0
                sidelr = WALLONLEFT             # wall is on left
            else :
                direction = 2
                sidelr = WALLONRIGHT
        elif clippeddy == -1 :                  # obstacle is in -Y dir
            if (dx > 0) :                       # if want to move in +X
                direction = 0
                sidelr = WALLONLEFT             # wall is on left
            else :
                direction = 2
                sidelr = WALLONRIGHT
        else :
            assert(False)                       # should never get here
        print("At (%d,%d) picked side %d, direction %d for wall follow." % (self.x, self.y, sidelr, direction))
        return (sidelr, direction)
        
    def followwall(self, sidelr, direction) :
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
        print("Following wall at (%d,%d) side %d direction %d" % (self.x, self.y, sidelr, direction))
        dx = EDGEFOLLOWDX[direction]
        dy = EDGEFOLLOWDY[direction]
        blockedahead = self.testcell(self.x, self.y, self.x + dx, self.y + dy)
        if blockedahead :
            dxopposite = EDGEFOLLOWDX[(direction + sidelr + 4) % 4]
            dyopposite = EDGEFOLLOWDY[(direction + sidelr + 4) % 4]
            blockedopposite = self.testcell(self.x, self.y, self.x + dxopposite, self.y + dyopposite)
            if blockedopposite :
                print("Dead end")
                direction = (direction + 2) % 4         # dead end, reverse direction
            else :
                print("Inside corner")
                direction = (direction -1 + 4) % 4      # inside corner, turn (***CHECK THIS***)
        else :
            dxsame = EDGEFOLLOWDX[(direction - sidelr + 4) % 4] # if not blocked ahead
            dysame = EDGEFOLLOWDY[(direction - sidelr + 4) % 4] 
            blockedsameahead = self.testcell(self.x + dx, self.y + dy, self.x + dx + dxsame, self.y + dy + dysame);
            if blockedsameahead :                       # straight, not outside corner
                print("Straight")
                self.x += dx                            # move ahead 1
                self.y += dy
                self.addtopath()
            else :                                      # outside corner
                print("Outside corner")
                self.x += dx                            # move ahead 1
                self.y += dy
                self.addtopath()
                direction = (direction - sidelr + 4) % 4    # turn in direction
                self.x += dxsame                        # move around corner
                self.y += dysame
                self.addtopath() 
        return direction                                # new direction   
        
    def dump(self, route) :
        """
        Debug dump
        """
        print("Graph and path.")
        for i in range(self.ysize) :
            s = ""
            for j in range(self.xsize) :
                barrier = self.testdata[j][i] & BARRIER
                examined = self.testdata[j][i] & EXAMINED
                ch = " "
                if examined :
                    ch = "◦"
                if barrier > 0 :
                    ch = "█"
                else :
                    if (j,i) in route :                    
                        ch = "•"
                s = s + ch
            print(s)
 
    
#   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell1(prevx, prevy, ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF1           # true if on barrier
    
#   Test barriers. These cells are blocked.
####BARRIERDEF2 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]
BARRIERBLOCKER = [(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(10,8),(11,8)]

def checkbarriercell2(prevx, prevy, ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF1 + BARRIERBLOCKER           # true if on barrier
    
def runtest(xsize, ysize, barrierfn) :
    graph = Mazegraph(xsize, ysize)
    result = graph.solvemaze(0, 0, xsize-1, ysize-1, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.dump(result)

    
 
if __name__=="__main__":
    runtest(8,8,checkbarriercell1)
    runtest(12,12,checkbarriercell2)

