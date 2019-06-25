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

#
#   Data storage.
#
#   This will have to be done with globals in LSL
#
class Mazegraph(object):
   
    def __init__(self, xsize, ysize) :
        self.xsize = xsize                                          # set size of map
        self.ysize = ysize
        self.testdata = numpy.full((xsize, ysize), 0)
                        
    def testcell(self, x, y, barriercheck) :
        v = self.testdata[x][y]                 # this cell
        if (v & EXAMINED) :
            return v & BARRIER                  # already have this one
        barrier = barriercheck(x,y)             # check this location
        v = EXAMINED | barrier
        self.testdata[x][y] = v                 # update sites checked
        return barrier                          # return 1 if obstacle
        
    def dump(self, route) :
        """
        Debug dump
        """
        print("Graph and path.")
        for i in range(self.ysize) :
            s = ""
            for j in range(self.xsize) :
                barrier = self.testdata[j][i] & BARRIER
                ch = " "
                if barrier == 0 :
                    s = s + " "
                elif barrier > 0 :
                    ch = "█"
                else :
                    if (j,i) in route :                    
                        ch = "*"
                s = s + ch
            print(s)
 
def limitto1(n) :
    """
    Limit to range -1 .. 1
    """
    if (n < 0) :
        return -1
    if (n > 0) :
        return 1
    return 0            
        
 
def solvemaze(start, end, graph, checkbarrier) :
    """
    Maze solver. Returns a path through the maze.
    
    This is not always optimal in terms of route length, but is efficient
    in terms of number of cells tested.
    """
 
    x = start[0]                                        # current working position in cell coords
    y = start[1]
    xend = end[0]
    yend = end[1]
    path = [(x,y)]                                      # working path
    edgefollow = 0                                      # -1=right edge, 1 = left edge, 0=no edge
    edgefollowdir = 0                                   # 0=+x, 1 = +y, 2=-x, 3=-y
    
    while x != start[0] and y != start[1] and len(path) > 1 : # if we end up back at the starting point, we are in a loop
        print("(%d,%d)" % (x,y))                        # ***TEMP***
        dx = xend - x                                   # distance to end
        dy = yend - y
        if (dx == 0) and (dy == 0) :                    # if done
            return path                                 # at goal, done
        if len(path) > graph.xsize * graph.ysize :      # are we in a loop?
            return []                                   # yes, quit
        #   Which direction is most direct towards goal?
        if abs(dx) > abs(dy) :                          # X is most direct
            dy = 0
            dx = limitto1(dx)
        else :                                          # Y is most direct
            dx = 0
            dy = limitto1(dy)        
        obstacle = graph.testobstacle(x+dx, y+dy)       # check for obstacle
        if not obstacle :                               # it worked
            x = x + dx                                  # advance to new point
            y = y + dy
            path += (x,y)                               # add point to path
            edgefollow = 0
            continue                                    # and keep going
        #   Can't advance directly towards goal. Must now follow an edge.
        if edgefollow != 0 :                            # if don't have an edge direction
           pass ### ***MORE***
        #   Collect data on obstacles ahead, left, and right
        aheaddx, aheaddy = EDGEFOLLOWDIRS(edgefollowdir)          # get delta for this direction
        obstacleahead = graph.testobstacle(x+aheaddx,y+aheaddy)   # test cell ahead
        leftdx, leftdy = EDGEFOLLOWDIRS((edgefollowdir + 1) % 4) # on right
        obstacleleft = graph.testcell(x + leftdx, y + leftdy, checkbarrier)
        rightdx, rightdy = EDGEFOLLOWDIRS((edgefollowdir -1 + 4) % 4) # on left
        obstacleright = graph.testcell(x + rightdx, y + rightdy, checkbarrier)
        #   If no obstacle ahead, we are still wall following and there is an
        #   obstacle to left or right, so move ahead.
        if not obstacleahead :                              # if no obstacle ahead
            assert(obstacleright or obstacleleft)           # must be one of the other
            x = x + dx
            y = y + dy
            path += (x,y)                                   # add to path
            #   Now decide next direction for wall following.
            #   If no obstacle on followed side, turn that way and advance
            if (not obstacleright) and edgefollow == -1 :   # if open on right and following right
                edgefollowdir = (edgefollowdir - 1 + 4) % 4 # right turn
            elif (not obstacleleft) and edgefollow == 1 :   # if open on left and following left
                edgefollowdir = (edgefollowdir +1) % 4      # left turn
            else :
                continue                                    # not turning on this cycle
            dx, dy = EDGEFOLLOWDIRS(edgefollowdir)          # one step in new dir
            x = x + dx
            y = y + dy
            path += (x,y)
            continue
        #   Obstacle ahead. Must turn.        
        if (not obstacleright) and edgefollow == -1 :       # if open on right and following right
            edgefollowdir = (edgefollowdir - 1 + 4) % 4     # right turn
        elif (not obstacleleft) and edgefollow == 1 :       # if open on left and following left
            edgefollowdir = (edgefollowdir +1) % 4          # left turn
        else :                                              # blocked on both sides
            edgefollowdir = (edgefollowdir + 2) % 4         # reverse direction
        dx, dy = EDGEFOLLOWDIRS(edgefollowdir)          # one step in new dir
        x = x + dx
        y = y + dy
        path += (x,y)
        continue
    return []                                           # fails
    
    
    
#   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell1(ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF1           # true if on barrier
    
#   Test barriers. These cells are blocked.
BARRIERDEF2 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell2(ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF2           # true if on barrier
    
def runtest(xsize, ysize, barrierfn) :
    graph = Mazegraph(xsize, ysize)
    result = solvemaze((0,0), (xsize-1, ysize-1), graph, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.dump(result)

    
 
if __name__=="__main__":
    runtest(8,8,checkbarriercell1)
    runtest(32,32,checkbarriercell2)

