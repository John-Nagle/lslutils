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
        self.x = startx
        self.y = starty
        self.barrierfn = barrierfn              # tests cell for blocked
        self.endx = endx
        self.endy = endy
        self.mdbest = self.xsize+self.ysize+1   # best dist to target init
        self.path = []                          # accumulated path
        #   Outer loop - shortcuts or wall following
        while (self.x != self.endx and self.y != self.endy) : # while not at dest
            if (len(self.path) > self.xsize*self.ysize) :
                return []                       # we are in a loop
            if (self.existsproductivepath()) :
                self.takeproductivepath()
            else :
                self.mdbest = md(self.x, self.y, self.endx, self.endy)
                sidelr, direction = self.pickside()        # follow left or right?
                while md(self.x, self.y, self.endx, self.endy) != self.mdbest or not self.existsproductivepath() :
                    direction = self.followwall(sidelr, direction)     # follow edge, advance one cell
        return(self.path)
                    
    def addtopath(self) :
        """
        Add current position to path
        """
        self.path += [self.x, self.y]
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
            clippeddy = 0
        assert(clippeddx == 0 or clippeddy == 0) # must be rectangular move
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
        elif clippddy == 1 :                    # obstacle is in +Y dir
            if (dx > 0) :                       # if want to move in +X
                direction = 0
                sidelr = WALLONLEFT             # wall is on left
            else :
                direction = 2
                sidelr = WALLONRIGHT
        elif clippedy == -1 :                   # obstacle is in -Y dir
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
            dxopposite = EDGEFOLLOWDX[(direction - sidelr + 4) % 4]
            dyopposite = EDGEFOLLOWDY[(direction - sidelr + 4) % 4]
            blockedopposite = self.testcell(self.x, self.y, self.x + dxopposite, self.y + dyopposite)
            if blockedopposite :
                direction = (direction + 2) % 4         # dead end, reverse direction
            else :
                direction = (direction -1 + 4) % 4      # inside corner, turn (***CHECK THIS***)
        else :
            dxsame = EDGEFOLLOWDX[(direction + sidelr) % 4] # if not blocked ahead
            dysame = EDGEFOLLOWDX[(direction + sidelr) % 4] 
            blockedsameahead = self.testcell(self.x + dx, self.y + dy, self.x + dx + dxsame, self.y + dy + dysame);
            if blockedsameahead :                       # straight, not outside corner
                self.x += dx                            # move ahead 1
                self.y += dy
                self.addtopath()
            else :                                      # outside corner
                self.x += dx                            # move ahead 1
                self.y += dy
                self.addtopath()
                direction = (direction + sidelr + 4) % 4    # turn in direction
                self.x += dxsame                        # move around corner
                self.y += dxsame
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
 
def limitto1(n) :
    """
    Limit to range -1 .. 1
    """
    if (n < 0) :
        return -1
    if (n > 0) :
        return 1
    return 0  
             
        
 
def solvemaze(start, end, graph ) :
    """
    Maze solver. Returns a path through the maze.
    
    This is not always optimal in terms of route length, but is efficient
    in terms of number of cells tested.
    
    Algorithm is from Wikipedia: 
    
    https://en.wikipedia.org/wiki/Maze_solving_algorithm#Maze-routing_algorithm
    
    It's wall-following with shortcuts through open spaces.
    
    Wall following rules:
        Always blocked on follow side. Algorithm error if not.
        
        If open ahead and blocked on follow side 1 ahead, 
            advance straight.
        If open ahead and not blocked on follow side 1 ahead, outside corner,
            advance straight, 
            turn towards follow side, 
            advance straight.
        If not open ahead and not blocked opposite follow side, inside corner
            turn away from follow side. No move.
        If not open ahead and blocked opposite follow side, dead end
            turn twice to reverse direction, no move.
    """
 
    x = start[0]                                        # current working position in cell coords
    y = start[1]
    xend = end[0]
    yend = end[1]
    path = [(x,y)]                                      # working path
    edgefollow = 0                                      # -1=right edge, 1 = left edge, 0=no edge
    edgefollowdir = 0                                   # 0=+x, 1 = +y, 2=-x, 3=-y
    bestdist = graph.xsize + graph.ysize + 1            # worst possible distance
    print("Solving maze")
    while True:                                         # until done or loop
        print("(%d,%d)" % (x,y))                        # ***TEMP***
        assert(not graph.testcell(x,y));                # must not be on an obstacle cell
        dx = xend - x                                   # distance to end
        dy = yend - y
        if (dx == 0) and (dy == 0) :                    # if done
            return path                                 # at goal, done
        if len(path) > graph.xsize * graph.ysize :      # are we in a loop?
            return []                                   # yes, quit
        #   Which direction is most direct towards goal?
        #   Newedgefollow does not match Wikipedia.
        newedgefollow = 1                               # assume follow left
        if abs(dx) > abs(dy) :                          # X is most direct
            if (dx > 0) :                               # +X wins
                ahead = 0                               # +X is ahead, for later use
                if (dy < 0) :                           # if +Y is better
                    newedgefollow = -1                  # follow right edge
                dx = 1
                dy = 0
            else :                                      # -X wins
                ahead = 3                               # -X direction
                if (dy > 0) :                           # follow
                    newedgefollow = -1                  # follow right edge
                dx = -1
                dy = 0
        else :                                          # Y is most direct
            if (dy > 0) :                               # we want to move in +Y
                ahead = 1                               # +Y direction
                if (dx < 0) :                            
                    newedgefollow = -1                  # follow right edge                  
                dx = 0
                dy = 1
            else :                                      # move in -Y
                ahead = 3
                if (dx > 0) :
                    newedgefollow = -1                  # follow right edge
                dx = 0
                dy = -1

        #   x+dx, y+dy is the "productive path".
        currdist = abs(x-xend) + abs(y-yend)            # distance to obstacle
        obstacle = graph.testcell(x+dx, y+dy)           # check for obstacle on productive path
        if (not obstacle) and (currdist != bestdist) and edgefollow == 0:  # can advance, and not in a loop, and not edge following
            x = x + dx                                  # advance to new point
            y = y + dy
            print("Productive path at (%d,%d)" % (x,y)) # ***TEMP***
            path += [(x,y)]                             # add point to path
            edgefollow = 0                              # no edge follow direction
            continue                                    # and keep going
        #   Can't advance directly towards goal. Must now follow an edge.
        if edgefollow == 0 :                            # if don't have an edge direction
            edgefollow = newedgefollow                  # use new edge follow direction
            edgefollowdir = ahead                       # assume ahead, not that it will work
            print("(%d,%d) Starting edge follow." % (x,y))
            bestdist = currdist
        #   Collect data on obstacles ahead, left, and right
        aheaddx, aheaddy = EDGEFOLLOWDIRS[edgefollowdir]    # get delta for this direction
        obstacleahead = graph.testcell(x+aheaddx,y+aheaddy) # test cell ahead
        leftdx, leftdy = EDGEFOLLOWDIRS[(edgefollowdir - 1 + 4) % 4] # on right
        obstacleleft = graph.testcell(x + leftdx, y + leftdy)
        rightdx, rightdy = EDGEFOLLOWDIRS[(edgefollowdir +1) % 4] # on left
        obstacleright = graph.testcell(x + rightdx, y + rightdy) 
        print("Wall following from (%d,%d): l/r %d, heading %d, ahead %d, left %d, right %d" % (x,y,edgefollow, edgefollowdir, obstacleahead, obstacleleft, obstacleright))        
        #   If no obstacle ahead, we are still wall following and there is an
        #   obstacle to left or right, so move ahead.
        if not obstacleahead :                              # if no obstacle ahead in follow dir
            #   ***WRONG*** Must go ahead, but then what?
            assert(obstacleright or obstacleleft)           # must be next to a wall
            x = x + aheaddx
            y = y + aheaddy
            path += [(x,y)]                                 # add to path
            print("(%d,%d) No obstacle ahead" % (x,y))
            ####continue    ### ***WRONG***
            #   If nothing on follow side after that move, must turn and advance
            #   to get around an outside corner
            obstacleleft = graph.testcell(x + leftdx, y + leftdy)
            obstacleright = graph.testcell(x + rightdx, y + rightdy) 
            
            #   Now decide next direction for wall following.
            #   If no obstacle on followed side, turn that way and advance
            if (not obstacleright) and edgefollow == -1 :   # if open on right and following right
                edgefollowdir = (edgefollowdir + -1 + 4) % 4     # right turn
                aheaddx, aheaddy = EDGEFOLLOWDIRS[edgefollowdir]    # get delta for this direction
                x = x + aheaddx
                y = y + aheaddy
                print("Wall following outside right turn extra move (%d,%d)" % (x,y))
                path += [(x,y)]                                 # add to path
                continue
            elif (not obstacleleft) and edgefollow == 1 :   # if open on left and following left
                edgefollowdir = (edgefollowdir + 1) % 4  # left turn
                aheaddx, aheaddy = EDGEFOLLOWDIRS[edgefollowdir]    # get delta for this direction
                x = x + aheaddx
                y = y + aheaddy
                print("Wall following outside left turn extra move (%d,%d)" % (x,y))
                path += [(x,y)]                                 # add to path               
            continue
        #   Obstacle ahead. Must turn.        
        if (not obstacleright) and edgefollow == -1 :       # if open on right and following right
            edgefollowdir = (edgefollowdir + 1) % 4         # right turn
        elif (not obstacleleft) and edgefollow == 1 :       # if open on left and following left
            edgefollowdir = (edgefollowdir -1 + 4) % 4         # left turn
        else :                                              # blocked on both sides
            edgefollowdir = (edgefollowdir + 2) % 4         # reverse direction
        dx, dy = EDGEFOLLOWDIRS[edgefollowdir]              # one step in new dir
        x = x + dx
        y = y + dy
        path += [(x,y)]
        continue
    return []                                           # fails
    
    
    
#   Test barriers. These cells are blocked.
BARRIERDEF1 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell1(prevx, prevy, ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF1           # true if on barrier
    
#   Test barriers. These cells are blocked.
BARRIERDEF2 = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell2(prevx, prevy, ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF2           # true if on barrier
    
def runtest(xsize, ysize, barrierfn) :
    graph = Mazegraph(xsize, ysize)
    result = graph.solvemaze(0, 0, xsize-1, ysize-1, barrierfn)
    print ("route", result)
    print ("cost", len(result))
    graph.dump(result)

    
 
if __name__=="__main__":
    runtest(8,8,checkbarriercell1)
    ####runtest(32,32,checkbarriercell2)

