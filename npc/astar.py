#
#   astar.py  - classic A* pathfinding algorithm
#
#   Finds shortest path through a grid of squares with obstacles.
#
#   Intermediate step to an implementation in LSL.
#
#
#   Animats
#   June, 2019
#
#   The algorithm is classic, and from Wikipedia. The data 
#   structures are unusual, because the purpose of this code
#   is to prepare for translation to Linden Scripting Language.
#   LSL is very limited. The only data structure is an
#   immutable list. We have to work within that limitation.
#
#   The data for each cell is:
#   - barrier - 1 bit, obstacle present
#   - examined - 1 bit, obstacle presence tested
#   - closed - 1 bit, done processing this vertex
#   - camefrom - 3 bits, direction from previous cell
#   - gcost - 10 bits, cost to get here
#
#   These are packed into 16 bits, which are packed two per 32 bit word
#   (LSL being a 32-bit system), which are stored in several LSL lists
#   to keep the list length from becoming too long. Timing tests indicate
#   that the cost of updating an LSL list is constant up to size 128; then
#   it starts to increase linearly. 
#
#   BUGS:
#   1. Route is suboptimal.                                 [FIXED]
#   2. X and Y are reversed in maps so arrows are wrong.    [FIXED]
#   3. New storage system is half-installed.
#
import numpy
import math

#
#   Data storage.
#
#   This will have to be done with globals in LSL
#
class AStarGraph(object):
    MAXCOST = 255                                                   # maximum possible cost
    ALLOWEDMOVES = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (1,-1), (1,1), (-1,1)] # allow diagonal moves  
    ALLOWEDMOVES = [(-1,0), (1,0), (0,-1), (0,1)]                   # do not diagonal moves  
    ALLOWEDARROWS = "🡠🡢🡡🡣🡤🡥🡦🡧"                                 # down is + here
    ALLOWEDARROWSBOLD = "🡰🡲🡱🡳🡴🡵🡶🡷"
    
    #   Item format
    MASKCOST = 0x00ff                                               # 8 bits for cost
    SHIFTCLOSED = 8
    MASKCLOSED = 1 << SHIFTCLOSED                                   # closed bit
    SHIFTBARRIER = 9
    MASKBARRIER = 1 << SHIFTBARRIER                                 # barrier bit
    SHIFTEXAMINED = 10
    MASKEXAMINED = 1 << SHIFTEXAMINED                               # examined bit
    SHIFTCAMEFROM = 11;
    MASKCAMEFROM = 0x7 << SHIFTCAMEFROM                             # came from 3-bit field 

 
    def __init__(self, xsize, ysize) :
        self.xsize = xsize                                          # set size of map
        self.ysize = ysize
        self.barrierarray = numpy.full((xsize, ysize),0)            # 0 means unknown, 1 means obstacle, -1 means clear
        self.closedverticesarray = numpy.full((xsize, ysize), 0)    # 0 means not closed, 1 means closed
        self.camefromarray = numpy.full((xsize, ysize), 0)          # index into ALLOWEDMOVES
        self.gcostarray = numpy.full((xsize, ysize), 0)             # G cost
        #   Data storage in a form LSL can do efficiently.  
        self.data0 = [0]*int((xsize*ysize+3)/4);                       # fill with zeroes
        self.data1 = [0]*int((xsize*ysize+3)/4);                       # fill with zeroes
        self.data2 = [0]*int((xsize*ysize+3)/4);                       # fill with zeroes
        self.data3 = [0]*int((xsize*ysize+3)/4);                       # fill with zeroes
        
        
    def get(self, x, y) :
        """
        Get 16-bit value at X,Y.
        Storage is 2 16 bit values per 32-bit word, for LSL.
        Stored in 4 lists because LSL has constant performance up to size 128,
        then it gets worse. So to allow for 32x32 storage, we do this.
        """
        ix = y*ysize+x;
        ixitem = ix / 2
        ixoffset = (ix % 2) * 16            # bit offset
        ixrow = ixitem / 4
        ixix = ixitem % 4
        if (ixrow == 0) :
            return (self.data0[ixix] >> ixoffset) & 0xffff
        elif ixrow == 1 :
            return (self.data1[ixix] >> ixoffset) & 0xffff
        elif ixrow == 2 :
            return (self.data2[ixix] >> ixoffset) & 0xffff
        elif ixrow == 3 :
            return (self.data3[ixix] >> ixoffset) & 0xffff
        else :
            raise ValueError                # unlikely
            
    def set(self, x, y, newval) :
        """
        Set 16-bit value at X,Y.
        Storage is 2 16 bit values per 32-bit word, for LSL.
        Stored in 4 lists because LSL has constant performance up to size 128,
        then it gets worse. So to allow for 32x32 storage, we do this.
        """
        newval = newval & 0xffff            # redundant, for safety
        ix = y*ysize+x;
        ixitem = ix / 2
        ixoffset = (ix % 2) * 16            # bit offset
        ixrow = ixitem / 4
        ixix = ixitem % 4
        if (ixrow == 0) :
            self.data0[ixix] =  (self.data0[ixix] & ~(0xffff << ioffset)) | (newval << ioffset)
        elif ixrow == 1 :
            self.data1[ixix] =  (self.data1[ixix] & ~(0xffff << ioffset)) | (newval << ioffset)
        elif ixrow == 2 :
            self.data2[ixix] =  (self.data2[ixix] & ~(0xffff << ioffset)) | (newval << ioffset)
        elif ixrow == 3 :
            self.data3[ixix] =  (self.data3[ixix] & ~(0xffff << ioffset)) | (newval << ioffset)
        else :
            raise ValueError                # unlikely
          
        
    def update(self, x, y, camefrom, cost, examined, barrier, closed) :
        """
        Full update
        """
        graph.camefromarray[x][y] = camefrom    # move from previous
        graph.gcostarray[x][y] = cost
        graph.closedverticesarray[x][y] = closed
        barrierval = 0
        if examined :
            if barrier :
                barrierval = 1
            else :
                barrierval = -1
        graph.barrierarray[x][y] = barrierval
        #   New form - pack into 16 bit word
        datum = (camefrom << self.SHIFTCAMEFROM | (cost & self.MASKCOST) | (examined << self.SHIFTEXAMINED) |
            (closed << self.SHIFTCLOSED) | (barrier << self.SHIFTBARRIER))
                                                       
 
    def heuristic(self, start, goal):
        """
        Ordinary distance measure
        """
        dx = start[0] - goal[0]
        dy = start[1] - goal[1]
        return int(4*math.sqrt(dx*dx + dy*dy))              # squared distance
 
    def get_vertex_neighbours(self, pos):
        """
        Try all allowed moves
        """
        n = []
        for dx, dy in self.ALLOWEDMOVES :
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 >= self.xsize or y2 < 0 or y2 >= self.ysize: # out of bounds, skip
                continue
            n.append((x2, y2))
        return n
        
    def update_barrier(self,x,y, checkbarrier) :
        """
        Get info about barrier. Only do this once per cell.
        """
        find = checkbarrier(x,y) # go out and check the barrier in the world
        if find :
            self.barrierarray[x][y] = 1         # barrier
        else :
            self.barrierarray[x][y] = -1        # no barrier
 
    def move_cost(self, a, b, checkbarrier):
        x,y = b
        v = self.barrierarray[x][y]
        if v == 0 :                             # not defined yet
            self.update_barrier(x,y, checkbarrier)            # so go look at barrier info
        if self.barrierarray[x][y] > 0:
            return self.MAXCOST                        # move into barrier, infinite cost
        dx = a[0]-b[0]
        dy = a[1]-b[1]
        return int(4*math.sqrt(dx*dx + dy*dy))
        ####return 1
        
    def dump(self, route) :
        """
        Debug dump
        """
        print("Graph info.")
        for i in range(self.ysize) :
            s = ""
            for j in range(self.xsize) :
                direction = self.camefromarray[j][i]
                barrier = self.barrierarray[j][i]
                ch = " "
                if barrier == 0 :
                    s = s + " "
                elif barrier > 0 :
                    ch = "█"
                else :
                    if (j,i) in route :                    
                        ch = self.ALLOWEDARROWSBOLD[direction] # show appropriate arrow
                    else :
                        ch = self.ALLOWEDARROWS[direction] # show appropriate arrow
                s = s + ch
            print(s)
        print("Cost info")
        for i in range(self.ysize) :
            s = ""
            for j in range(self.xsize) :
                cost = self.gcostarray[j][i]
                s = s + ("%4i " % (cost,))
            print(s)

            
        
 
def AStarSearch(start, end, graph, checkbarrier):
 
    #   Initialize starting values
    openVertices = [(start, graph.heuristic(start, end))]            # our to-do ilst
 
    while len(openVertices) > 0:
        #   Get the vertex in the open list with the lowest F score.
        current = None
        currentFscore = None
        for (pos, fscore) in openVertices:
            if current is None or fscore < currentFscore :
                currentFscore = fscore
                current = pos
 
        #   Check if we have reached the goal
        if current == end :
            #   Retrace our route backward
            path = [current]
            while current != start :
                currentdirix = graph.camefromarray[current[0]][current[1]]
                currentdir = graph.ALLOWEDMOVES[currentdirix]       # get current dir offset
                current = (current[0] - currentdir[0], current[1] - currentdir[1])
                if graph.barrierarray[current[0]][current[1]] > 0 :
                    RuntimeError("ERROR: path through blocked point at " + str(current))
                path.append(current)
            path.reverse()
            return path                                                 # done
 
        #   Mark the current vertex as closed
        ix = findinpairlist(openVertices, current)                      # index of vertex to remove
        assert(ix >= 0)
        del(openVertices[ix])
        #   Expensive update step
        graph.closedverticesarray[current[0]][current[1]] = 1           # update map of vertices done
 
        #   Update scores for vertices near the current position
        for neighbor in graph.get_vertex_neighbours(current):
            if graph.closedverticesarray[neighbor[0],neighbor[1]] :     # cell marked as done, skip
                continue
            candidateG = graph.gcostarray[current[0]][current[1]] + graph.move_cost(current, neighbor, checkbarrier) # always 1 or infinite.
            if candidateG >= graph.MAXCOST :                            # bound cost
                continue                                                # hit barrier, skip
            ix = findinpairlist(openVertices, neighbor)
            if ix < 0 :                                                 # new neighbor
                openVertices.append((neighbor, 0))                      # discovered a new vertex - add with dummy fscore
            elif candidateG >= graph.gcostarray[neighbor[0]][neighbor[1]]:
                continue                                                # this G score is no better than previously found
 
            #   Adopt this G score
            neighbordiff = (neighbor[0] - current[0], neighbor[1] - current[1]) # offset to neighbor
            assert(neighbordiff in graph.ALLOWEDMOVES)                        # must be valid move
            #   Expensive update step where lists are updated.
            #   Here, we know that examined is true, barrier is false, and closed is true
            ####graph.camefromarray[neighbor[0]][neighbor[1]] = graph.ALLOWEDMOVES.index(neighbordiff)    # move from previous
            ####graph.gcostarray[neighbor[0]][neighbor[1]] = candidateG
            graph.update(neighbor[0],neighbor[1], graph.ALLOWEDMOVES.index(neighbordiff), candidateG, True, False, True)   # update graph
            H = graph.heuristic(neighbor, end)
            fscore = graph.gcostarray[neighbor[0]][neighbor[1]] + H
            #   Update fscore for this item in to-do list
            ix = findinpairlist(openVertices, neighbor)
            assert(ix >= 0)                                             # must find
            openVertices[ix] = (neighbor, fscore) 

 
    raise RuntimeError("A* failed to find a solution")
    
def findinpairlist(lst, key) :
    """
    Find index of value in list of k,v
    
    Done this way to prepare for conversion to LSL
    """
    for ix in range(len(lst)) :
        if lst[ix][0] == key :
            return ix
    return -1                                                       # no find
    
#   Test barrier. These cells are blocked.
BARRIERDEF = [(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)]

def checkbarriercell(ix, iy) :
    """
    Get whether a point is a barrier cell.
    
    Simulates actually probing the world for obstacles
    """
    return (ix, iy) in BARRIERDEF           # true if on barrier
    
 
if __name__=="__main__":
    graph = AStarGraph(8,8)
    result = AStarSearch((0,0), (7,7), graph, checkbarriercell)
    print ("route", result)
    print ("cost", len(result))
    graph.dump(result)

