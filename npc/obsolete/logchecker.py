#
#   Log checker for path planner logs.
#
import sys
import re

#   Match these patterns in logs
REMAZESTART = re.compile(r".*Path Maze solver task: Request to maze solver:.*\"pathid\":(\d+),.*\"segmentid\":(\d+)")
####REMAZEEND = re.compile(r".*Path Maze solver task: Maze solver finished task\, pathid (\d+)\, segment (\d+)")
REMAZEEND = re.compile(r".*Path Maze solver task: Maze solver finished task\, pathid (\d+)\, segment (\d+)")

#
#   class logdata -- analyze log 
#
class logdata :
    
    def __init__(self) :
        self.pathid = None                      # current path ID
        self.segid = None                       # current segid
        
    def readlog(self, fname) :
        with open(fname,"r") as infile :
            for line in infile :
                self.doline(line)
                
    def doline(self, line) :
        match1 = REMAZESTART.match(line)        # check for request start 
        if match1 :
            print("Start: %s %s" % (match1.group(1), match1.group(2)))
        match2 = REMAZEEND.match(line)          # check for request end
        if match2 :
            print("End:   %s %s" % (match2.group(1), match2.group(2)))
            

        
        
                
                
#
#   Main program
#
def main() :
    for fname in sys.argv[1:] :
        print("Processing file \"%s\"" % (fname,))
        logitem = logdata()
        logitem.readlog(fname)                  # do file
        
if __name__ == "__main__" :
    main()
        
        

    
    
