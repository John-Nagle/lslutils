//
//  mazesolvercall.lsl
//
//  Finds reasonable path through a grid of squares with obstacles.
//
//  Animats
//  June, 2019
//
//  This file is used by callers of the maze solver. The maze solver
//  itself is in a separate script, for memory size reasons.
//
#include "npc/mazedefs.lsl"
////#include "npc/pathbuildutils.lsl"
//
//  Format:
//  { "request" : "mazesolve",  "verbose" : INTEGER, "serial", INTEGER,
//      "regioncorner" : VECTOR, "pos": VECTOR, "rot" : QUATERNION, "cellsize": FLOAT, "probespacing" : FLOAT, 
//      "width" : FLOAT, "height" : FLOAT, 
//      "sizex", INTEGER, "sizey", INTEGER, 
//      "startx" : INTEGER, "starty" : INTEGER, "endx" : INTEGER, "endy" : INTEGER }
//
//  Globals
//
integer gMazeSerial = 0;                            // serial number of request.
vector gMazePos;                                    // position of current maze
rotation gMazeRot;                                  // rotation of current maze
float gMazeCellSize;                                // cell size of maze
//
//  mazesolverstart -- make a request of the maze solver.
//
//  Reply comes back later as a message.
//
//  ***GEOMETRY CALC NEEDS WORK***
//
integer mazesolverstart(vector p0, vector p1, float width, float height, float probespacing, integer verbose) 
{
    //  Lay out the rectangle for the maze
    integer MAXMAZESIZE = 41;                       // ***TEMP*** belongs elsewhere
    vector v = p1 - p0;                             // from start to goal
    float vdist = llVecMag(v);                      // distance from start to goal
    if (vdist < 0.01) { return(MAZESTATUSTOOSHORT); }              // too close, error
    vector pmid = (p0 + p1)*0.5;                    // center of maze area
    //  "pos" is the center of cell (0,0) of the maze.
    //  "cellsize" is the size of a maze cell. This must be larger than "width".
    //  We enlarge cell size so that there is an integer number of cells from p0 to p1.
    //  This works better if something upstream ensures a reasonable minimum distance between p0 and p1, like a meter.
    float cellsize = 1.5*width;                     // initial cell size
    float celldistfromstarttoend = vdist / cellsize;   // number of cells between start and end
    if (celldistfromstarttoend < 2) { return(MAZESTATUSTOOSHORT); } // start too close to end. Need to back off start and end points.
    integer cellsfromstarttoend = (integer)celldistfromstarttoend; // at least 2
    if (cellsfromstarttoend >= MAXMAZESIZE*0.75)    // too big
    {   return(MAZESTATUSTOOLONG); }
    //  OK, good to go.
    gMazeCellSize = vdist / cellsfromstarttoend;         // size of a cell so that start and end line up
    gMazeRot = rotperpenonground(p0, p1);           // rotation of center of maze
    //  For now, we always build a maze of MAXMAZESIZE*MAXMAZESIZE.
    //  The maze is aligned so that the X direction of the maze is from xstart to xend, the midpoint
    //  between xstart and xend is the center of the maze (roughly), and ystart and yend are halfway
    //  across the maze. 
    integer startx = (integer)(MAXMAZESIZE/2) - (integer)(cellsfromstarttoend / 2);
    integer starty = (integer)MAXMAZESIZE/2;
    integer endx = startx + cellsfromstarttoend;    // 
    integer endy = (integer)MAXMAZESIZE/2;
    vector p0inmaze = (<startx,starty,0>*gMazeCellSize) * gMazeRot;    // convert p0 back to world coords
    ////vector startrel = <startx*gMazeCellSize,starty*gMazeCellSize,0>; // vector from maze (0,0) to p0
    gMazePos = p0 - p0inmaze;                       // position of cell (0,0)
#define GEOMCHECK
#ifdef GEOMCHECK
    vector p0chk = gMazePos + (<startx,starty,0>*gMazeCellSize) * gMazeRot;    // convert p0 back to world coords
    vector p1chk = gMazePos + (<endx,endy,0>*gMazeCellSize) * gMazeRot; 
    if (llVecNorm(p1chk -p0chk) * llVecNorm(p1-p0) < 0.999)
    {   
        llSay(DEBUG_CHANNEL, "Maze geometry incorrect. Direction between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
        return(MAZESTATUSGEOMBUG);                                 // fails
    }

    if (llFabs(llVecMag(p1chk-p0chk) - llVecMag(p1-p0)) > 0.001)
    {   
        llSay(DEBUG_CHANNEL, "Maze geometry incorrect. Distance between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
        return(MAZESTATUSGEOMBUG);                                 // fails
    }
    if ((llVecMag(p0chk-p0) > 0.01) || (llVecMag(p1chk-p1) > 0.01))
    {   llSay(DEBUG_CHANNEL, "Maze geometry incorrect. p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
        return(MAZESTATUSGEOMBUG);                                 // fails
    }
#endif // GEOMCHECK

    gMazeSerial++;                                  // next maze number
    llMessageLinked(LINK_THIS, MAZESOLVEREQUEST, llList2Json(JSON_OBJECT, [
        "request", "mazesolve",                     // type of request
        "verbose", verbose,                         // debug use - maze solver will print messages
        "probespacing", probespacing,               // distance between ray casts in height dir
        "cellsize", gMazeCellSize,                  // size of a maze cell. Typically 0.333 meter
        "serial", gMazeSerial,                      // serial number for check
        "pos", gMazePos,                            // corner of maze in world space
        "rot", gMazeRot,                            // rotation of maze in world space
        "width", width,                             // avatar dimension
        "height", height,
        "sizex", MAXMAZESIZE,                       // maze dimensions in cells
        "sizey", MAXMAZESIZE,
        "startx", startx,                           // start, cell coords
        "starty", starty,
        "endx", endx,                               // goal, cell coords
        "endy", endy]),"");
    return(0);
}

//
//  mazesolveranswer -- maze solver has replied with a message, decode result
//
//  Result is either a list of vector waypoints, or a list with one integer status code.
//
list mazesolveranswer(string jsn, integer status) 
{
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "mazesolve") { return([MAZESTATUSFORMAT]); }              // ignore, not our msg
    string serial = llJsonGetValue(jsn, ["serial"]);
    if ((integer)serial != gMazeSerial) { return([MAZESTATUSCOMMSEQ]); }            // out of sequence 
    integer status = (integer)llJsonGetValue(jsn, ["status"]);      // get status from msg
    if (status != 0) { return([status]); }                  // error status from other side
    list ptsmaze = llJson2List(llJsonGetValue(jsn, ["points"])); // points, one per word
    list ptsworld = [];
    integer i;
    integer length = llGetListLength(ptsmaze);              // number of points
    for (i=0; i<length; i++)
    {   integer val = llList2Integer(ptsmaze,i);            // X and Y encoded into one integer
        llOwnerSay("Maze solve pt: (" + (string)mazepathx(val) + "," + (string)mazepathy(val) + ")");
        vector cellpos = gMazePos + (<mazepathx(val), mazepathy(val), 0>*gMazeCellSize)*gMazeRot;  // center of cell in world space
        ptsworld += [cellpos];                              // accum list of waypoints
    }
    return(ptsworld);
}

