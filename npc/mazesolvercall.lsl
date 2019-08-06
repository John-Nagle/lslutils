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
//  p0-p1 distance must be an integral number of widths.
//
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
    //  "cellsize" is the size of a maze cell. This must be the same as width.
    //  p0 to p1 in the XY plane must be an integral number of widths.
    float cellsize = width;                         // just use width
    float flatdist = llVecMag(<p0.x,p0.y,0.0> - <p1.x,p1.y,0.0>);   // distance in 2D plane
    integer unitcells = (integer)(flatdist/cellsize+0.001);   // integral number of cells
    if (unitcells < 2) { return(MAZESTATUSTOOSHORT); } // start too close to end. Need to back off start and end points.
    if (unitcells >= MAXMAZESIZE*0.75)    // too big
    {   return(MAZESTATUSTOOLONG); }
    if (llFabs(unitcells*cellsize - flatdist) > 0.01)   // not a multiple of width
    {   return(MAZESTATUSBADCELLSIZE); }
    //  OK, good to go.
    gMazeCellSize = cellsize;                       // size of a cell so that start and end line up
    gMazeRot = rotperpenonground(p0, p1);           // rotation of center of maze
    //  For now, we always build a maze of MAXMAZESIZE*MAXMAZESIZE.
    //  The maze is aligned so that the X direction of the maze is from xstart to xend, the midpoint
    //  between xstart and xend is the center of the maze (roughly), and ystart and yend are halfway
    //  across the maze. 
    integer startx = (integer)(MAXMAZESIZE/2) - (integer)(unitcells / 2);
    integer starty = (integer)(MAXMAZESIZE/2);
    integer endx = startx + unitcells;              // end is unitcells from startx in X dir
    integer endy = starty;
    vector p0inmaze = mazecellto3d(startx, starty, gMazeCellSize, ZERO_VECTOR, gMazeRot);      // convert back to 3D coords relative to maze space 

    //  Calculate base pos of maze.
    //  ***MAY BE WRONG***
    gMazePos = p0 - p0inmaze;                       // position of cell (0,0)
#define GEOMCHECK
#ifdef GEOMCHECK
    vector p0chk = mazecellto3d(startx, starty, gMazeCellSize, gMazePos, gMazeRot);                        // convert back to 3D coords 
    vector p1chk = mazecellto3d(endx, endy, gMazeCellSize, gMazePos, gMazeRot);                        // convert back to 3D coords 
    if (llVecNorm(p1chk -p0chk) * llVecNorm(p1-p0) < 0.999)
    {   
        panic("Maze geometry incorrect. Direction between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
    }

    if (llFabs(llVecMag(p1chk-p0chk) - llVecMag(p1-p0)) > 0.001)
    {   
        panic("Maze geometry incorrect. Distance between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
    }
    if ((llVecMag(p0chk-p0) > 0.01) || (llVecMag(p1chk-p1) > 0.01))        
    {   panic("Maze geometry incorrect. p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
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
        ////vector cellpos = gMazePos + (<mazepathx(val), mazepathy(val), 0>*gMazeCellSize)*gMazeRot;  // center of cell in world space
        vector cellpos = mazecellto3d(mazepathx(val), mazepathy(val), gMazeCellSize, gMazePos, gMazeRot);                        // convert back to 3D coords 
        ptsworld += [cellpos];                              // accum list of waypoints
    }
    return(ptsworld);
}
//
//  Path builder task interface
//
integer gPathSerial = 0;                                        // path build serial number
integer gPathSectionSerial = 0;                                 // serial number of path section
//
//  pathbuildstart -- make a request of the path builder
//
//  Reply comes back later as multiple messages.
//
//
integer pathbuildstart(vector p0, vector p1, float width, float height, integer mustreachgoal, integer verbose) 
{
    gPathSerial++;
    gPathSectionSerial = 0;                         // segments in order
    llMessageLinked(LINK_THIS, PATHBUILDREQUEST, llList2Json(JSON_OBJECT, [
        "request", "pathbuild",                     // type of request
        "verbose", verbose,                         // debug use - maze solver will print messages
        "mustreachend", mustreachgoal,              // if true, must reach goal. If false, best effort 
        "p0", p0,
        "p1", p1,
        "serial", gPathSerial,                      // serial number for check
            ]),"");
    return(0);
}

//
//  pathbuildanswer -- path builder replied with a message, decode result
//
//  Result is one path section, or a one-element list with a status.
//  A path segment is a maze start point followed by a list of safe points.
//  If the first point is ZERO_VECTOR, a maze solve is not needed.
//
list pathbuildanswer(string jsn) 
{
    string requesttype = llJsonGetValue(jsn,["reply"]);   // request type
    if (requesttype != "pathbuild") { return([MAZESTATUSFORMAT]); }                 // ignore, not our msg
    string serial = llJsonGetValue(jsn, ["serial"]);
    if ((integer)serial != gPathSerial) { return([MAZESTATUSCOMMSEQ]); }            // out of sequence 
    integer status = (integer)llJsonGetValue(jsn, ["status"]);                      // get status from msg
    if (status != 0) { return((integer)status); }                   /               // error status from other side
    integer segserial = (integer)llJsonGetValue(jsn,["segment"]);
    if (segserial != gPathSegmentSerial) { return([MAZESTATUSCOMMSEQ]); }           // sections out of sequence
    gPathSectionSerial++;                                                           // ready for next segment
    return(llJson2List(llJsonGetValue(jsn, ["points"])));                           // points, one per word
}

