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
//      "startx" : INTEGER, "starty" : INTEGER, "startclear": BOOLEAN, "endx" : INTEGER, "endy" : INTEGER }
//
//  Globals
//
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
integer mazesolverstart(vector p0, vector p1, float width, float height, float probespacing, integer startclear, integer pathid, integer segmentid, integer msglev) 
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
    //  This happens sometimes, due to some numeric error not yet found.
    if ((llVecMag(p0chk-p0) > 0.02) || (llVecMag(p1chk-p1) > 0.02))        
    {   pathMsg(PATH_MSG_ERROR,"Maze geometry incorrect. p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
    }
#endif // GEOMCHECK

    llMessageLinked(LINK_THIS, MAZESOLVEREQUEST, llList2Json(JSON_OBJECT, [
        "request", "mazesolve",                     // type of request
        "msglev", msglev,                           // debug use - maze solver will print messages
        "probespacing", probespacing,               // distance between ray casts in height dir
        "cellsize", gMazeCellSize,                  // size of a maze cell. Typically 0.333 meter
        "pathid", pathid,                           // path we are working on
        "segmentid", segmentid,                     // segment ID we are on
        "pos", gMazePos,                            // corner of maze in world space
        "rot", gMazeRot,                            // rotation of maze in world space
        "width", width,                             // avatar dimension
        "height", height,                        
        "sizex", MAXMAZESIZE,                       // maze dimensions in cells
        "sizey", MAXMAZESIZE,
        "startx", startx,                           // start, cell coords
        "starty", starty,
        "startclear", startclear,                   // assume start point is unobstructed
        "endx", endx,                               // goal, cell coords
        "endy", endy]),"");
    return(0);
}

