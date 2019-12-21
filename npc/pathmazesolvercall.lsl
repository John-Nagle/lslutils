//
//  pathmazesolvercall.lsl
//
//  Finds reasonable path through a grid of squares with obstacles.
//
//  Animats
//  June, 2019
//
//  This file is used by callers of the maze solver. The maze solver
//  itself is in a separate script, for memory size reasons.
//
#include "npc/pathmazedefs.lsl"
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
//  The start and end cells are assumed to be clear. The caller must check that.
//
//
integer mazesolverstart(vector p0, vector p1, float width, float height, integer chartype, float probespacing, key hitobj, integer pathid, integer segmentid) 
{
    //  Lay out the rectangle for the maze
    integer MAXMAZESIZE = 41;                           // ***TEMP*** belongs elsewhere
    ////vector v = p1 - p0;                             // from start to goal
    vector dv = p1-p0;                                  // direction from start to end of maze
    dv.z = 0.0;
    float flatdist = llVecMag(dv);                      // distance from start to goal
    if (flatdist < 0.01) { return(PATHERRMAZETOOSHORT); }// too close, error
    //  "pos" is the center of cell (0,0) of the maze.
    //  "cellsize" is the size of a maze cell. This must be the same as width.
    //  p0 to p1 in the XY plane must be an integral number of widths.
    float cellsize = width;                         // just use width
    integer unitcells = (integer)(flatdist/cellsize+0.001);   // integral number of cells
    if (unitcells < 2) { return(PATHERRMAZETOOSHORT); } // start too close to end. Need to back off start and end points.
    if (unitcells >= MAXMAZESIZE*0.75)    // too big
    {   return(PATHERRMAZETOOLONG); }
    if (llFabs(unitcells*cellsize - flatdist) > 0.01)   // not a multiple of width
    {   return(PATHERRMAZEBADCELLSIZE); }
    //  OK, good to go.
    gMazeCellSize = cellsize;                       // size of a cell so that start and end line up
    ////gMazeRot = RotBetween((<1,0,0>), dv);             // rotation of maze coord system in XY plane - NO GOOD - bug in RotBetween
    gMazeRot = RotFromXAxis(dv);                    // rotation of maze coord system in XY plane

    
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
    gMazePos = p0 - p0inmaze;                       // position of cell (0,0)
#define GEOMCHECK
#ifdef GEOMCHECK
    vector p0chk = mazecellto3d(startx, starty, gMazeCellSize, gMazePos, gMazeRot);                        // convert back to 3D coords 
    vector p1chk = mazecellto3d(endx, endy, gMazeCellSize, gMazePos, gMazeRot);                        // convert back to 3D coords 
    p1chk.z = p1.z;                                     // Z doesn't have to match. Z is not part of the transformation here
    //  This used to happen sometime, due to problems with llRotBetween.
    assert(llVecMag(p0chk-p0) < 0.001);
    assert(llVecMag(p1chk-p1) < 0.001);
#endif // GEOMCHECK
    pathMsg(PATH_MSG_INFO, "Sending job to maze solver, pathid: " + (string)pathid + " segmentid: " + (string)segmentid);
    llMessageLinked(LINK_THIS, MAZESOLVEREQUEST, llList2Json(JSON_OBJECT, [
        "request", "mazesolve",                     // type of request
        "probespacing", probespacing,               // distance between ray casts in height dir
        "cellsize", gMazeCellSize,                  // size of a maze cell. Typically 0.333 meter
        "pathid", pathid,                           // path we are working on
        "segmentid", segmentid,                     // segment ID we are on
        "pos", gMazePos,                            // corner of maze in world space
        "rot", gMazeRot,                            // rotation of maze in world space
        "width", width,                             // avatar dimension
        "height", height,                        
        "chartype", chartype,                       // character type, for static path use
        "hitobj", hitobj,                           // obstacle that started maze solve
        "sizex", MAXMAZESIZE,                       // maze dimensions in cells
        "sizey", MAXMAZESIZE,
        "startx", startx,                           // start, cell coords
        "starty", starty,
        "startz", p0.z,                             // float, starting Z position
        "endx", endx,                               // goal, cell coords
        "endy", endy,
        "endz", p1.z,
        "p0", p0,                                   // for checking purposes only
        "p1", p1                                    // for checking purposes only
        ]),"");
    return(0);
}

