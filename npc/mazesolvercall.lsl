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
//  mazecellto3d  -- convert maze cell to 3D coords
//
//  gMazeCellSize is the size of a cell in the XY plane, not the 3D plane
//  gMazePos is the position of cell (0,0);
//  gMazeRot is the rotation of the maze plane.
//
//  ***NEEDS WORK*** ***WRONG*** ***ASSERTION FAILS***
//  ***THIS IS OFF BY 1-2%. WHY???***
//
vector mazecellto3d(integer x, integer y)
{
    if (x == 0 && y == 0) { return(gMazePos); }     // avoid divide by zero
    vector vflat = <x*gMazeCellSize,y*gMazeCellSize,0.0>;   // vector to cell in XY plane
    vector dir = llVecNorm(vflat*gMazeRot);         // 3D direction to point, but wrong dist
    float flatlen = llVecMag(vflat);                // vector length in 2D
    vector dirflat = llVecNorm(<dir.x,dir.y,0.0>);  // azimuth vector
    //  Now we need to scale up.
    float scale = dir*dirflat;                      // scale-down factor
    vector p = (flatlen/scale) * dir;               // scale correct dir to fit 2D X and Y. 
    //  Checks
    vector azimuthvec = <1,0,0>*gMazeRot;           // rotation 
    azimuthvec = llVecNorm(<azimuthvec.x, azimuthvec.y,0.0>);
    rotation azimuthrot = llRotBetween(<1,0,0>,azimuthvec);
    vector vflatrot = vflat*azimuthrot;             // vector in XY plane
    //  Vflatrot has correct X and Y. Now we need Z.
    vector planenormal = <0,0,1>*gMazeRot;          // normal to rotated plane. Plane is through origin here.
    //  Distance from point to plane is p*planenormal.  We want that to be zero.
    p = vflatrot;
    //  We want p.z such that p*planenormal = 0;
    //  want p.x*planenormal.x + p.y * planenormal.y + p.z * planenormal.z = 0
    //  want p.x*planenormal.x + p.y * planenormal.y = -p.z * planenormal.z
    p.z = - (p.x*planenormal.x + p.y * planenormal.y)/planenormal.z;    // planenormal.z cannot be zero unless tilted plane is vertical
    DEBUGPRINT1("mazecellto3d: x: " + (string)x + " y: " + (string)y + " p: " + (string)p + " vflatrot: " + (string)vflatrot);
#ifdef NOGOOD // This doesn't work at all

    assert(llFabs(p.x - vflatrot.x) < 0.01);        // check X and Y ***WRONG***
    assert(llFabs(p.y - vflatrot.y) < 0.01);  
#endif // NOGOOD
    return(p + gMazePos);
}
//
//  mazesolverstart -- make a request of the maze solver.
//
//  Reply comes back later as a message.
//
//  p0-p1 distance must be an integral number of widths.
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
    //  "cellsize" is the size of a maze cell. This must be the same as width.
    //  p0 to p1 in the XY plane must be an integral number of widths.
#ifdef OBSOLETE
    //  We enlarge cell size so that there is an integer number of cells from p0 to p1.
    //  This works better if something upstream ensures a reasonable minimum distance between p0 and p1, like a meter.
    float cellsize = 1.5*width;                     // initial cell size
    float celldistfromstarttoend = vdist / cellsize;   // number of cells between start and end
#endif // OBSOLETE
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
    integer endx = startx + unitcells;    // 
    integer endy = starty;
    ////vector p0inmaze = (<startx,starty,0>*gMazeCellSize) * gMazeRot;    // convert p0 back to world coords
    gMazePos = ZERO_VECTOR;                                                // temporary for this maze cell calc
    vector p0inmaze = mazecellto3d(startx, starty);                        // convert back to 3D coords 

    ////vector startrel = <startx*gMazeCellSize,starty*gMazeCellSize,0>; // vector from maze (0,0) to p0
    gMazePos = p0 - p0inmaze;                       // position of cell (0,0)
#define GEOMCHECK
#ifdef GEOMCHECK
    //  ***WRONG? - gMazeCellSize is in the XY plane, not for tilted cells.*** Trying new calc
    ////vector p0chk = gMazePos + (<startx,starty,0>*gMazeCellSize) * gMazeRot;    // convert p0 back to world coords
    vector p0chk = mazecellto3d(startx, starty);                        // convert back to 3D coords 

    ////vector mazedirflat = llVecNorm(p1.x-p0.x, p1.y-p0.y,0.0>;   // p0 to p1 in XY plane
    ////rotation mazerotflat = llRotBetween(<1,0,0>,mazedirflat);  // rotate vec in XY plane
    ////vector p1chkflat = <gMazePos.x,gMazePos.y, 0.0> + (<endx,endy,0>*gMazeCellSize)*mazerotflat;    // X and Y for p1chk
    //  Need to compute Z for p1chk ***MORE***
    ////p1chkdirflat = llVecNorm((<endx,endy,0>*gMazeCellSize) * gMazeRot); // dir to p1 in XY plane
    ////vector p1chk = gMazePos + (<endx,endy,0>*gMazeCellSize) * gMazeRot; // ***WRONG***
    vector p1chk = mazecellto3d(endx, endy);                        // convert back to 3D coords 
    ////p1chk = p1chkflat; // ***TEMP TEST*** Z is wrong
    if (llVecNorm(p1chk -p0chk) * llVecNorm(p1-p0) < 0.999)
    {   
        panic("Maze geometry incorrect. Direction between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
    }

    if (llFabs(llVecMag(p1chk-p0chk) - llVecMag(p1-p0)) > 0.01)
    {   
        panic("Maze geometry incorrect. Distance between p0: " + (string)p0 + " differs from p0chk: " + (string)p0chk + " or p1 : " 
        + (string)p1 + " differs from p1chk: " + (string) p1chk);
    }
    if ((llVecMag(p0chk-p0) > 0.05) || (llVecMag(p1chk-p1) > 0.05))         // allow 5cm error. Rounding problem?
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
        vector cellpos = gMazePos + (<mazepathx(val), mazepathy(val), 0>*gMazeCellSize)*gMazeRot;  // center of cell in world space
        ptsworld += [cellpos];                              // accum list of waypoints
    }
    return(ptsworld);
}

