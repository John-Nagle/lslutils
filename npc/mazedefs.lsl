//
//  mazedefs.lsl -- definitions for maze solver
//
//  Animats
//
//  June, 2019
//
#ifndef MAZEDEFSLSL
#define MAZEDEFSLSL
#include "npc/assert.lsl"        
//
//   Maze path storage - X && Y in one 32-bit value
//
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value

#ifndef INFINITY                                // should be an LSL builtin
#define INFINITY ((float)"inf")                             // is there a better way?
#endif // INFINITY

//  Message types
//
#define MAZESOLVEREQUEST 201                    // to maze solver
#define MAZESOLVERREPLY 202                     // from maze solver

#define MAZEMAXSIZE (41)                                    // maximum size of maze

//
//  Error codes from maze solving
//
#define MAZESTATUSOK        0                   // solved
#define MAZESTATUSNOFIND    -1                  // no solution found, normal
                                                // bad calls
#define MAZESTATUSFORMAT    -101                // not valid message
#define MAZESTATUSCOMMSEQ   -102                // communications out of sequence
#define MAZESTATUSCOMMTIMEOUT -103              // communications failure

                                                // bad mazes
#define MAZESTATUSTOOSHORT  -201                // end too close to start
#define MAZESTATUSTOOLONG   -202                // end too far from start                                           
#define MAZESTATUSBADEND    -203                // end cell of maze is occupied
#define MAZESTATUSBADSIZE   -204                // maze is too big
#define MAZESTATUSBADCELLSIZE -205              // maze not a multiple of the cell size

                                                // internal errors
#define MAZESTATUSTIMEOUT   -501                // took too long
#define MAZESTATUSNOMEM     -502                // out of memory
#define MAZESTATUSLOOPING   -503                // runaway, didn't terminate
#define MAZESTATUSCASTFAIL  -506                // cast ray hard failure
                                                // bug traps
#define MAZESTATUSGEOMBUG   -1001               // Geometry setup failed

//
//  mazecellto3d  -- convert maze cell to 3D coords
//
//  mazecellsize is the size of a cell in the XY plane, not the 3D plane
//  mazepos is the position of cell (0,0);
//  mazerot is the rotation of the maze plane.
//
//  Used in multiple scripts.
//
vector mazecellto3d(integer x, integer y, float mazecellsize, vector mazepos, rotation mazerot)
{
    if (x == 0 && y == 0) { return(mazepos); }      // avoid divide by zero
    vector vflat = <x*mazecellsize,y*mazecellsize,0.0>;   // vector to cell in XY plane
    //  Calc X and Y in 2D space.
    vector azimuthvec = <1,0,0>*mazerot;           // rotation 
    azimuthvec = llVecNorm(<azimuthvec.x, azimuthvec.y,0.0>);   
    rotation azimuthrot = llRotBetween(<1,0,0>,azimuthvec);
    vector p = vflat*azimuthrot;             // vector in XY plane
    //  Vflatrot has correct X and Y. Now we need Z.
    vector planenormal = <0,0,1>*mazerot;          // normal to rotated plane. Plane is through origin here.
    //  Distance from point to plane is p*planenormal.  We want that to be zero.
    //  We want p.z such that p*planenormal = 0;
    //  want p.x*planenormal.x + p.y * planenormal.y + p.z * planenormal.z = 0
    p.z = - (p.x*planenormal.x + p.y * planenormal.y)/planenormal.z;    // planenormal.z cannot be zero unless tilted plane is vertical
    DEBUGPRINT1("mazecellto3d: x: " + (string)x + " y: " + (string)y + " p: " + (string)p + " p+mazepos: " + (string)(p+mazepos));
    return(p + mazepos);
}




#endif // MAZEDEFSLSL
