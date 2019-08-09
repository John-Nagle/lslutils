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
#define MAZESTATUSBADSTART  -206                // can't find open space at start
#define MAZESTATUSBADOBSTACLE -208              // can't find open space after obstacle
#define MAZESTATUSBADBACKUP  -209               // backup to find space before obstacle hit beginning
#define MAZESTATUSNOPTS      -210               // empty path

                                                // internal errors
#define MAZESTATUSTIMEOUT   -501                // took too long
#define MAZESTATUSNOMEM     -502                // out of memory
#define MAZESTATUSLOOPING   -503                // runaway, didn't terminate
#define MAZESTATUSCASTFAIL  -506                // cast ray hard failure
                                                // bug traps
#define MAZESTATUSGEOMBUG   -1001               // Geometry setup failed

#endif // MAZEDEFSLSL
