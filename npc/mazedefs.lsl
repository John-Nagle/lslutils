//
//  mazedefs.lsl -- definitions for maze solver and path planner
//
//  Animats
//
//  June, 2019
//
#ifndef MAZEDEFSLSL
#define MAZEDEFSLSL
#include "npc/assert.lsl"
#include "npc/patherrors.lsl"        
//
//   Maze path storage - X && Y in one 32-bit value
//
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value

#ifndef INFINITY                                // should be an LSL builtin
#define INFINITY ((float)"inf")                             // is there a better way?
#endif // INFINITY
#define PATHMAXUNSIGNED 2147483647              // 2^31-1, largest positive integer
//
//  Link message types
//
#define PATHMASTERRESET 100                     // reset all path scripts
#define PATHPLANREQUEST 101                     // character controller to path planner                  
#define PATHPLANREPLY 102                       // path planner to character controller

#define MAZESOLVEREQUEST 201                    // from path planner to maze solver
#define MAZESOLVERREPLY 202                     // from maze solver to execution
#define MAZEPATHREPLY 203                       // from path planner to execution
#define MAZEPATHSTOP 204                        // from path planner to execution




#define MAZEMAXSIZE (41)                        // maximum size of maze



#endif // MAZEDEFSLSL
