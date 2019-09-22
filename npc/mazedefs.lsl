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
#ifdef OBSOLETE
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value
#endif // OBSOLETE
//  X and Y are integer cell coordinates, 0..255.
//  Z is the offset in Z from "refz", stored as an integer in 0.01m units offset by 08000x, range +-327m. 
#define mazepathx(val) (((val) >> 24) & 0xff)   // X is high byte
#define mazepathy(val) (((val) >> 16) & 0xff)   // Y is second byte
#define mazepathz(val, refz) ((float)(((val) & 0xffff)-0x8000)*0.01 + refz)
#define mazepathconstruct(x,y,z,refz) (((((x) & 0xff))<<24) | (((y)&0xff) << 16) | ((((integer)(((z)-(refz))*100.0))+0x8000)&0xffff))
#define mazepathcomparexy(p1,p2) (((p1)>>16) == ((p2)>> 16))        // comparison in X and Y only.
#define mazepathval(x,y) (mazepathconstruct(x,y,0,0))   // ***TEMP*** backwards compatible while we put in Z info

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
