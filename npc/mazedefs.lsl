//
//  mazedefs.lsl -- definitions for maze solver
//
//  Animats
//
//  June, 2019
//
#ifndef MAZEDEFS
#define MAZEDEFS        
//
//   Maze path storage - X && Y in one 32-bit value
//
#define mazepathx(val) ((val) & 0xffff)         // X is low half
#define mazepathy(val) (((val)>> 16) & 0xffff)  // Y is high half
#define mazepathval(x,y) (((y) << 16) | (x))    // construct 32 bit value

#define MAZEMAXSIZE (41)                                    // maximum size of maze

#endif // MAZEDEFS
