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

                                                // internal errors
#define MAZESTATUSTIMEOUT   -501                // took too long
#define MAZESTATUSNOMEM     -502                // out of memory
#define MAZESTATUSLOOPING   -503                // runaway, didn't terminate
#define MAZESTATUSCASTFAIL  -506                // cast ray hard failure
                                                // bug traps
#define MAZESTATUSGEOMBUG   -1001               // Geometry setup failed

////#define DEBUG
#ifdef DEBUG
#define DEBUGPRINT(s) // Nothing for now
#define DEBUGPRINT1(s) llOwnerSay(s)
////#define assert(exp) // Nothing for now
#define assert(exp) { if (!(exp)) { llOwnerSay("Assertion failed at " + __SHORTFILE__ + " line " + (string) __LINE__); panic(); }}
#else // not debugging
#define DEBUGPRINT(s) {}
#define DEBUGPRINT1(s) {}
#define assert(exp) {}
#endif // DEBUG



#endif // MAZEDEFS
