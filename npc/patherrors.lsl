//
//  patherrors.lsl -- error codes from the path system
//
//
//  Animats
//  August, 2019
//
//  License: GPLv3
//
//
#ifndef PATHERRORS
#define PATHERRORS
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
#define MAZESTATUSKFMSTALL  -507                // not moving in KFM mode
                                                // bug traps
#define MAZESTATUSGEOMBUG   -1001               // Geometry setup failed

//  Path execution errors
//
#define PATHEXEBADPATH1     -2001
#define PATHEXESEGOUTOFSEQ2 -2002
#define PATHEXESEGOUTOFSEQ1 -2003
#define PATHEXEBADSTARTPOS  -2004
#define PATHEXECOLLISION    -2005
#define PATHEXENOTWALKABLE  -2006               // can't find walkable surface under point
#define PATHEXEOBSTRUCTED   -2007               // obstruction ahead

#endif // PATHERRORS
