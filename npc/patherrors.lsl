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
#define MAZESTATUSCELLCHANGED -211              // status of cell changed from unoccupied to occupied
#define MAZESTATUSBADZ      -212                // Z value not near maze end. Probably multi-level maze failure

                                                // internal errors
#define MAZESTATUSTIMEOUT   -501                // took too long
#define MAZESTATUSNOMEM     -502                // out of memory
#define MAZESTATUSLOOPING   -503                // runaway, didn't terminate
#define MAZESTATUSCASTFAIL  -506                // cast ray hard failure
#define MAZESTATUSKFMSTALL  -507                // not moving in KFM mode
                                                // bug traps
#define MAZESTATUSGEOMBUG   -1001               // Geometry setup failed

//  Path planning and execution errors
//
#define PATHEXEBADPATH1     -2001
#define PATHEXESEGOUTOFSEQ2 -2002
#define PATHEXESEGOUTOFSEQ1 -2003
#define PATHEXEBADSTARTPOS  -2004
#define PATHEXECOLLISION    -2005               // hit something
#define PATHEXENOTWALKABLE  -2006               // can't find walkable surface under point
#define PATHEXEOBSTRUCTED   -2007               // obstruction ahead
#define PATHEXEBADMOVEEND   -2008               // out of position at end of move
#define PATHEXEEMPTYPATH    -2009               // empty path - how did that happen?
#define PATHEXETARGETMOVED  -2010               // target avatar moved during pursue
#define PATHEXETARGETGONE   -2011               // target avatar disappeared.
#define PATHEXEBADDEST      -2012               // bad destination - out of parcel, etc.
#define PATHEXEOBSTRUCTEDSTART     -2013               // starting point of entire path is occupied.
#define PATHEXESTOPREQ      -2014               // behavior requested a stop
#define PATHEXEWALKABLEFAIL -2015               // tried to recover from non-walkable area but failed
#define PATHEXEWALKABLEFIXED -2016              // successful recovery from non-walkable area problem

//
//  List of retryable errors.  For these, try again, if there was progress on the previous try.
//
#define PATHRETRYABLES [PATHEXEOBSTRUCTED, PATHEXECOLLISION, PATHEXEBADMOVEEND, PATHEXEBADSTARTPOS, PATHEXETARGETMOVED, MAZESTATUSNOMEM, PATHEXEWALKABLEFIXED]

#endif // PATHERRORS
