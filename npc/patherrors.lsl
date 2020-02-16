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
#define PATHERRMAZEOK        0                   // solved
#define PATHERRMAZENOFIND    -104                // no solution found, normal
                                                 // bad calls
#define PATHERRMAZEFORMAT    -101                // not valid message
#define PATHERRMAZECOMMSEQ   -102                // communications out of sequence
#define PATHERRMAZECOMMTIMEOUT -103              // communications failure

                                                 // bad mazes
#define PATHERRMAZETOOSHORT  -201                // end too close to start
#define PATHERRMAZETOOLONG   -202                // end too far from start                                           
#define PATHERRMAZEBADEND    -203                // end cell of maze is occupied
#define PATHERRMAZEBADSIZE   -204                // maze is too big
#define PATHERRMAZEBADCELLSIZE -205              // maze not a multiple of the cell size
#define PATHERRMAZEBADSTART  -206                // can't find open space at start
#define PATHERRMAZEBADOBSTACLE -208              // can't find open space after obstacle
#define PATHERRMAZEBADBACKUP  -209               // backup to find space before obstacle hit beginning
#define PATHERRMAZENOPTS      -210               // empty path
#define PATHERRMAZECELLCHANGED -211              // status of cell changed from unoccupied to occupied
#define PATHERRMAZEBADZ      -212                // Z value not near maze end. Probably multi-level maze failure
#define PATHERRMAZEBACKWARDS -213                // backwards move trying to get clear of obstacle, do a retry

                                                 // internal errors
#define PATHERRMAZETIMEOUT   -501                // took too long
#define PATHERRMAZENOMEM     -502                // out of memory
#define PATHERRMAZELOOPING   -503                // runaway, didn't terminate
#define PATHERRMAZECASTFAIL  -506                // cast ray hard failure
#define PATHERRMAZEKFMSTALL  -507                // not moving in KFM mode
#define PATHERRMAZEBADRECOV  -508                // recovery  move requested while moving
                                                 // bug traps
#define PATHERRMAZEGEOMBUG   -1001               // Geometry setup failed

//  Path planning and execution errors
//
#define PATHERRBADPATH1     -2001
#define PATHERRSEGOUTOFSEQ2 -2002
#define PATHERRSEGOUTOFSEQ1 -2003
#define PATHERRBADSTARTPOS  -2004
#define PATHERRCOLLISION    -2005               // hit something
#define PATHERRNOTWALKABLE  -2006               // can't find walkable surface under point
#define PATHERROBSTRUCTED   -2007               // obstruction ahead
#define PATHERRBADMOVEEND   -2008               // out of position at end of move
#define PATHERREMPTYPATH    -2009               // empty path - how did that happen?
#define PATHERRTARGETMOVED  -2010               // target avatar moved during pursue
#define PATHERRTARGETGONE   -2011               // target avatar disappeared.
#define PATHERRBADDEST      -2012               // bad destination - out of parcel, etc.
#define PATHERROBSTRUCTEDSTART     -2013        // starting point of entire path is occupied.
#define PATHERRSTOPREQ      -2014               // behavior requested a stop
#define PATHERRWALKABLEFAIL -2015               // tried to recover from non-walkable area but failed
#define PATHERRWALKABLEFIXED -2016              // successful recovery from non-walkable area problem
#define PATHERRWALKABLETROUBLE -2017            // at non-walkable location, must try recovery
#define PATHERRREQOUTOFSYNC  -2018              // request at move level while doing something else
#define PATHERROFFPATH       -2019              // off the path in move. Retry.
#define PATHERRPROHIBITED    -2020              // Point is in a prohibited area (cannot enter parcel)
#define PATHERRREGIONCROSS   -2021              // crossed a region boundary. Retry will pick up in the new region

//
//  List of retryable errors.  For these, try again, if there was progress on the previous try.
//
#define PATHRETRYABLES [PATHERROBSTRUCTED, PATHERRCOLLISION, PATHERRBADMOVEEND, PATHERRBADSTARTPOS, PATHERRTARGETMOVED,\
    PATHERRMAZENOMEM, PATHERRWALKABLEFIXED, PATHERRMAZEBACKWARDS,PATHERRREQOUTOFSYNC, PU_FAILURE_INVALID_START,\
    PATHERRWALKABLETROUBLE,PATHERROFFPATH, PATHERRREGIONCROSS]
    
//
//  For these errors, try the recovery sequence.
//
    
#define PATHRECOVERABLES [PU_FAILURE_INVALID_START]

#endif // PATHERRORS
