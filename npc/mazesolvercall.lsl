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
#include "npc/pathbuildutils.lsl"
//
//  Format:
//  { "request" : "mazesolve",  "verbose" : INTEGER, 
//      "regioncorner" : VECTOR, "pos": VECTOR, "rot" : QUATERNION, "cellsize": FLOAT, "probespacing" : FLOAT, 
//      "width" : FLOAT, "height" : FLOAT, 
//      "sizex", INTEGER, "sizey", INTEGER, 
//      "startx" : INTEGER, "starty" : INTEGER, "endx" : INTEGER, "endy" : INTEGER }
//
//  mazesolverstart -- make a request of the maze solver.
//
//  Reply comes back later as a message.
//
mazesolverstart(vector p0, vector p1, float width, float height, float probespacing, integer verbose) 
{
    //  Lay out the rectangle for the maze
    vector v = p1 - p0;                             // from start to goal
    float vdist = llVecMag(v);                      // distance from start to goal
    vector pmid = (p0 + p1)*0.5;                    // center of maze area
    rotation rot = rotperpenonground(p0, p1);       // rotation of center of maze
    // ***MORE***
}

