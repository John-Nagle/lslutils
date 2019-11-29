#ifndef MATHUTILSLSL
#define MATHUTILSLSL
//
//  Misc. support functions. For general use.
//
//  No dependencies on other files.
//
#ifndef INFINITY
#define INFINITY ((float)"inf")                 // ought to be a builtin
#endif // INFINITY

#ifndef RotFromXAxis 
//  RotFromXAxis -- rotation from X axis in XY plane.
#define RotFromXAxis(dv) llAxes2Rot(llVecNorm(dv),<0,0,1>%llVecNorm(dv),<0,0,1>)
#endif // RotFromXAxis
//
//  Usual SLERP function for quaternion interpolation
//
rotation slerp(rotation a, rotation b, float t) {
   return llAxisAngle2Rot( llRot2Axis(b /= a), t * llRot2Angle(b)) * a;
}    

integer rand_int(integer bound)                 // bound must not exceed 2^24.
{   return((integer)llFrand(bound)); }          // get random integer                   

vector vec_to_target(key id)         
{   return(target_pos(id) - llGetPos());   }         

vector target_pos(key id)
{   list v = llGetObjectDetails(id, [OBJECT_POS]);  
    if (v == []) { return(ZERO_VECTOR); }         // not really a good choice for fails  
    return(llList2Vector(v,0));                             
}

float dist_to_target(key id)
{   
    list v = llGetObjectDetails(id, [OBJECT_POS]);  
    if (v == []) { return(INFINITY); }         // if gone, infinitely far away
    return(llVecMag(llList2Vector(v,0) - llGetPos()));  // distance to target                            
}

vector dir_from_target(key id)                      
{
    list r = llGetObjectDetails(id, [OBJECT_ROT]);  
    if (r == []) { return(ZERO_VECTOR); }           
    rotation arot = llList2Rot(r,0);                
    vector facingdir = <1,0,0>*arot;
    facingdir.z = 0.0;                              
    return(llVecNorm(facingdir));                   
}

//
//  is_active_obstacle -- true if obstacle might move.
//
integer is_active_obstacle(key id)
{   if (id == "" || id == NULL_KEY) { return(FALSE); }          // no object
    //  Guess if this is a live object.
    list details = llGetObjectDetails(id, [OBJECT_VELOCITY, OBJECT_PHYSICS, OBJECT_PATHFINDING_TYPE, OBJECT_ANIMATED_COUNT]);
    integer pathfindingtype = llList2Integer(details,2);            // get pathfinding type
    if (pathfindingtype == OPT_AVATAR || pathfindingtype == OPT_CHARACTER) { return(TRUE); } // definitely alive, yes
    if (pathfindingtype != OPT_LEGACY_LINKSET) { return(FALSE); }                           // if definitely static, no.
    if (llVecMag(llList2Vector(details,0)) > 0.0 || llList2Integer(details,1) != 0 || llList2Integer(details,3) > 0) { return(TRUE); } // moving or physical or animesh
    //  Need a really good test for KFM objects.
    return(FALSE);                                                      // fails, for now.
}


//
//   linear_interpolate  -- simple linear interpolation
//
float linear_interpolate(float n1 , float n2 , float fract )
{   return n1 + ( (n2-n1) * fract );    } 

//
//  easeineaseout  --  interpolate from 0 to 1 with ease in and ease out using cubic Bezier.
//
//  ease = 0: no smoothing
//  ease = 0.5: reasonable smoothing
//
float easeineaseout(float ease, float fract)
{   float ym = linear_interpolate(0, ease, fract);
    float yn = linear_interpolate(ease, 1, fract);
    float y =  linear_interpolate(ym, yn,  fract);
    return(y);
}
#endif // MATHUTILSLSL
