//
//  Touch tester with line of sight check. Demo.
//
//  John Nagle
//  Animats
//  2019
//
//  License: GPLv3.
//
//
//  freetouchpath - is there a free path from the touching avatar to the touch point?
//
//  Returns empty string if yes, name of interfering object if no.
//
string freetouchpath(integer detected_num)
{
    key avi = llDetectedKey(detected_num);                 // first toucher
    vector avpos = llDetectedPos(detected_num);            // position of touching avi
    vector touchpos = llDetectedTouchPos(detected_num);    // position of touch
    list raycast = llCastRay(avpos, touchpos, []);         // check space between them
    integer status = llList2Integer(raycast,-1);    // last element is status
    if (status == 0)                            // ray hit nothing
    {   return(""); }                           // no obstacle
    //  Hard case. Check for ray hitting avi or object being touched. Those don't count.
    integer linknum = llDetectedLinkNumber(detected_num);  // which link was touched
    key linkkey = llGetLinkKey(linknum);        // key of link touched
    integer i;
    for (i=0; i<llGetListLength(raycast)-1; i = i+2)// list obstacles
    {   key obstacle = llList2Key(raycast,i);   // key of obstacle
        if (obstacle != linkkey && obstacle != avi)  // not false alarm
        {   string s = llKey2Name(obstacle);        // name of obstacle
            if (s == "") { s = "UUID " + (string)obstacle; }  // name lookup failed
            return(s);                           // return name of obstacle
        }
    }
    return("");                                 // true if no problem
}

default
{

    touch_start(integer total_number)
    {
        string obstruction = freetouchpath(0);      // touched. Is it valid?
        if (obstruction == "")
        {   llSay(0, "Touched"); }                  // Yes.
        else
        {   llSay(0, "Can't touch this! " + obstruction + " is in the way."); } // No
    } 
}

