//
//  stopallanims.lsl  -- stop all animations as clean up
//
default
{
    state_entry() 
    {
        list anims =  llGetObjectAnimationNames();
        integer i;
        for (i=0; i < llGetListLength(anims); i++)
        {   string s = llList2String(anims,i);
            llStopObjectAnimation(s);
        }
    }
}
