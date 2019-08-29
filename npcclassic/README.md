# Old-stype path planning for non-player characters in Second Life

John Nagle

Animats

June, 2019


## Legacy demo
This is our classic pathfinding system. It uses SL pathfinding, and adds extensive error
recovery. This will perform reasonably well in non-overloaded regions. In overloaded 
regions down to 25% pathfinding steps executed, it will still perform adequately most 
of the time. Below 25%, errors will occur. The characters will shift to very slow movement
in seriously overloaded sims.
