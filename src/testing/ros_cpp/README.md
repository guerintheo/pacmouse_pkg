To compile everything, go to `~/ros_catkin_ws` and run `catkin_make`.

I'm getting a permissions error when I run `rosrun pacmouse_pkg cpp_test`, but when I try `sudo rosrun` it says `rosrun: command not found`. WTF? 





I think I'm importing the `Drive.h` file correctly with `#include "pacmouse_pkg/Drive.h"`, but I want to verify that is correct. I also don't know how to access the message parameters L and R right now, so I'm just doing `msg->L` or `msg->R`.

