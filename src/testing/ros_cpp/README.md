To compile everything, go to `~/ros_catkin_ws` and run `catkin_make`. Make sure you run this as suer pi, or all the permissions will get borked. 

In order to run the `cpp_test` node, you must become the root user with `sudo su` and then run `rosrun pacmouse_pkg cpp_test`

The vl6180_pi C library must be installed by cloning the repo and installing it as they tell you. The CMakeLists file is set up to link it properly. 
