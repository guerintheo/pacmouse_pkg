To compile everything, go to `~/ros_catkin_ws` and run `catkin_make`. Make sure you run this as suer pi, or all the permissions will get borked. 

In order to run the `cpp_test` node, you must become the root user with `sudo su` and then run `rosrun pacmouse_pkg cpp_test`
