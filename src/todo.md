# TODO for the Micromouse competition

## High priority

 * ~~develop concrete software architecture (Block diagram. where are the nodes. who publishes what)~~
 * ~~order a new BNO055~~
 * ~~create infrastructure for restarting all the scripts. decide how to use the buttons (state machine?)~~
 * ~~test encoders in C++ (using example script from pigpio)~~
 * ~~merge all pigpio interfacing into a single c++ ROS node (buttons, ~~encoders, motors, leds~~)~~
 * ~~parallelize the sensor model for the particle filter~~
 * ~~if the encoders work, write a motor speed controller (in the C++ node)~~
 * ~~tune (figure out what's wrong with) the motion model in the motion capture rig~~
 * add ROS publishing for the lidars in a C++ script
 * fix edge cases in macaroni planner
 
 * ~~Investigate IMU intermittencies and try rebooting the serial line if it goes down. ~~
 * Use Planner and controller in mocap
 * Fully test mode_controller
 * clean up the directories