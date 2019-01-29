# Instructions for getting the models loaded in Gazebo

You need to add the gazebo models folder to your gazebo model path. You can do with by adding

```
export GAZEBO_MODEL_PATH=/path/to/models
```

Run gazebo with 

```
rosrun gazebo_ros gazebo
```

(roscore must be already running).

You can insert the models into gazebo from the "Insert" tab on the left side of the screen.

You can see the laser range returns by running 

```
rostopic echo /robot/laser_back
```
