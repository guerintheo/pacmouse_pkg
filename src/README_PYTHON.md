# Instructions for setting up the python environment on the Rapsberry Pi

To install scipy on the raspberry pi run (you can use pip on most other computers)
```
sudo apt-get install python2.7-scipy
```

Install other requirements
```
pip install numpy
```


# Getting the feeback control stack working

```
./estimation_node.py
```

```
./odom_estimation_converter.py
```

```
./planner_node.py
```

```
./pose_control.py
```

```
roslaunch pacmouse_pkg mousedrive.launch world_name:=pacmouse_pkg/gazebo_worlds/TestMazeAutogen.world
```
