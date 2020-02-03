# Installation

In order for gazebo to know where the models for the maze are, you need to make them available to
gazebo in a known installation location. (The alternative to this is forcing you to edit your
`GAZEBO_RESOURCE_PATH` environment variable, but I went with the other method. The model for the
walls of the maze is in `pacmouse_pkg/gazebo_models`. You need to make a symlink from
gazebo's model database to the actual model location, e.g.
```
ln -s /home/aaron/ros_ws/src/pacmouse_pkg/gazebo_models/maze_wall
/home/aaron/.gazebo/models/maze_wall
```
Note that using full paths when making the link is important. (You will know if you screwed up if
when you try to launch everything, it crashes with an error about too many symlinks). I will make a
script for this setup step eventually. 

You will also need to modify one of your environment variables. Add this to your .bashrc or .zshrc
```
export GAZEBO_RESOURCE_PATH="/home/<user>/<ros_ws>/src"
```

# Running

Run to see the model in `rviz` and `gazebo`. Requires all of the ros/gazebo related packages.
Also requires the ros `controller_manager` package. Probably also requires one of the URDF
packages, but that might included with the ros/gazebo stuff

```
roslaunch pacmouse_pkg mousedrive.launch world_name:=pacmouse_pkg/gazebo_worlds/TestMazeAutogen.world
```

Rviz and gazebo don't seem to shut down very happily. I usually end up having to force quit them
from the command line. Even trying a `rosnode kill --all` seems to sometimes not kill everything. If
you try relaunching the roslaunch command and one of the programs fails on startup, you need to look
at current running tasks with `ps aux` and `kill` any remaining tasks related to gazebo, and maybe
roscore. The `pkill -f gazebo` command is very useful for this, but be careful because any other
processes that have gazebo in the name (e.g. `vim gazebo_worlds/myworld.world`) will probably also
get killed. 

Currently, the robot gets its commands from a topic that ends in `cmd_vel` (rostopic echo to see the
full name). There's a builtin ros controller for differential driving that translates the direction
ros `Twist` message into wheel movements. For current testing purposes with the rest of the software
stack, we can test everything if the planner puts out a sequence of velocity commands (i.e. we
abstract out / don't test right now the inner feedback loop that controls wheel torque to give
desired speed). Currently the `odom` topic gives odemetry as measured by wheel encoders (which you
will notice in rviz does not line up with real movement because of slip).

# Maze Design
In the `gazebo_worlds` folder, you will find `gazebo_mazegen.py`. This python script generates an
sdf file that can be loaded into gazebo. The script still needs a bunch of work on how to interface
with it, but for now you can mess around with new mazes by editing the `test_adj` adjacency matrix
(note some stuff is currently hard-coded for 3x3, but very easy to modify 
