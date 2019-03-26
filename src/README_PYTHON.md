# Instructions for setting up the python environment on the Rapsberry Pi

Before running any code from this package, you will need to install it. You 
install it by running

```
sudo ln -s /home/<user>/<ros_ws_path>/src/pacmouse_pkg /usr/lib/python2.7/pacmouse_pkg
```

On mac, the command that worked was:
 ```
sudo ln -s /<path_to_package>/pacmouse_pkg /usr/local/lib/python2.7/site-packages/pacmouse_pkg
```
Note that you cannot link to `/usr/lib/python2.7/*` because it is protected on mac

More generally, you just need to make sure that your directory of `pacmouse_pkg` is visible
to Python. You can check which where Python looks for packages by opening a Python interpreter
and running
```
import sys
sys.path
```
Put a link to `pacmouse_pkg` somewhere in one of the directories on that path.
Ideally all of this would be done by an installation script, but for now I think this works.


Note that you *must* give the absolute paths for the link target and name,
otherwise it will not work. Now you can import modules from this package, such
as 

```
import pacmouse_pkg.src.ros.estimation_node as estimation_node
```

Note that there may be name collisions with the package that ROS generates for messages. For example, the $PYTHONPATH environment variable may include `/home/<user>/<ros_ws_path>/devel/lib/python2.7/dist-packages`, which contains a `pacmouse_pkg` directory. If this is the case, then one option is to symlink the individual folders from the `pacmouse_pkg` package that you wish to be able to import, such as `pacmouse_pkg/src`:

```
sudo ln -s /home/<user>/<ros_ws_path>/src/pacmouse_pkg/src /home/<user>/<ros_ws_path>/devel/lib/python2.7/dist-packages/pacmouse_pkg/src
```

To install scipy on the raspberry pi run (you can use pip on most other computers)
```
sudo apt-get install python2.7-scipy
```

Install other requirements
```
pip install numpy multiprocess
```

You may want to add the following line to the Pi's `~/.bashrc` file, as it will
allow you to call `rosrun pacmouse_pkg <executable>` to run nodes, for example.

```
source /home/<user>/<ros_ws_path>/devel/setup.bash
```

# Getting the feeback control stack working

```
./estimation_node.py
```

```
./planner_node.py
```

```
./pose_control_node.py
```

```
roslaunch pacmouse_pkg mousedrive.launch world_name:=pacmouse_pkg/gazebo_worlds/TestMazeAutogen.world
```
