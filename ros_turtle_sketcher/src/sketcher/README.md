# turtle_sketcher
A [ROS](http://wiki.ros.org/) node that controls [turtlesim](http://wiki.ros.org/turtlesim) to draw figures taken from text files.
Tested in [ROS Jade](http://wiki.ros.org/jade) & Ubuntu 14.04.

## Installation
This package assumes you have ROS Jade installed in your system. Please follow the [official instructions](http://wiki.ros.org/jade/Installation) if not.
It is also assumed that a catkin workspace is already set up. Catkin installation instructions can be found [here](http://wiki.ros.org/catkin), and instructions to create a Catkin workspace, [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). The whole repository should be placed in ../catkin_ws/src/turtle_sketcher.

## Compilation
Once the repository is properly placed in Catkin's workspace, go to Catkin's root directory, and run 
`catkin_make`

## Running the node
First of all, roscore and turtlesim shall be started. If official install instructions were followed, you should have roscore in your path, so you can ran
`roscore`
from a terminal.
Then, start turtlesim as `rosrun turtlesim turtlesim_node` from a separate terminal. 
Finally, run `rosrun turtle_sketcher turtle_sketcher_node [figure file]`
If rosrun can't find sketcher node, run `source devel/setup.bash` from Catkin workspace's root directory (only needed once).

## browser_sketcher
Optionally, if you want to see the sketcher's progress in a web browser, you can use sketcher_web. You will need rosbridge_server application for this purpose.
If rosbridge_server is not available in your system, you can get it with `sudo apt-get install ros-jade-rosbridge-server`.

After running roscore and turtlesim, but before running sketcher node, run `roslaunch rosbridge_server rosbridge_websocket.launch` in a separate terminal. With roscore, turtlesim, and rosbridge_server running, you can now lunch the turtle sketcher node, and the progress should appear in your web browser in real time.

Note: By default, the turtle's name is "turtle1". You can change which robot to listen to by changing the textbox and pressing the button in the web.

## star_generator
Create your own figures! Open the html file, change the parameters and play! The console will output the points you need to build a file readable by sketcher node without any scaling; just copy and paste to create a brand new figure.
