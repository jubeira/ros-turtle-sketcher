# ros-turtle-sketcher

A [ROS](http://wiki.ros.org/) node that controls [turtlesim](http://wiki.ros.org/turtlesim) to draw figures taken from text files.
Tested in ROS Jade (Ubuntu 14.04) & Kinetic (Ubuntu 16.04); should work with Melodic (Ubuntu 18.04).

[![Build Status](https://travis-ci.com/jubeira/ros-turtle-sketcher.svg?branch=master)](https://travis-ci.com/jubeira/ros-turtle-sketcher)

## Building from source
This package assumes you have ROS installed in your system; we'll assume ROS Kinetic, but any distribution that has Turtlesim should work. Please follow the [official instructions](http://wiki.ros.org/kinetic/Installation) if not.

Then, create a catkin workspace and clone the repository inside:

```
mkdir -p ros_turtle_sketcher_ws/src
git clone https://github.com/jubeira/ros-turtle-sketcher ros_turtle_sketcher_ws/src
cd ros_turtle_sketcher_ws
catkin_make
```

Don't forget to run `source devel/setup.bash` to add the package to your workspace before proceeding!

## Running the node

For a quick demo, run

```
roslaunch turtle_sketcher sketcher.launch
```

You can set the figure by setting `figure_file` parameter (use absolute paths).

## Browser sketcher
Optionally, if you want to see the sketcher's progress in a web browser, you can use sketcher_web. You will need rosbridge_server application for this purpose.
If rosbridge_server is not available in your system, you can get it with `apt-get install ros-kinetic-rosbridge-server`.

Before running sketcher node, run `roslaunch rosbridge_server rosbridge_websocket.launch` in a separate terminal. You can now lunch the turtle sketcher node. Open `sketcher_web` and the progress should appear in your web browser in real time.

Note: By default, the turtle's name is "turtle1". You can change which robot to listen to by changing the textbox and pressing the button in the web.

## star_generator
Create your own figures! Open the html file, change the parameters and play! The console will output the points you need to build a file readable by sketcher node without any scaling; just copy and paste to create a brand new figure.
