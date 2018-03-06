# ros-turtlebot-to-ball
Drives a turtlebot into a ball in gazebo.

## Instructions to Run
In order to download and build the package, run the following commands in the terminal.
~~~~
git clone https://github.com/khanh111/ros-turtlebot-to-ball.git
cd ros-turtlebot-to-ball
catkin_make
catkin_make install
~~~~

Place the file `mini_world.world` in the root of the workspace (i.e. in `ros-turtlebot-to-ball`).

Next, run the following commands (potentially in multiple terminals) to start ros, gazebo, and the our node.
~~~~
roscore
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<PATH_TO_WORKSPACE>/ros-turtlebot-to-ball/mini_world.world
roslaunch seeker interview.launch
~~~~

Use the Gazebo interface to alter the locations of the ball and turtlebot as desired, ensuring that the ball remains within the range of the laser scanner. Finally, we can enable the seeker.
~~~~
rosservice call /enable "true"
~~~~

We can also echo the relative displacement of the ball from the robot.
~~~~
rostopic echo /displacement
~~~~

The seeker can be disabled with the following command.
~~~~
rosservice call /enable "false"
~~~~
