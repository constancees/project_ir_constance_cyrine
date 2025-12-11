# project_ir_constance_cyrine

For our project, we decided to try to implement a PD controller to run our robot. The way this works is that the robot has 3 different states : FORWARD, REALIGN and TURN.

During the FORWARD state, which is always the first applied, the robot moves forward and tries to correct its position in function of the error. We added a Kp coefficient which is an attribute of the class to easily adapt this value. When an object is detected in front of the robot, at a distance set by the turn_trigger attribute, the robot switches into the TURN state.

In the TURN state, the robot does not move linearly but looks at the values left and right and turns in the direction where the biggest free space is, until there is enough free space in front of it. It then switches to the REALIGN state

In the REALIGN state, the robot does effectively the same as in the FORWARD state, but moving slowly in order to finely correct the positioning errors before advancing. We also added a functionality to go from this state directly into the TURN state in case an obstacle was detected but the robot hadn't finished turning, in order to improve the robustness of our code.


To run our code, we need to type in a first terminal : 

    colcon build
    source source ~/.bashrc
    ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_1.world

We can replace the number to match the id of the maze we wish to run. This will open Gazebo in the wanted maze, we must then open another terminal or split the current one and run the line :

    ros2 run simulation_pkg simulation_node

Which will make the robot move and complete the maze.
