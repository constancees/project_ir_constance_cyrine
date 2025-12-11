# project_ir_constance_cyrine

To run our code, we need to type in a first terminal : 


    colcon build
    source source ~/.bashrc
    ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_1.world

We can replace the number to match the id of the maze we wish to run. This will open Gazebo in the wanted maze, we must then open another terminal or split the current one and run the line :


ros2 run simulation_pkg simulation_node

