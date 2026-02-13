1. terminator install (cannot use launch.sh without terminator)
a) $ sudo apt update
b) $ sudo apt install terminator

2. RViz2 install (cannot visualize without RViz2)
a) $ sudo apt install ros-humble-rviz2
b) $ sudo apt install ros-humble-rviz-2d-overlay-plugins
c) $ sudo apt install ros-humble-rviz-2d-overlay-msgs

3. Build
a) unzip the given .tar.gz file within ROS2 workspace src folder
b) $ cd src/free_floating_simulation
   (recommended to work in independent repository; current project is composed of multiple packages)
c) colcon build --symlink-install

4. Execution
   (!!! execute the terminal within free_floating_simulation folder !!!)
a) $ source install/setup.bash 
b) $ bash launch.sh
   will automatically launch all the packages, nodes

5. Changing Parameters (Recommended to use VS Code environment)
Info) all the parameters are connected via launch.xml files
      no need to re-build the packages after changing the parameters within launch files
Warning) make sure that dynamic & hardware parameters for fsw and simulation packages matches each other
a) Guidance / Control parameters
   open and modify : /src/free_floating_simulation/src/fsw/launch/fsw.launch.xml
b) Simulation / Visualization parameters
   open and modify : /src/free_floating_simulation/src/simulator/launch/simulator.launch.xml