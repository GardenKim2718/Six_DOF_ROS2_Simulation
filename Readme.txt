1. terminator install (cannot use launch.sh without terminator)
a) $ sudo apt update
b) $ sudo apt install terminator

2. Eigen install (for handling linear algebra within Cpp)
a) $ sudo apt update
b) $ sudo apt install libeigen3-dev

3. RViz2 install (cannot visualize without RViz2)
a) $ sudo apt install ros-humble-rviz2
b) $ sudo apt install ros-humble-rviz-2d-overlay-plugins
c) $ sudo apt install ros-humble-rviz-2d-overlay-msgs

4. Installation
a) unzip the given .tar.gz file within ROS2 workspace src folder
   e.g. <your ros2 workspace>
         >src
          >SIX_DOF_SIMULATION

5. eiquadprog install (QP solver)
a) $ cd src/six_dof_simulation/src
b) $ git clone --recursive https://github.com/stack-of-tasks/eiquadprog.git
c) $ cd ..

6. Build
a) move into the project directory (if you are not already in it)
   $ cd src/six_dof_simulation
b) remove existing build, install, log directories
   $ rm -rf build install log
c) build the packages
   $ colcon build --symlink-install --packages-up-to
   if above CLI does not work, remove the built /build /install /log directories
   and then try the below commands in the following order
   $ colcon build --symlink-install --packages-select interfaces
   $ colcon build --symlink-install --packages-select eiquadprog
   $ colcon build --symlink-install --packages-select simulation
   $ colcon build --symlink-install --packages-select fsw
   $ colcon build --symlink-install --packages-select evaluation

7. Execution
   (!!! execute the terminal within SIX_DOF_SIMULATION folder !!!)
a) $ source install/setup.bash 
b) $ bash launch.sh
   will automatically launch all the packages, nodes

8. Changing Parameters
Info) all the parameters are connected via launch.xml files
      no need to re-build the packages after changing the parameters within launch files
Warning) make sure that dynamic & hardware parameters for fsw and simulation packages matches each other
a) Guidance & Control parameters
   open and modify : /src/SIX_DOF_SIMULATION/src/fsw/launch/fsw_launch.py
b) Simulation & Visualization parameters
   open and modify : /src/SIX_DOF_SIMULATION/src/simulator/launch/simulator_launch.py
c) Evaluation parameters
   open and modify : /src/SIX/DOF_SIMULATION/src/evaluation/launch/evaluation_launch.py
