#!/bin/bash

# Launch simulator node in a new terminator window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 launch simulation simulator.launch.xml; echo \"Press Enter to close the terminal...\"; read'" &
sleep 0.1

# Launch control node in another window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 launch fsw fsw.launch.xml; echo \"Press Enter to close the terminal...\"; read'" &
sleep 0.5

# Monitor State in another window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 topic echo /state; echo \"Press Enter to close the terminal...\"; read'" &
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 topic echo /guidance; echo \"Press Enter to close the terminal...\"; read'" &
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 topic echo /command; echo \"Press Enter to close the terminal...\"; read'" &