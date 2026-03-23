#!/bin/bash

# Launch evaluator node in a new terminator window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 launch evaluation evaluation_launch.py; echo \"Press Enter to close the terminal...\"; read'" &

# Launch simulator node in a new terminator window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 launch simulation simulator_launch.py; echo \"Press Enter to close the terminal...\"; read'" &

# Launch control node in another window
terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 launch fsw fsw_launch.py; echo \"Press Enter to close the terminal...\"; read'" &

# Monitor State in another window
# terminator -e "bash -c 'source ~/.bashrc; source install/setup.bash; ros2 topic echo /evaluation; echo \"Press Enter to close the terminal...\"; read'"