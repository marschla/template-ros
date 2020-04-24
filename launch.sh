#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#node1:PID; node2: cascade PID; node3: PI

#roslaunch my_package node1.launch
roslaunch my_package node2.launch
#roslaunch my_package node3.launch
