#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo 'Trulla die Waldfee'

dist=true

if [ $dist ]
then
	
	roslaunch my_package nodedisturbance.launch &

fi

roslaunch my_package nodePID.launch  
#roslaunch my_package nodeCascade1.launch
#roslaunch my_package nodePI.launch
#roslaunch my_package nodeCascade2.launch
#roslaunch my_package nodeCascade2T.launch
#roslaunch my_package nodePole_place.launch
#roslaunch my_package nodeLQR.launch
#roslaunch my_package nodeCascade3.launch
#roslaunch my_package nodePurePursuit.launch


