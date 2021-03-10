#!/bin/bash
file=$(readlink -f "$0")
filepath=$(dirname "$file")
roslaunch behavior_coordinator behavior_coordinator.launch robot_namespace:="drone111" testing:=true catalog_path:="behavior_catalog.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res != *ERROR* ]]
then
	echo "Test if the belief_manager_config.yaml is correct: succeeded"
else 
	echo "Test if the belief_manager_config.yaml is correct: failed"
fi
rm temp.txt
kill -2 %1