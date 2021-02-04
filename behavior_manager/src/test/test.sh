#!/bin/bash

NUMID_DRONE=111
DRONE_SWARM_ID=1

export AEROSTACK_PROJECT=${AEROSTACK_STACK}/projects/basic_mission_rotors_simulator
. ${AEROSTACK_STACK}/setup.sh

#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Manager                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior manager" --command "bash -c \"
roslaunch behavior_manager behavior_manager.launch \
  robot_namespace:=drone$NUMID_DRONE \
  testing:=true \
  catalog_path:=${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Test                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior manager test" --command "bash -c \"
rosrun behavior_manager test.py \
exec bash\""  &
