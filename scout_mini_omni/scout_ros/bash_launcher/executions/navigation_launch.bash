#!/bin/bash
tab="--tab-with-profile=bash_launcher --command"
window="--window-with-profile=bash_launcher --comand"

sudo sudo ip link set can0 up type can bitrate 500000
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1

cd  $HOME/demulab/scout_ws/src/scout_ros/bash_launcher/bashes

gnome-terminal \
	$tab 'roscore'\
	$tab 'bash -c "sleep 1.0; ./scout_mini_omni_base.bash"'\
	$tab 'bash -c "sleep 1,0; ./sensor.bash"'\
	$tab 'bash -c "sleep 3.0; ./display_mode.bash"'\
	$tab 'bash -c "sleep 2.0; ./scout_v2_navigation.bash"'\

