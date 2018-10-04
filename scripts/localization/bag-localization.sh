if [ "$#" -lt 1 ]; then
	ls ~/sick2018/maps/ | grep ".yaml$"
else
	export MAP_FILE=~/sick2018/maps/"$1"
	roscore &
	sleep 5
	#rosparam set robot_description
	rosparam set use_sim_time true
	roslaunch fortnight amcl.launch
	fg
fi
