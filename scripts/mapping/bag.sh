roscore &
sleep 5
rosparam set use_sim_time true
roslaunch fortnight bag.launch
