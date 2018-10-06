if [ "$#" -lt 1 ]; then
	ls ~/sick2018/bags/ | grep ".bag$"
else
#	# this is needed for old recordings where laser_mount_link was wrong and stored inside /tf instead of /tf_static
#	rosrun tf tf_remap _mappings:='[
#                {old: laser_mount_link, new: /laser_mount_link_old},
#                {old: map, new: /map_old},
#                {old: base_link, new: /base_link_old}
#                ]' &
#                #{old: base_footprint, new: /base_footprint_old},
	rosbag play --clock ~/sick2018/bags/"$1" --topics "/tf" --topics "/scan" #/tf:=/tf_old
	fg
fi
