if [ "$#" -lt 1 ]; then
	ls ~/sick2018/bags/ | grep ".bag$"
else
	rosbag play --clock ~/sick2018/bags/"$1" --topics "/tf" --topics "/scan" --topics "/tf_static"
fi
