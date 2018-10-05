if [ "$#" -lt 1 ]; then
	ls ~/sick2018/maps/ | grep ".yaml$"
else
	export MAP_FILE=~/sick2018/maps/"$1"
	roslaunch fortnight localization.launch
	fg
fi
