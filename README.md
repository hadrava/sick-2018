This is our project for the SICK robot day 2018.

How to build our software?

 - clone it somewhere e.g. ~/sick-2018
 - create new catkin workspace by (outside of your git repository)

```
mkdir catkin_ws
```

  - link `src` directory from git to catkin workspace:

```
cd catkin_ws
ln -s ~/sick-2018/src
```

 - now you are ready to make everything with:

```
catkin_make
```
 - it might be a good idea to add setup script from your main catkin workspace to your bashrc:
```
echo ". ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
. ~/catkin_ws/devel/setup.bash
```


How to run one executable?
```
rosrun [package] [executable]
rosrun planing hermite
```
