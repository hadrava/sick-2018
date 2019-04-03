## Warning: this code was written in hurry - during last two weeks before competition ##

This is our project for the [SICK robot day 2018](https://www.sick.com/de/en/robotday-2018/w/robotday/)

Task: collect as many as possible red balls from moving transporter and bring them to box near starting position.

- Robot was not successful during the competition.
- Video of robot in action on [youtube](https://www.youtube.com/watch?v=FbryiO-kIiE) (filmed after the competition)


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
