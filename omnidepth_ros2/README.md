```
colcon build --symlink-install
source install/setup.bash
```

```
rviz2 -d src/omnidepth_ros2/rviz_omnidepth.rviz
```

```
ros2 launch omnidepth_ros2 omnidepth_front.launch.xml

ros2 launch omnidepth_ros2 omnidepth_left.launch.xml

ros2 launch omnidepth_ros2 omnidepth_right.launch.xml

ros2 launch omnidepth_ros2 omnidepth_back.launch.xml

```

```
Rotation of point-cloud
Front: 
- 3.14159
Back: 
- NA
Left: 
- -1.5708
Right: 
- 1.5708
```