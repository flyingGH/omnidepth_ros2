## Setup repository and ROS2 workspace
```
cd ~/
mkdir omnidepth_ws
cd omnidepth_ws
mkdir src
cd src
git clone git@github.com:synapsemobility/omnidepth_ros2.git
```

## Build package
```
cd ~/omnidepth_ws/
colcon build --symlink-install
source install/setup.bash
```
You may additionally add this in your bashrc, or you can manually run last command manually everytime you start a new terminal.


## RVIZ Launch
Launch RVIZ
```
rviz2 -d src/omnidepth_ros2/rviz_omnidepth.rviz
```
You may save this rviz file after adding any relevant topics that you desire.

## ISAAC Launch
1. Download USD file from the drive link. 
2. Open ISAAC simulator open USD file. (It may take upto 1min if you are launching this USD file first time.)
3. Play the animation using play button on the right.


## Launch OmniDepth pacakge
Now create a new terminal and launch one command at a time. \\
Each command below launches omniDepth on one monocular camera each at a time. \\
If entire 360 degree PCL is required, all the commands needs to be launched on a separate terminal after sourcing setup.bash as described in Build package section above.
```
ros2 launch omnidepth_ros2 omnidepth_front.launch.xml

ros2 launch omnidepth_ros2 omnidepth_left.launch.xml

ros2 launch omnidepth_ros2 omnidepth_right.launch.xml

ros2 launch omnidepth_ros2 omnidepth_back.launch.xml

```

## Appendix
### Rotation angle of the point cloud.
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