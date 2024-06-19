
# omniDepth ROS2
An AI Framework for class-aware surround BEV 3D occupancy map with multi-view monocular cameras. It enables low-cost, precise, camera-only perception for autonomous mobile robots with surround-view cameras.
Our service can be easily integrated into any AMR robotics framework (Currently supported: ROS2 Humble). 

* [Model request form](https://forms.gle/2JLW8mkCmrBkLmZw8): Model is only available on request basis. We have a US-based small team, we can get back in 24 hours.
* [Demo Video: omniDepth for AMRs](https://www.youtube.com/watch?v=T_HIsUSDyoQ&ab_channel=SynapseMobility)

## Visualizations

| Input Modality |            Input Visualization        |            Output Visualization           |
|:--------------------------------------:|:--------------------------------------:|:--------------------------------------:|
|Camera views| Multi-view monocular Images and Intrinsics/ Extrinsic Matrices | Class-aware 3D dense point cloud from surround view cameras |
| Four Surround-view cameras, covering 360 degree without overlap| ![scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene.gif)| ![OmniDepth](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/omniDepth.gif)|
| Front view monocular camera | ![Front scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_front.gif) | ![Front point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_front.gif)|
| Left view monocular camera | ![Left scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_left.gif) | ![Left point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_left.gif)|
| Back view monocular camera | ![Back scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_back.gif) | ![Back point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_back.gif)|
| Right view monocular camera | ![Right scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_right.gif) | ![Right point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_right.gif)|



# Instructions
Please follow below steps to reproduce above visualizations locally. First, you can clone this repository at your home location.
```
cd ~/
git clone https://github.com/synapsemobility/omnidepth_ros2.git
cd omnidepth_ros2/
```

## Download model
We are handing out the the model checkpoint on a per-request basis. If you want to deploy it on your robot please fill out this [form](https://forms.gle/2JLW8mkCmrBkLmZw8), and we can get back to you within 24hrs. \\
Place this model at the below location:
```
~/omnidepth_ros2
```

## Build Docker Image
First build a docker image. It builds a linux-based docker with ROS2 Humble with omniDepth dependencies. 
```
docker build -t omnidepth_humble_image .
```

Note: First time you run this, it may take some while as all the pip3 dependencies and ROS2 dependencies are being installed here!

## Run Docker Container
It runs docker container with interactive terminal where you can run rviz2 from the docker container.
```
./run_docker.bash
```
Note: For Debugging commands for docker, please refer to Troubleshoot section at the end of this page. 

## Pull latest changes
```
cd /root/omnidepth_ws/src/omnidepth_ros2/
git pull
```

## Launch visualization
Within docker terminal, go to workspace directory:
```
cd /root/omnidepth_ws
```
Then, launch rviz2, with cached attributes to visualize
```
rviz2 -d src/omnidepth_ros2/rviz_omnidepth.rviz
```

## Build omniDepth
Create new terminal and attach it to the container
```
docker exec -it omnidepth_humble_image_container bash
```
Then, go to the workspace directory: 
```
cd /root/omnidepth_ws
```
Finally build the workspace
```
colcon build --symlink-install
source install/setup.bash
```

## Launch omniDepth
From the same terminal above, launch omniDepth using front launchfile
```
ros2 launch omnidepth_ros2 omnidepth_front.launch.xml
```

### Optionally: For other cameras, run below commands for each camera

Create new terminal and attach it to the container
```
docker exec -it omnidepth_humble_image_container bash
```
Then, go to the workspace directory: 
```
/root/omnidepth_ws
```
Source setup file
```
source install/setup.bash
```
Each command below launches omniDepth on one monocular camera each at a time. \\
If entire 360 degree PCL is required, all the commands needs to be launched on a separate terminal each, following all the steps in this section from start. 
```
ros2 launch omnidepth_ros2 omnidepth_right.launch.xml
ros2 launch omnidepth_ros2 omnidepth_left.launch.xml
ros2 launch omnidepth_ros2 omnidepth_back.launch.xml
```

## Launch Scene
To understand the scene first, please refer these [four images](https://github.com/synapsemobility/omnidepth_ros2/tree/main/visualizations/rosbag_scene). They give view of the test warehouse from different angles to get yourself familiarize with it. 
There are two ways where you can launch the scene so that required topics can be subscribed:
### Option 1: Launch ROSBag
1. Download ROSBag file from the [link](https://drive.google.com/file/d/1pFJr5HNQ8YixB9AHIeVBL4CRUS0oy5zV/view?usp=drive_link)
2. Go to the directory location of the zip file.
3. Extract the file into a direcotry named `omnidepth_usd_v1_rosbag`, i.e. the Default name.
4. Finally, launch ROS bag file in loop with: 
```
ros2 bag play omnidepth_usd_v1_rosbag/ -l
```

### Option 2: Play Isaac sim USD file
1. Download USD file from the [drive link](https://drive.google.com/file/d/1CKsHDYRw4J_wQ6_jgNjwoaswgUvqomCb/view?usp=drive_link). Note, when opening this USD file first time, it can take upto 5mins for ISAAC sim to launch this USD file.  
2. Open ISAAC simulator open USD file. (It may take upto 1min if you are launching this USD file the first time.)
3. Play the animation using the play button on the left pannel.

Note: Latest Isaac sim version is recommended [ISAAC 4.0](https://developer.nvidia.com/blog/supercharge-robotics-workflows-with-ai-and-simulation-using-nvidia-isaac-sim-4-0-and-nvidia-isaac-lab/)


### All Done!
Now you can visualize point cloud on the rviz window. Feel free to move it around to see the depth information! \\
You can also visualize monocular depth images by hitting checkbox on the left. 

On the launch file termincal you should see something like below, which shows that point cloud meta info is logged: 
```
[omnidepth_inference_node-1] [INFO] [1718829469.171886841] [omnidepth_inference]: Model Done
[omnidepth_inference_node-1] [INFO] [1718829469.172464220] [omnidepth_inference]: Initializing static memory...
[omnidepth_inference_node-1] [INFO] [1718829469.175627241] [omnidepth_inference]: Initialization Done
[omnidepth_inference_node-1] [INFO] [1718829526.909568003] [omnidepth_inference]: Total runtime: 1.404341220855713
[omnidepth_inference_node-1] [INFO] [1718829529.333300118] [omnidepth_inference]: (249241, 6)
[omnidepth_inference_node-1] [INFO] [1718829530.747238818] [omnidepth_inference]: Total runtime: 1.4040896892547607
[omnidepth_inference_node-1] [INFO] [1718829533.224480874] [omnidepth_inference]: (240365, 6)
[omnidepth_inference_node-1] [INFO] [1718829534.738286196] [omnidepth_inference]: Total runtime: 1.506906270980835
...
```

# Appendix
## Rotation angle of the point cloud.
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

## Troubleshoot Docker commands
New terminal for docker
```
docker exec -it <container_name> bash
```

Copy from/ to docker
```
docker cp <local_path> <container_id>:<container_path>
```

To start container
```
docker start <container_name>
```

Remove images
```
docker images
docker rmi -f <Image ID>
```

List installed containters
```
docker ps -a
```

Remove all containers
```
docker rm -v -f $(docker ps -qa)
```

## Record rosbag from USD file 
For quick experiments, you may chose to store your own rosbag file for the given USD file. 

```
ros2 bag record -o omnidepth_usd_v1_rosbag \
    /front_stereo_camera/right/image_raw \
    /front_stereo_camera/right/camera_info \
    /left_stereo_camera/right/image_raw \
    /left_stereo_camera/right/camera_info \
    /right_stereo_camera/right/image_raw \
    /right_stereo_camera/right/camera_info \
    /back_stereo_camera/right/image_raw \
    /back_stereo_camera/right/camera_info
```

## Relevant ROS2 Topics

### Subscribed Topics
|            Topic Name           |            Topic Info           |
|:--------------------------------------:|:--------------------------------------:|
| /front_stereo_camera/right/camera_info | sensor_msgs/msg/CameraInfo |
| /front_stereo_camera/right/image_raw | sensor_msgs/msg/Image |
| /back_stereo_camera/right/camera_info | sensor_msgs/msg/CameraInfo |
| /back_stereo_camera/right/image_raw | sensor_msgs/msg/Image |
| /left_stereo_camera/right/camera_info | sensor_msgs/msg/CameraInfo |
| /left_stereo_camera/right/image_raw | sensor_msgs/msg/Image |
| /right_stereo_camera/right/camera_info | sensor_msgs/msg/CameraInfo |
| /right_stereo_camera/right/image_raw | sensor_msgs/msg/Image |


### Published Topics
|            Topic Name           |            Topic Info           |
|:--------------------------------------:|:--------------------------------------:|
| /point_cloud_front | sensor_msgs/msg/PointCloud2 |
| /depth_image_front | sensor_msgs/msg/Image |
| /point_cloud_back | sensor_msgs/msg/PointCloud2 |
| /depth_image_back | sensor_msgs/msg/Image |
| /point_cloud_left | sensor_msgs/msg/PointCloud2 |
| /depth_image_left | sensor_msgs/msg/Image |
| /point_cloud_right | sensor_msgs/msg/PointCloud2 |
| /depth_image_right | sensor_msgs/msg/Image |


# License
The OmmniViewDepth code is under a 2-clause BSD License for non-commercial usage.
