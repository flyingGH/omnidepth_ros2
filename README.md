
# omniDepth ROS2
An AI Framework for class-aware surround BEV 3D occupancy map with multi-view monocular cameras. It enables low-cost, precise, camera-only perception for autonomous mobile robots with surround-view cameras.
Our service can be easily integrated into any AMR robotics framework (Currently supported: ROS2 Humble). 

* [Model request form](https://forms.gle/2JLW8mkCmrBkLmZw8): Model is only available on request basis. We have a US-based small team, we can get back in 24 hours.
* [Demo Video: omniDepth for AMRs](https://www.youtube.com/watch?v=T_HIsUSDyoQ&ab_channel=SynapseMobility)

## Visualizations
For reproducing locally, the USD file created on [ISAAC 4.0](https://developer.nvidia.com/blog/supercharge-robotics-workflows-with-ai-and-simulation-using-nvidia-isaac-sim-4-0-and-nvidia-isaac-lab/) can be downloaded from [drive link](https://drive.google.com/file/d/1CKsHDYRw4J_wQ6_jgNjwoaswgUvqomCb/view?usp=drive_link).

| Input Modality |            Input Visualization        |            Output Visualization           |
|:--------------------------------------:|:--------------------------------------:|:--------------------------------------:|
|Camera views| Multi-view monocular Images and Intrinsics/ Extrinsic Matrices | Class-aware 3D dense point cloud from surround view cameras |
| Four Surround-view cameras, covering 360 degree without overlap| ![scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene.gif)| ![OmniDepth](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/omniDepth.gif)|
| Front view monocular camera | ![Front scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_front.gif) | ![Front point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_front.gif)|
| Left view monocular camera | ![Left scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_left.gif) | ![Left point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_left.gif)|
| Back view monocular camera | ![Back scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_back.gif) | ![Back point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_back.gif)|
| Right view monocular camera | ![Right scene](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/scene_right.gif) | ![Right point cloud](https://github.com/synapsemobility/omnidepth_ros2/blob/main/visualizations/pcl_right.gif)|

## Docker Installation
We are handing out the docker file and the model on a per-request basis. If you want to deploy it on your robot please fill out this [form](https://forms.gle/2JLW8mkCmrBkLmZw8), and we can get back to you within 24hrs.

### Steps to install Docker
TODO
### Steps to run Docker
TODO
## Relevant ROS2 Topics

### Subscribed Topics
|            Topic Name           |            Topic Info           |     Description  | Visualization | 
|:--------------------------------------:|:--------------------------------------:| :--------------------------------------:| :--------------------------------------:|
| TODO | TODO |TODO | TODO | 

### Published Topics
|            Topic Name           |            Topic Info           |     Description  | Visualization | 
|:--------------------------------------:|:--------------------------------------:| :--------------------------------------:| :--------------------------------------:|
| TODO | TODO |TODO | TODO | 

## License
The OmmniViewDepth code is under a 2-clause BSD License for non-commercial usage.
