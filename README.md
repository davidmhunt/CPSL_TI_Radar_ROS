# CPSL_TI_Radar_ROS
Set of ROS packages which can be used to integrate with the CPSL_TI_Radar Python Module available at the [CPSL_TI_Radar Repository](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer)


## Installation:

### 1. Install CPSL_TI_Radar Python Module
Prior to running any of this code in ROS, setup and install the CPSL_TI_Radar python module by following the instructions in the [readme](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer/)

### 2. Install ROS
1. Follow the instructions on the [ROS installation instructions](http://wiki.ros.org/noetic/Installation) website to install ROS. If you are unfamiliar with ROS, its worth taking some of the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)

## Adding CPSL_TI_Radar_ROS packages to catkin workspace
We provide several ROS packages to integrate with the CPSL_TI_Radar module:
* radar_connect: a ROS package that handles connecting to the CPSL_TI_Radar and receiving/publishing the raw ADC datacube from the radar
* radar_datasets: a ROS package designed to capture time synchronized radar and lidar data from multiple radar units.
* radar_mmseg: a ROS package designed to integrate with radnav models (which use mmsegmentation) to generate 2d point clouds from the radar's range-azimuth response
* radar_msgs: a ROS package defining custom messages used for topics on the adc data cube, range azimuth response, range doppler response, and the processed radar point cloud
* radar_sig_processing: a set of ROS packages used for receiving the range doppler and range azimuth responses from the CPSL_TI_Radar module
* radar_views: a ROS pacakge used for viewing the range-azimuth and range-doppler responses in windows. Additionally includes functionality for viewing the rad-nav model's output in rviz.
* (archived) radar: a previous set of ROS nodes for integrating with the CPSL_TI_Radar module for the ICRA paper
* (archived) dataset_generator: a previous ros package used for generating datasets of time synchronized radar and lidar data

1. To add the two packages to the catkin workspace, perform the following commands
```
cd [catkin_ws]/src/
git clone https://github.com/davidmhunt/CPSL_TI_Radar_ROS.git
```
Here, [catkin_ws] is the path to your catkin workspace

2. Next, build the ROS nodes in your catkin workspace using the following commands:
```
cd ~/[catkin_ws]
catkin_make
```

3. Finally, source the setup.bash file so that ROS can find the nodes and the messages
```
source devel/setup.bash
```


## Setting up Code [IWR Streaming only]
If streaming from the IWR directly (i.e: no DCA1000), the following steps must be taken to correctly specify the reference frame of the streamed point cloud

The radar is mounted on a drone based platform. Additionally, we use a VICOM system to provide ground truth information for the rotation/position of the drone and objects in the scene. To setup the ROS node correctly, complete the following instructions

1. In the "TI_Radar_Demo_Visualizer/ROS Packages/radar/scripts/radar_static_transform.py", change the frame_id to be the frame_ID of the reference frame of the drone (or other platform)
```
static_transformStamped.header.frame_id = "[Drone Frame ID]"
```
For example, you could set [Drone Frame ID] to be "vicon/Drone1/Drone1"

## Running ROS Nodes

Running the code is performed in several steps. Here, it is recommended  to use a tool that allows you to display multiple tmux windows simultaneously. The following sets of instructions should be performed in order and in separate terminals:
1. Start roscore
2. Start Vicom rosnodes
3. Start CPSL_TI_Radar python module
4. Start Radar rosnodes
5. Perform visualization

### 1. Start roscore

1. In a new terminal window, perform the following command
```
conda deactivate
source devel/setup.bash
roscore
```

### 2. Start Vicom Rosnodes [IWR only]
1. If you are using the IWR and want to view the generated point cloud, start the Vicom system so that the generated pointcloud has a valid reference frame

### 3. Start Radar python module
Follow the instructions from the [CPSL_TI_Radar Repository](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer) readme file to start the CPSL_TI_Radar python module. Once, the .cfg and settings.json files have been set correctly, the following command will start the CPSL_TI_Radar module
```
poetry run python run_radar.py
```
The radar module should now be running, and should be waiting on the line similar to: "Radar.start_Radar:waiting for TLV listeners to connect to ROS clients"

### 4. Start Radar rosnodes
Depending on which streaming method is being used, the ROS nodes can be started in one of two ways:
#### Option 1: Streaming from IWR only (No DCA1000):
1. In a new terminal, run the following commands
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
roslaunch radar_launch IWR6843_run_dataset.launch
```

#### Option 2: Streaming from DCA1000:
1. Start the ROS node for obtaining the raw packets from the DCA1000. In a new terminal window, run the following command:
```
conda deactivate
cd ~/[catkin_ws]
roslaunch radar_launch DCA_run_radnav.launch
```