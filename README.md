# CPSL_TI_Radar_ROS
Set of ROS packages which can be used to integrate with the CPSL_TI_Radar Python Module available at the [CPSL_TI_Radar Repository](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer)


## Installation:

### 1. Install CPSL_TI_Radar Python Module
Prior to running any of this code in ROS, setup and install the CPSL_TI_Radar python module by following the instructions in the [readme](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer/)

### 2. Install ROS
1. Follow the instructions on the [ROS installation instructions](http://wiki.ros.org/noetic/Installation) website to install ROS. If you are unfamiliar with ROS, its worth taking some of the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)

## Adding radar and radar_msgs packages to catkin workspace
We provide two ROS packages to integrate with the CPSL_TI_Radar module:
* radar: a set of ROS nodes that integrate connect to the radar module and convert it into ROS formats
* radar_msgs: a set of custom messages used when streaming data from the DCA1000 board. 

1. To add the two packages to the catkin workspace, perform the following commands
```
cp -r radar ~/[catkin_ws]/src
cp -r radar_msgs ~/[catkin_ws]/src
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

### 2. Start Vicom Rosnodes
1. Text to be added

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
roslaunch radar start_radar_nodes.launch
```

#### Option 2: Streaming from DCA1000:
1. Start the ROS node for obtaining the raw packets from the DCA1000. In a new terminal window, run the following command:
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
rosrun radar DCA_RawPacketData.py
```
2. Next, activate the ROS node for obtaining the normalized range-azimuth response
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
rosrun radar DCA_RngAzResp.py
```

### 5. Visualize the radar operation using RVIZ
Depending on the streaming method being used, the following methods can be used to visualize the data from the Radar in ROS

#### Option 1: Streaming from IWR (i.e: no DCA1000)
1. To visualize the environment and the radar data, use rviz which can be launched using 
```
conda deactivate
rosrun rviz rviz
```
If you get an error, make sure that conda is deactivated as sometimes it will cause errors
2. To visualize the scene, we have implemented a simple rviz setup which can be loaded using the "rviz_config.rviz" file. The vicom system visualizations and reference frames must be set for this to work correctly 

#### Option 2: Streaming from DCA1000:
If streaming from the DCA1000, the following rosnodes can be used to obtain/read the data:

* Visualizing normalized range azimuth response: To view the range-azimuth response in real time, run the following commands:
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
rosrun radar DCA_RngAzResp_img_sub.py
```

* Example node for obtaining data for normalized range azimuth response: To obtain the data corresponding to the range-azimuth response, use the following commands as a starter.
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
rosrun radar DCA_RngAzResp_array_sub.py
```

* Example node for obtaining raw packets: To obtain the data corresponding to the range-azimuth response, use the following commands as a starter.
```
conda deactivate
cd ~/[catkin_ws]
source devel/setup.bash
rosrun radar DCA_RawPacketData_sub.py
```