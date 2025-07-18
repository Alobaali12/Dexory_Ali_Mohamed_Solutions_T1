
# LiDAR Height Filter ROS2 Project

## Author: Ali Mohamed  
## Date: 17/07/2025  

---

## Description
This ROS2 project implements a LiDAR point cloud height filter node.  
The node subscribes to raw LiDAR PointCloud2 data, filters points within a specified height range (min and max z values),  
and publishes the filtered point cloud data.  
The project also supports playback of recorded ROS2 bag files containing LiDAR data, filtering them in real-time.

---

## Features
- Filter LiDAR point clouds based on height (z-axis) range parameters.
- Play and filter recorded ROS2 bag files (.mcap).
- Publish filtered point clouds for visualization or further processing.
- Simple ROS2 node written in C++ using pcl and rclcpp.

---

## Prerequisites

- Docker (tested with Docker Desktop)
- ROS2 Iron environment inside Docker
- ROS2 bag files containing PointCloud2 messages (.mcap format)

---

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/Alobaali12/Dexory_Ali_Mohamed_Solutions
cd ros2_ws
```

### 2. Build the Docker Image

```bash
docker build -t lidar_filter:iron .
```

### 3. Build the ROS2 Workspace inside Docker

```bash
docker run -it --rm -v ${PWD}:/ros2_ws -w /ros2_ws lidar_filter:iron bash -c "colcon build"
```

---

## Running the System

### 1. Source the Workspace

Inside the Docker container or your shell where ROS2 is installed, run:

```bash
source /ros2_ws/install/setup.bash
```

### 2. Play the Bag File with Topic Remapping

Play your recorded LiDAR data bag file and remap the topic to `/points_raw` which the filter node subscribes to:

```bash
ros2 bag play lidar_data_0.mcap --remap /lidars/bpearl_front_right:=/points_raw
```

### 3. Run the Height Filter Node

Run the node with parameters for minimum and maximum height filtering range (example: 0.0 to 1.5 meters):

```bash
ros2 run lidar_height_filter height_filter_node --ros-args -p min_height:=0.0 -p max_height:=1.5
```

The node will print logs confirming the filtering range.

---

## Visualizing the Results

To visualize raw and filtered point clouds:

1. Launch `rviz2` (either natively or inside Docker with GUI forwarding).
2. Add two PointCloud2 displays subscribing to:
   - Raw point cloud: `/points_raw`
   - Filtered point cloud: `/points_filtered`

---

## Output Filtered Data to a Bag File (Optional)

To record the filtered output to a new bag file:

```bash
ros2 bag record /points_filtered -o filtered_output
```

This will save filtered point clouds in the `filtered_output` directory as a `.mcap` file.

---

## Example Screenshot

<img width="758" height="521" alt="sss" src="https://github.com/user-attachments/assets/7862409a-197f-4293-99ab-10d8140e1dde" />


---

## Notes

- Ensure your Docker setup supports GUI forwarding if you want to run `rviz2` inside the container.
- Adjust parameters as needed for your LiDAR data characteristics.
- The filter node is configurable via parameters, making it flexible for different environments.

---

## Contact

For questions or issues, please open an issue in this repository or contact me via alimodather2@gmail.com.

---

**End of README**
