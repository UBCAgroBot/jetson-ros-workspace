# Navigation Subteam

## Internal Nodes and Topics

| Node                  | Description                              | Subscribed Topics       | Published Topics       |
| --------------------- | ---------------------------------------- | ----------------------- | ---------------------- |
| mock_camera_publisher | Publishes a mock camera image stream     |                         | navigation/mock_camera |
| center_row_publisher  | Runs center_row_algorithm on image steam | /camera/color/image_raw | navigation/center_row  |

To run the nodes, run the following commands:

```bash
ros2 run navigation mock_camera_publisher
ros2 run navigation center_row_publisher
```

## External Nodes and Topics

### Intel Realsense D455

To start the camera run:

```bash
roslaunch realsense2_camera rs_camera.launch device_type:=d455
```

The image stream is published to multiple topics. We will be using `/camera/color/image_raw`.

List of important links that include information on the topics and on the arguments for the launch file:

- https://dev.intelrealsense.com/docs/ros-wrapper
- https://index.ros.org/r/realsense2_camera/
- https://github.com/intel/ros2_intel_realsense
