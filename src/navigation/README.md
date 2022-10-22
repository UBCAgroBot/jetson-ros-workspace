# Navigation Subteam

## Internal Nodes and Topics

| Node                  | Description                              | Subscribed Topics       | Command Line Arguments | Published Topics       |
| --------------------- | ---------------------------------------- | ----------------------- | ---------------------- | ---------------------- |
| mock_camera_publisher | Publishes a mock camera image stream     |                         |                        | navigation/mock_camera |
| center_row_publisher  | Runs center_row_algorithm on image steam | /camera/color/image_raw | mock<br>show           | navigation/center_row  |

To run the nodes, run the following commands:

```bash
ros2 run navigation mock_camera_publisher
ros2 run navigation center_row_publisher
```

## Nodes with Command Line Arguments

You can also run the nodes with the command line arguments:

```bash
ros2 run navigation center_row_publisher --ros-args -p mock:=False
ros2 run navigation center_row_publisher --ros-args -p mock:=True -p show:=True
```

Not passing any arguments will result in default argument values being used.

### center_row_publisher

#### `mock`

- Type: `bool`
- True: Use `navigation/mock_camera` for the image stream
- False: Use `/camera/color/image_raw` for the image stream
- Default: `True`

#### `show`

- Type: `bool`
- True: Display the image stream and the processed frames.
- False: Do not display the image stream and the processed frames.
- Default: `True`

## External Nodes and Topics

### Intel Realsense D455

To start the camera run:

```bash
ros2 launch realsense2_camera rs_launch.py
```

The image stream is published to multiple topics. We will be using `/camera/color/image_raw`.

List of important links that include information on the topics and on the arguments for the launch file:

- https://dev.intelrealsense.com/docs/ros-wrapper
- https://index.ros.org/r/realsense2_camera/
- https://github.com/intel/ros2_intel_realsense
