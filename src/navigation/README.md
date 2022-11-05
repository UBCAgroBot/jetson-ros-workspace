# Navigation Subteam

## Internal Nodes and Topics

| Node                  | Description                          | Subscribed Topics       | Command Line Arguments      | Published Topics         |
| --------------------- | ------------------------------------ | ----------------------- | --------------------------- | ------------------------ |
| mock_camera_publisher | Publishes a mock camera image stream |                         | video                       | navigation/mock_camera   |
| algorithm_publisher   | Runs algorithm on image stream       | /camera/color/image_raw | algorithm\*<br>mock<br>show | navigation/\<algorithm\> |

To run a node, run the following command:

```bash
ros2 run navigation mock_camera_publisher
```

## Nodes with Command Line Arguments

You can also run the nodes with the command line arguments:

```bash
ros2 run navigation algorithm_publisher --ros-args -p algorithm:=center_row
ros2 run navigation algorithm_publisher --ros-args -p algorithm:=center_row -p mock:=False -p show:=True
```

Not passing any arguments will result in default argument values being used. Arguments marked with \* are required.

### mock_camera_publisher

#### `video`

- Type: `string`
- True: Use a video file as the mock camera image stream
- False: Use as the mock camera image stream
- Default: `''`

### algorithm_publisher

#### `algorithm`

- Type: `string`
- String: The name of the algorithm to run:
  - `hough`
  - `center_row`
  - `mini_contour`
  - `mini_contour_downward`
  - `scanning`
  - `check_row_end`
- Required

#### `mock`

- Type: `boolean`
- True: Use `navigation/mock_camera` for the image stream
- False: Use `/camera/color/image_raw` for the image stream
- Default: `False`

#### `show`

- Type: `boolean`
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
- https://index.ros.org/r/realsense2_camera
- https://github.com/intel/ros2_intel_realsense
