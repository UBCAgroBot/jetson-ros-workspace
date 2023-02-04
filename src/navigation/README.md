# Navigation Subteam

## Internal Nodes and Topics

| Node                     | Description                              | Subscribed Topics       | Command Line Arguments              | Published Topics           |
| ------------------------ | ---------------------------------------- | ----------------------- | ----------------------------------- | -------------------------- |
| mock_camera_publisher    | Publishes a mock camera image stream     | -                       | video<br>verbosity                  | navigation/mock_camera     |
| algorithm_publisher      | Runs algorithm on image stream           | /camera/color/image_raw | algo\*<br>mock<br>show<br>verbosity | navigation/\<alg\>         |
| post_processor_publisher | Runs the post processor on the algorithm | /navigation/\<alg\>     | port\*<br>verbosity                 | navigation/post_processing |

To run a node, run the following command:

```bash
ros2 run navigation mock_camera_publisher
```

## Nodes with Command Line Arguments

You can also run the nodes with the command line arguments:

```bash
ros2 run navigation algorithm_publisher --ros-args -p alg:=center_row
ros2 run navigation algorithm_publisher --ros-args -p alg:=center_row -p mock:=False -p show:=True
```

**Not passing any arguments will result in default argument values being used. Arguments marked with \* are required.**

### common arguments

These arguments apply to all Nodes

#### `verbosity`

- Type: `int`
- Int: The verbosity of the logger:
  - 0: `UNSET`
  - 10: `DEBUG`
  - 20: `INFO`
  - 30: `WARN`
  - 40: `ERROR`
  - 50: `FATAL`
- Default: 20 `(INFO)`

### mock_camera_publisher

#### `video`

- Type: `string`
- String: use the video file at `src/navigation/videos/<video>.mp4` as the mock camera stream
- Empty string / undefined: use the images in `src/navigation/images/*.png` for the mock camera stream
- Default: `''`

### algorithm_publisher

#### `alg`

- Type: `string`
- String: The name of the algorithm to run:
  - `hough`
  - `center_row`
  - `mini_contour`
  - `mini_contour_downward`
  - `scanning`
  - `check_row_end`
  - `seesaw`
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

## Navigation pipeline

We maintain a [ROS2 Launch file](/src/navigation/launch/navigation_launch.py) that allows us to run all Nodes in the navigation pipeline at the same time.

```bash
# run launch file
ros2 launch navigation navigation_launch.py
```

To configure arguments for individual nodes in the pipeline edit [params.yml](/src/navigation/config/params.yml) before running `colcon build`.
