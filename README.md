# jetson-ros-workspace
This repo is for the code that will be deployed to the jetson board to run on the robot. 

## Getting Started

### Requirements
Use the links for installation guides. Versions are important.
* **Only For Developers** - Ubuntu 18.04 (You can use [your pc](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview), a VM, or a cloud provider such as [AWS](https://github.com/UBCAgroBot/jetson-ros-workspace/blob/main/docs/aws_developer_env.md))
* **Only For Jetson** - [Jetpack 4.6.1](https://developer.nvidia.com/embedded/jetpack) (Ubuntu is included in Jetpack)
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) Desktop Version

### Installation
* [Create a personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) on GitHub
* Clone the repo using the command belove
``` 
git clone https://<your-access-token>@github.com/UBCAgroBot/jetson-ros-workspace.git
```
* Open a terminal and navigate into the jetson-ros-workspace folder
* Use the command below to build
```
catkin_make
```

### Usage
* Open a terminal window
* Navigate to jetson-ros-workspace folder
* Use the command below to start running ROS commands
```
. devel/setup.bash
```
* Use the below command to start running roscore. You need roscore to run for any other ROS scripts to run.
```
  roscore
```
* **For scripts you are going to run:**
* Open a new terminal window
* Navigate to jetson-ros-workspace folder
* Use the ` . devel/setup.bash ` command to start running ROS commands
* Use the command below to run the scipt
```
  rosrun package_name script_name.py
```
For example, `rosrun navigation decision.py` currently publishes the integer `1` to topic `/state`

## Contribution
Please contribute to the project according to both GitHub and ROS contribution guidelines outlined below. ROS Contribution Guidelines are very important for your code to work on every computer.

### GitHub Contribution Guidelines
Please create a new branch from the latest version of main branch for your contribution. Create one branch for each individual feature / fix. After you finish writing and testing your feature / fix, open a pull request to merge your branch with the main. Please write meaningful comments to your commits and pull requests.

#### How to Branch
```
git checkout main
```

```
git pull
```

```
git checkout -b <new_branch_name> main
```
#### Naming your branch: 
Please name your branch as `<subteam-initial>/<your-name>/<short-description-of-feature-or-fix>`

For `<subteam-initial>`, use `n` for navigation, `i` for image recognition, `e` for extermination, `c` for chassis.

#### How to open a pull request
* Open the branch page in GitHub.
* Use the `contribute` button to open a PR (pull request).
* Your code is ready to be reviewed and merged. Great work!

### ROS Contribution Guidelines

#### Setting Up A New Package
Please create a new package only if it is agreed in an all-subteams meeting, so the file system does not get too complicated. Thank you.

* Open a new terminal
* Go into the src of the workspace
```
cd ~/<path_to_folder_workspace_is_in>/jetson_ros_workspace/src
```
* Use catkin_create_pkg script to create a package with dependencies you need. The usual dependencies `std_msgs rospy roscpp` are given below. You can add other dependencies.
```
catkin_create_pkg <name_of_your_package> std_msgs rospy roscpp
```
Go back to workspace folder and rebuild.
```
cd ..
catkin_make
```

#### Creating A New Node (Python)
* Open a new terminal
* Go into the scripts of the package you are working on. (If there is no scripts folder in src, go into package src and use `mkdir scripts`)
```
cd ~/<path_to_folder_workspace_is_in>/jetson_ros_workspace/src/<your_package>/src/scripts
```
* Create the script with `touch`. Be sure the script name is unique in the package and tells what the script does.
```
touch <script_name>.py
```
* Make the script executable
```
chmod +x <script_name>.py
```
* Open the CMakeList of the package you are working on using any text editor. `vim` for vim, `nano` for nano, `code` for VS Code, etc.
```
vim ../../CMakeLists.txt
```
* Add the following code to the file. According to ROS Tutorials, this is necessary for python script to install properly and use the correct interpreter.
```
catkin_install_python(PROGRAMS 
  src/scripts/<script_name>.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
* Go to workspace folder and build.
```
cd ~/<path_to_folder_workspace_is_in>/jetson_ros_workspace
catkin_make
```
#### Publishing To A New Topic
* All topics need to be unique throughout the system. Be sure there is no other topic with the same name in any package.
* Creating a publisher in your node script with `pub = rospy.Publisher('<topic_name>', String, queue_size=1)` declares the topic with `<topic_name>`.
