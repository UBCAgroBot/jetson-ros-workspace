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
