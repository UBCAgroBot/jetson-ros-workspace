# Create a ROS development environment in AWS for free

## Create EC2 Instance and connect

1. Go to EC2 > Instances > Launch Instannce
2. Choose AMI Image `Ubuntu Server 18.04 LTS (HVM), SSD Volume Type`
3. Choose `t2.micro` Instance type. (It should say "free tier eligible")
4. press `Review and launch` > press `launch`
5. Create a key pair and download the .pem file. (If you don"t have one yet)
6. Press `Launch`
7. Go back to EC2 > Instances page
8. Find `Public IPv4 DNS` of the instance on page.
9. Go to terminal (or what you use in windows, sorry.)
10. Use command below for ssh connection
```
ssh -i [Address of .pem file] ubuntu@[Public IPv4 DNS]
```
11. You are connected to the server. Congrats!

## Download necessary libaries

1. Follow [this page](http://wiki.ros.org/melodic/Installation/Ubuntu) for ROS melodic
2. Choose "Desktop Install" option

## Clone jetson-ros-workspace from GitHub to EC2

1. [Create a personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) on GitHub
2. Clone the repo using the command below
``` 
git clone https://<your-access-token>@github.com/UBCAgroBot/jetson-ros-workspace.git
```
3. Open a terminal and navigate into the jetson-ros-workspace folder
4. Use the command below to build
```
catkin_make
```

## Run

* Open two terminals
* **On terminal 1:**
```
cd jetson-ros-workspace
```

```
. devel/setup.bash
```
```
roscore
```
3. **On terminal 2:**
```
cd jetson-ros-workspace
```
```
. devel/setup.bash
```
```
rosrun navigation [script_you_want_to_run.py]
```

**Note:** If ROS can't find a package or script:
```
. ~/catkin_ws/devel/setup.bash
```

**Note 2: Do not forget to terminate EC2 instance when you are done**

