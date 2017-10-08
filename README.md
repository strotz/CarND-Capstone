## System Integration Project

### Team

Alexei Strots (strotz@gmail.com)  
Ilya Gerasimets (ilya.gerasimets@gmail.com)  
Barney Kim (illumine03@naver.com)  

### Objective

To gain experience with ROS (Robot Operating System) and ROS modules development. Implement system that controls vehicle steering wheel, acceleration and brakes. Ensure that car can follow the route defined by set of waypoints; can detect state of traffic light from provided camera images; maintain desired velocity and stop when there is a red traffic light ahead.

### Overview 

There are 3 ROS modules modified: 

* **waypoint_updater** - module that receive list of map waypoints, upcoming traffic light info and current position and orientation of the car. The goal of this module is to generate short list of waypoints ahead of the car that will be used for short term trajectory calculation. Also this module enriches waypoints with linear velocity information.  It order to do it, module contains builders of various speed profiles (stop, run, slow down)  

* **tl_detector** - module responsible for localization of upcoming traffic light and detection of its state. It receives list of waypoints and current vehicle position and orientation, position of traffic lights on the map and image from camera. It uses this information to calculate position of stop line before traffic light, precise position of the light in image and control classifier that detects state of traffic light.

* **twist_controller** - module that translates desired vehicle's linear and angular velocity to throttle, brake and steering signals. It implements PID and yaw controllers.

### Waypoint Updater

waypoint_updater.py contains implementation of module that builds motion and speed profile for the vehicle based on its current position and surroundings. As a first step we have to find nearest waypoint in front of the vehicle. In order to do it, we looped through all waypoints and first, filtered out items that has negative correlation between car orientation and vector drown from car to waypoint. Then, we found one that has min distance to vehicle current position (square root operation is omitted for performance reason). Same approach we used in traffic light detector to find closest light in front of he car.

Assuming that waypoints in complete map ordered to maintain desired direction, we copied next N elements and dealt with fact that map has ring nature, i.e. when there are not enough elements in tail of source array, read the required elements from the beginning.

Then we need to generate velocity profile. We use data from "/traffic_waypoint" channel to calculate distance to stop line of red or yellow traffic light. Originally, we used simple rule to assign speed to MAX when there is no red signal ahead and 0 if the is one in close proximity. Then PID controlled dealt with large acceleration and jerk. Later, we added "slow down" profile that gradually decreases velocity, based on distance to stop line. Linear decrease was not enough sometimes to stop the car, so we switch to paraboloid descent.

```
a = (way_to_go - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE) # between 0 and 1
v = a * a * MAX_SPEED # x^2
```
 
It would be nice to be able to use data from "/current_velocity" in this module. Such data can open posibilities to generate better predictions and exact speed profiles. Yet, site bag does not contain this signal and it limits funtionality of this module.
 
### Traffic Light Detector
 
### Twist Controller

twist_controller.py file contains a stub of the Controller class. We implemented the control method to return throttle, brake, and steering values. We have also added a reset method to prevent the PID controller from accumulating errors when the safety driver is taken over.

The throttle of the car was calculated based on the current velocity and the target velocity. A generic PID controller in pid.py was used for error correction. After several trial and error and tuning, the PID controller used the following parameters.
```
kp = 6.0
ki = 0.25
kd = 0.4
```

The brake torque is based on multiple parameters. Vehicle acceleration, as well as mass of the vehicle, weight of fuel, radius of the wheel, and other values have been considered. The brake torque is in N/m and the formulae used for calculting the brake is as follows.
```
total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
longitudinal_force = total_mass * acceleration
brake_torque = longitudinal_force * wheel_radius
```

Finally, the steering angle was calculated based on the current linear velocity and the target linear and angular velocity. The YawController in yaw_controller.py was used to convert target linear and angular velocity to steering commands.

# Content from Udacity:

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
