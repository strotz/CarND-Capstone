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

#### Contour Classifier

tl_detector.py contains implentation of algorithms which decides if a traffic light is close enough, cuts region of interest (ROI() from the image, calls classifier, and reports upcoming traffic light state.

It has the following methods:

- 'get_closest_waypoint' identifies the closest path waypoint to the given position;
- 'to_car_coordinates' converts a world coordinate point to car coordinate system (to find ROI of a traffic light);
- 'project_to_image_plane_1' projects point from 3D world coordinates to 2D camera image location;
- 'get_light_state' gets a camera image, cuts ROI, and sends it to classifier;
- 'load_stop_line_waypoints' stop line waypoints from config;
- 'process_traffic_lights' contains core algorithm which
1) finds traffic light closest to the car (and in front of it);
2) decides if it is too far or not (the limit is 70 meters);
3) finds correct coordinate transformation to build ROI;
4) sends ROI for classification;
5) reports the traffic light state to the caller.

contour_classifier.py contains implementation of traffic light classifier, which determines the color of the traffic light in the image. It uses computer vision library OpenCV to analyze incoming images and decide if there is a traffice ligth and what it's color.

The algorithm works as following:

- the input parameter is a part of the image presumably containing a traffic light (Region Of Interest);
- validates the incoming image (line 23);
- converts the image to grayscale, inverts it, and removes too bright components (lines 27-28);
- searches for the large objects on the image using OpenCV 'findContours' algorithm (line 30);
- from all contours it keeps only a few largest with size at least 1% of original image but no more than 20% (lines 32-36);
- uses the contours (as a mask) to crop the original imagel (lines 38-40);
- extracts color information from the original image by building histograms (lines 42-44);
- makes dicision as following (lines 46-63):
1) if the bright areas of histogram contain green (at least 40%) and red (at least 40%) - it is yellow;
2) if the bright areas of histogram contain at least 50% red - it is red;
3) if the bright areas of histogram contain at least 50% green - it is green;
4) otherwise the color is unknown.

#### SSD Classifier

The above-mentioned classifier focuses on finding out what color the traffic lights are. However, in order to classify the color of the traffic light by using the above detector, firstly, it is necessary to know the exact position of the traffic light. Object detection is about deciding where exactly in the image are objects belonging to certain categories such as cars, dogs etc.

A very promising family of object detectors, are deep learning networks that receive an image as an input, run the network on the image, and output a list of detections. Since these networks run through the image only once, they are referred to as “single shot” detectors.

ssd_classifier.py contains a classifier implementation that identifies where the traffic lights are in the image. Using the pre-trained SSD model, we were able to easily find the light of an image.

It has the following methods:

1) Load frozen graph.
2) Receive an image as an input, run the network on the image, and output a list of detections.
3) Select the one with the highest score among the detected objects.
4) Determines the color of the traffic light.

#### Hybrid Classifier

A pre-trained model that we can easily obtain can locate the location of a traffic light, but most of the time it is not possible to distinguish the color of the traffic light. However you can fine tune them on your own data by changing the number of categories in the last layer of the network, possibly removing / adding layers, and retraining on your data. Even if you fine tune them using existing pre-trained networks, requires a lot of data and long training time to get good results.

Since SSD classifier correctly detects locations of traffic light, we decided to implement a hybrid classifier. It is implemented in the hybrid_classifer.py file and works as follows.

1) Detect location of traffic light and it's color.
2) The color of the traffic light is estimated by applying the contour classifier method to the detected traffic light image.
3) Select one of the two estimates with a higher score.

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
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
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
