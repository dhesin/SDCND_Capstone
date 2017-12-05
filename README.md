### **Capstone Project Reflections**

The final project of System Integration from Udacity SDCND program is a good opportunity to have a look at a complete system. Although it only comes with traffic light detection and control subsystem, one can expand it to include other subsystems given that you have an access to a simulator which responds to these subsystems. I say - accessing a simulator - since Udacity simulators so far came as separate ones each have different obstacles and responding to only one subsystem such as path planning with other vehicles, light detection, pid controller. A simulator that combines other vehicles and traffic light detection along with other obstacles such as person, bicycle would be useful to exercise more complex relations and/or dependencies between subsystems. 

Recently, I had the chance to see and drive with Carla, Udacity's Self Driving Car, by downloading the code we have written. Since I didn't get to see the review and had no idea if the code run on simulator, I was a bit nervous but also excited. Luckily, Carla recognized the red light and stopped shortly after it started but was fast and furious once light turned green so the driver had to take over the control. Although it was very short time, running the code on Carla was exciting. 

#### Traffic Light Detection and Recognition

To solve the traffic light detection problem, we used <a href="https://github.com/tensorflow/models/tree/master/research/object_detection">the Tensorflow Object Detection API</a> , more specifically SSD Inception V2 Network trained with COCO images, to train and test the system. SSD inception paper can be found <a href="https://arxiv.org/pdf/1512.02325v5.pdf">here</a>. We transferred original network weights and kept training with real and simulated images for simulator and site operations respectively. Annotated real traffic light images are provided by Anthony Sarkis who is one of the peers in the Udacity Self Driving Car Nanodegree program. We thank him for making his data available to community. We needed to modify the configuration file provided by SSD Inception V2 api to setup number of classes, 4 in our case; red, yellow, green and unknown, batch size and number of training steps. Once tensorflow specific data records file is obtained, system was ready to train and test.


#### Vehicle Speed and Position Control

For vehicle speed and position control subsystem, we made use of existing yaw controller to control the steering angle of the vehicle and PID to control brake and throttle. PID output is scaled down by maximum speed when acceleration is needed and scaled up by a constant to decelerate. Deceleration constant is obtained by using vehicle mass, wheel radius, current speed and required speed. Update rate for control subsystem was set at 50Hz as originally set by framework. 

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
