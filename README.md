# Donkey ROS

## Donkey car with ROS!!

<p align="center">
    <img src="./Images/blob_tracking.gif" width="400" />
    <img src="./Images/joy_control.gif" width="400" />
</p>


> How to make it?

There's Notion Lecture Notes and Youtube video's about this project. 
But, It's written in Korean. Anyway, Here's the link

* [Notion Lecture Notes](https://www.notion.so/ROS-for-G-Camp-410d95df137d403ca176cfec4822351a)
* Youtube Lecture Video - **Coming Soon!!**

## Tested System information

**Jetson Nano 4GB**

* Ubuntu 18.04
* ROS Melodic
* Opencv4

## Packages with Brief Explanation

```
├── csi_camera => Handling Image data For IMX219 Camera  
├── donkey_control => Control RC Car with Adafruit PCA9685
├── donkey_cv => Computer Vision Package with Opencv4 
├── donkey_joy => Control RC Car with Logitech F710 Game Controller 
│
(...)
├── Images
├── LICENSE
├── README.md
└── STL_Files
```

## Prerequisite

1. Ros Packages installation
   
```bash
$ sudo apt-get install ros-melodic-cv-bridge
$ sudo apt-get install ros-melodic-image-view
```

2. OpenCV4 installation

- Can be found in [JetsonHacks repo](https://github.com/JetsonHacksNano/buildOpenCV)

3. Clone this Repo

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Road-Balance/donkey_ros.git

$ cd ../
$ catkin_make
$ source devel/setup.bash
```

## Usage

1. csi_camera package

Packages for Image Streaming

> Check Camera Connection First!!!

```bash
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=2 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e
```

* `sensor_id` : this value depends on Camera Slot in Jetson Nano.

### Webcam Puslish

```bash
$ roscore

$ rosrun csi_camera webcam_pub.py
$ rosrun image_view image_view image:=/csi_image
```

### CSI Camera Publish

```bash
$ roscore

$ rosrun csi_camera csi_pub.py
$ rosrun image_view image_view image:=/csi_image
```

2. donkey_control package

Packages for controlling `RC Car` with `PCA9685` PWM driver.
You need to install `Adafruit_PCA9685` python package first 

There's two modes for controlling RC Car

* JoyStick Control
* Blob Control

```bash
$ roscore

$ rosrun donkey_control joy_control.py
$ rosrun donkey_control blob_chase.py
```

3. donkey_joy package

There's two modes for using joystick

* Button mode
* Axes mode

```bash
$ roslaunch donkey_joy joy_teleop_axes.launch
$ roslaunch donkey_joy joy_teleop_btns.launch
```

4. donkey_cv package

Packages for OpenCV applications

* Find Blob with Certain color
* Publish Image location as a `geometry_msgs/Point`

```bash
$ roscore

$ rosrun donkey_cv find_ball.py
```

## Application

### **1. joy_control**

Control RC Car with logitech F710 game controller

<p align="center">
    <img src="./Images/joy_control.gif" width="500" />
</p>

```bash
$ roscore

# Jetson
$ rosrun donkey_control joy_control.py

# Laptop or ROS Slave PC
$ roslaunch donkey_joy joy_teleop_axes.launch
# or
$ roslaunch donkey_joy joy_teleop_btns.launch

```

### **2. blob_tracking**

Find the green box of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/blob_tracking.gif" width="500" />
</p>


```bash
$ roscore

$ rosrun csi_camera csi_pub.py
$ rosrun donkey_cv find_ball.py 
$ rosrun donkey_control chase_the_ball.py 
$ rosrun donkey_control blob_chase.py 
```

Debugging with `image_view`

```bash
rosrun image_view image_view image:=/webcam_image
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob
```

---

Nothing

```
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=2 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e

sudo apt-get install ros-melodic-cv-bridge
sudo apt-get install ros-melodic-image-view

rosrun csi_camera webcam_pub.py
rosrun image_view image_view image:=/csi_image

take photo

python range_detector.py --image frame0000.jpg --filter HSV --preview

rosrun donkey_cv find_ball.py

rosrun image_view image_view image:=/csi_image
rosrun image_view image_view image:=/webcam_image
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob

roslaunch donkey_control blob_control.launch 

rosrun csi_camera csi_pub.py
rosrun donkey_cv find_ball.py 
rosrun donkey_control chase_the_ball.py 
(env)  rosrun donkey_control blob_chase.py 

```
