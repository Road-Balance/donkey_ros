# donkey_ros
donkey car with ROS!!

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
