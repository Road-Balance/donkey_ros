# donkey_ros
donkey car with ROS!!

```
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
```
