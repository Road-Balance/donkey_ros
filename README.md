# donkey_ros
donkey car with ROS!!

```
python range_detector.py --image frame0000.jpg --filter HSV --preview

rosrun csi_camera webcam_pub.py

rosrun donkey_cv find_ball.py

rosrun image_view image_view image:=/webcam_image
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob
```