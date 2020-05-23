## 安装usb_cam功能包
```
sudo apt-get install ros-kinetic-usb-cam
```
摄像头测试：
```
roslaunch usb_cam usb_cam-test.launch
```
运行usb_cam_node节点：
```
rosrun usb_cam usb_cam_node
```
可使用以下工具查看摄像头数据话题：
```
rqt_image_view
```
## 安装OpenCV库
```
sudo apt-get install ros-kinetic-vision-opencv libopencv-dev python-opencv
```
测试：
```
rosrun vision_mod cv_bridge_test.py
```
## 人脸识别
```
roslaunch vision_mod usb_cam.launch
roslaunch vision_mod face_detector.launch

```
其他：
```
roslaunch vision_mod uvc_camera_with_calibration.launch
roslaunch vision_mod ar_track_camera.launch
rostopic echo /ar_pose_marker

roslaunch vision_mod freenect.launch
roslaunch vision_mod ar_track_kinect.launch
```