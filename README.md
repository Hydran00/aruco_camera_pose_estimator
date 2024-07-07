# aruco_camera_pose_estimator
This repo contains a ROS2 package that estimates the camera pose with respect to the world frame exploiting Aruco Markers. This package implements a [ROS2 Service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) that collects a set of $N$ measurements of the aruco position and orientation and computes the average. Then the resulting camera position and orientation is sent as the service response to the calling node. Each required parameter is loaded from a `.yaml` file.

# Installation
```
cd <path/to/your/ros2/workspace>/src
git clone https://github.com/Hydran00/aruco_camera_pose_estimator.git
cd .. 
colcon build --symlink-install --packages-select aruco_camera_pose_estimator
```
Then print your aruco using the 4x4 dictionary. You can use this [website](https://chev.me/arucogen/).

# Run the service
```
ros2 launch aruco_camera_pose_estimator aruco_camera_pose_estimator.launch.py
```
### Maths behind this implementation
The OpenCV Aruco Library let us compute $M_{c}^{a} \in \mathcal{R}^{4x4}$ that express the rototranslation of the Aruco with respect to the camera frame. 

Since we want $M_{w}^{c}$ (the camera with respect to the world frame), we can compute this matrix using homogeneous matrices multiplication as:

$M_{w}^{c} = M_{w}^{a}  M_{a}^{c}$  

where $M_{a}^{c} \in \mathcal{R}^{4x4}$ is the transpose of $M_{c}^{a}$ and $M_{w}^{a} \in \mathcal{R}^{4x4}$ is the rototranslation of the aruco marker with respect to the world frame.

**Note that if you just need the camera pose with respect to the aruco marker you can neglect $M_{w}^{a}$, i.e. setting it to equal to the identity matrix.**

### Parameters
#### ROS2 Parameters
  - `input_topic_name`: ROS2 input topic name of the camera image stream
  - `output_service_name`: ROS2 output service name of the computed camera pose
  - `aruco_marker_id`: id of the aruco in the dictionary. 
  - `n_observation`: the number of measurements used for the computation of the mean rototranslation (`timeout` should be set accordingly)
  - `timeout_ms`: the amount of time in milliseconds before the data collection is considered failed. Higher `n_observation` means more time to wait for the image stream. When the timeout is reached, the service response is sent having `error` in the `frame_id` field.  
  - `show_img`: wether to show the axis of the aruco when one is found.
  - `aruco_size`: the aruco edge size in centimeters
  - `aruco_XYZ_offset_from_baseframe`: XYZ displacement of the aruco with respect to the world frame. This corresponds to the entries (0,3), (1,3), (2,3) of $M_{w}^{a}$
  `aruco_rot_offset_from_baseframe`: orientation displacement of the aruco with respect to the world frame. It is specified in (w,x,y,z) quaternion forms. This, after the conversion to the rotation matrix, corresponds to the 3x3 matrix starting from (0,0) in $M_{w}^{a}$
#### Camera calibration parameters
Camera intrinsic and distortion coefficients can be set editing `calibration_params.yaml` under `config`.

camera intrinsic:
  - `cx`
  - `cy`
  - `fx`
  - `fy`    
These parameters are the one in the calibration matrix
$$
\begin{pmatrix}
  fx & 0 & cx \\
  0 & fy & cy \\
  0 & 0 & 1 \\
\end{pmatrix}
$$
distortion coefficients:
  - `k1`
  - `k2`
  - `k3`
  - `p1`
  - `p2`

**Note: The currently loaded parameters in the yaml file belongs to the RS435 @ 1920x1080.**

Each parameters can be set editing `config.yaml`

### Tips: changing aruco dictionary
If you need to use a different aruco dictionary you just need to update the following line in `image_processor.cpp`

```
dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::<your_dictionary_name>);
```