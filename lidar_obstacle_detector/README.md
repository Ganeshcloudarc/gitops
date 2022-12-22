# lidar_obstacle_detector

3D LiDAR Object Detection &amp; Tracking using Euclidean Clustering &amp; Hungarian algorithm

![](media/Screenshot%20from%202022-06-23%2012-37-13.png)
![](media/Screenshot%20from%202022-06-23%2012-37-18.png)
![](media/Screenshot%20from%202022-06-23%2012-37-26.png)

## Features
* Segmentation of ground plane and obstacle point clouds
* Customizable Region of Interest (ROI) for obstacle detection
* Customizable region for removing ego vehicle points from the point cloud
* Tracking of obstacles between frames using IOU gauge and Hungarian algorithm
* In order to help you tune the parameters to suit your own applications better, all the key parameters of the algorithm are controllable in live action using the ros param dynamic reconfigure feature

**TODOs**
* LiDAR pointcloud motion undistortion
* Drive Space/Kurb Segmentation
* Refine PCA Bounding Boxes by L-Shape fitting
* Add trackers such as UKF

**Known Issues**
* PCA Bounding Boxes might not be accurate in certain situations

## Dependencies
* autoware-msgs
* jsk-recognition-msgs

## Installation
```bash
# clone the repo
sudo apt install ros-noetic-jsk*
cd catkin_ws/src
git clone https://github.com/SS47816/lidar_obstacle_detector.git

# install dependencies & build 
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

```
