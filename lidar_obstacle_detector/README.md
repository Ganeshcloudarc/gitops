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

**Known Issues**
* PCA Bounding Boxes might not be accurate in certain situations

## Dependencies
* autoware-msgs
* jsk-recognition-msgs

## Installation
```bash
# clone the repo
cd catkin_ws/src

# install dependencies & build 
cd ..
catkin_make
source devel/setup.bash

```

## Test Cases and Failure scenarios for lidar based obstacle detection


- The vehicle stops only when the obstacle is on the path and not out of the path
- The vehicle stops only when the obstacle is 50cm far from the lidar.

- The vehicle should be tested with the following test cases:

   - Aprroaching in fron the vehicle on the path suddenly with less than 5m from the vehicle.
   
   - Walking in front the vehicle within 5m from the vehicle.
   
   - The ground is expected to more or less even(but too much uneven ground can cause little problems and the vehicle might detect the ground sometimes)
   
   - The stones or cones like very sharp and not much dense and small objects might not get detected.
 
 
   | Obstacle type | Results(INDIA)|Resuls(US)  |
   | ------------- | ------------- |------------|
   | Trees         | Detects       |            |
   | Humans        | Detects       |            |
   | Vehicles      | Detects       |            |
   | Fence         | Detects       |            |
   | Cones         | Fails         |            |
   | Very uneven ground| Fails(sometimes)|      |
   | Sudden jump(<5m)| Detects|                 |
   | Turnings(<5m)| Detects(once failed)|       |
   
   
 ```
   
```
