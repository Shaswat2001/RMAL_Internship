# Setting Up ORB-SLAM2

## Creating a workspace

To create a workspace in Ubuntu, run the following commands in the terminal - 


```python
mkdir -p ~/slam_ws/src
cd ~/slam_ws/src
catkin_init_workspace
```

Then move to the slam_ws directory -

```python
cd ..
catkin_make
```

To overlay the workspace on top of the ROS environment -

```python
source devel/setup.bash
```

## Installing Prerequisites for ORB-SLAM

All the packages are installed in the "src" folder on the created workspace.

### 1. Panglolin

To download and build pangolin package, execute the following - 

```python
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

Check https://github.com/stevenlovegrove/Pangolin to make sure all the dependencies for panglolin are installed.

### 2. OpenCV

I installed OpenCV 4.5.3. To build the core modules, run the following - 
**Make sure the OpenCV version used to create ORB-SLAM is the same as the one installed using ROS**

```python
sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
unzip opencv.zip
mkdir -p build && cd build
cmake  ../opencv-master
cmake --build .
```

## Building the OPB-SLAM Package

Clone the repository:
```python
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

Now to successfully build the ORB-SLAM package, certain changes have to be made in the files-

### 1. CmakeLists.txt

The CmakeLists file of the following folders have to be changed -
* ORB-SLAM
* ORB_SLAM2/Thirdparty/DBoW2
* ORB_SLAM2/Examples/ROS/ORB_SLAM2

Replace the following statements - 

```python
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 2.4.3 QUIET)
```
Replace the above statements with - 

```python
find_package(Eigen3 REQUIRED)
find_package(OpenCV QUIET)
```

### 2. Error : usleep is not declared

Import these libraries -

```C
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
```
The following files have to be changed - 

* ORB_SLAM2/src/LocalMapping.cc
* ORB_SLAM2/src/LoopClosing.cc
* ORB_SLAM2/src/System.cc
* ORB_SLAM2/src/Tracking.cc
* ORB_SLAM2/src/Viewer.cc
* ORB_SLAM2/Examples/Monocular/mono_euroc.cc
* ORB_SLAM2/Examples/Monocular/mono_kitti.cc
* ORB_SLAM2/Examples/Monocular/mono_tum.cc
* ORB_SLAM2/Examples/Stereo/stereo_euroc.cc
* ORB_SLAM2/Examples/Stereo/stereo_kitti.cc
* ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc
* ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/AR/ViewerAR.cc

### 3. CV2 Errors 

Import these libraries -

```C
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
using namespace cv;
```

Also remove the following line -

```C
#include <opencv/cv.h>
```

The following files have to be changed - 

* ORB_SLAM2/src/FrameDrawer.cc
* ORB_SLAM2/src/Sim3Solver.cc
* ORB_SLAM2/src/PnPsolver.h
* ORB_SLAM2/src/ORBextractor.h
* ORB_SLAM2/Examples/Monocular/mono_euroc.cc
* ORB_SLAM2/Examples/Monocular/mono_kitti.cc
* ORB_SLAM2/Examples/Monocular/mono_tum.cc
* ORB_SLAM2/Examples/Stereo/stereo_euroc.cc
* ORB_SLAM2/Examples/Stereo/stereo_kitti.cc
* ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc

### 4. Change in LoopClosing.h

Replace the statement at line 50 with -

```C
Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
```

## Building ORB-SLAM

After making the above changes ORB-SLAM can be built succcessfully. 

```python
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

## Building the nodes for mono, monoAR, stereo and RGB-D

Add the following path in the .bashrc file.

```python
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
```
Example -

```python
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/sis/slam_ws/src/ORB_SLAM2/Examples/ROS
```

Execute build_ros.sh script - 

```python
chmod +x build_ros.sh
./build_ros.sh
```

# Testing the installation

 Open 3 tabs on the terminal and run the following command at each tab:

 ```python
roscore
```
 ```python
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
```
 ```python
rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
```

Install the V1_01_easy.bag file from  http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets