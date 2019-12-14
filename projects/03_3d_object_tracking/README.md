# Project 3: 3D Object Tracking

<img src="./ttc.gif" width="1242" height="375" />

This is the third project for the [**Sensor Fusion NanoDegree**](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313) at Udacity. For this project, you have to estimate the time-to-collision (TTC) for the immediately preceeding vehicle in the ego lane. The animation shows TTC estimates based on LiDAR and Camera for sample image dataset available [here](https://github.com/udacity/SFND_3D_Object_Tracking).

Note that TTC-LiDAR is estimated using the simple constant velocity model (CVM) instead of the more accurate and complex constant acceleration model (CAM). The choice of CVM over CAM possibly leads to erroneous estimates for multiple frames. On the other hand, the robustness of TTC-Camera heavily depends upon the exact matching of keypoints in consecutive frames. Moreover, the precision of 2D bounding box affects both estimates. A comparison is provided in "output.csv" file for different detector-descriptor combinations.

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Note that to build OpenCV with the proprietary modules, it needs to be built with the variable **OPENCV_ENABLE_NONFREE** turned on.

```bash
$> cmake -D OPENCV_ENABLE_NONFREE:BOOL=ON -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
```

## Basic Build Instructions

1. Make a build directory in the top level project directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./3D_object_tracking`.
