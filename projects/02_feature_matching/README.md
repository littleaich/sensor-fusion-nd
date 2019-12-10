# Project 2: 2D Feature Matching

<img src="./keypoints.gif" width="828" height="125" />

This is the second project for the [**Sensor Fusion NanoDegree**](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313) at Udacity. For this project, you have to compare the quality and time-complexity of the descriptor matching algorithms available mostly in OpenCV. Feature descriptors are extracted from the keypoints detected from 2D images (grayscale).

For ***keypoint detection***, I explore the following algorithms:

* [Shi-Tomasi](https://docs.opencv.org/3.4/d8/dd8/tutorial_good_features_to_track.html)
* [Harris](https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html)
* [FAST](https://docs.opencv.org/3.4/df/d0c/tutorial_py_fast.html)
* [BRISK](https://docs.opencv.org/3.4/de/dbf/classcv_1_1BRISK.html)
* [ORB](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html)
* [AKAZE](https://docs.opencv.org/3.4/db/d70/tutorial_akaze_matching.html)
* [SIFT](https://docs.opencv.org/4.0.1/d5/d3c/classcv_1_1xfeatures2d_1_1SIFT.html)

Also, the ***decscriptor extraction*** algorithms investigated here are as follows:

* [BRISK](https://docs.opencv.org/3.4/de/dbf/classcv_1_1BRISK.html)
* [BRIEF](https://github.com/opencv/opencv_contrib/blob/master/modules/xfeatures2d/src/brief.cpp)
* [ORB](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html)
* [FREAK](https://docs.opencv.org/4.1.0/df/db4/classcv_1_1xfeatures2d_1_1FREAK.html)
* [AKAZE](https://docs.opencv.org/3.4/db/d70/tutorial_akaze_matching.html)
* [SIFT](https://docs.opencv.org/4.0.1/d5/d3c/classcv_1_1xfeatures2d_1_1SIFT.html)

Finally, [Brute-Force](https://docs.opencv.org/3.4/d3/da1/classcv_1_1BFMatcher.html) and [FLANN](https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html) with k-nearest neighbor (kNN) are employed for ***feature matching***.

The animation shows sample detection and matching results for sample image dataset available [here](https://github.com/udacity/SFND_2D_Feature_Tracking).

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

1. Make a build directory in the top level directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./2D_feature_tracking`.

Below I list the execution time along with the number of detected and matched features for several algorithms for all 10 images in the sample dataset and their averages.
The format in the table is #Matched\_Features [#Detected\_Features] (TOTAL_EXECUTION_TIME). The statistics is based on the default parameter settings in OpenCV. Also, the numbers are for the whole images (not only for the immediately preceding vehicle in the ego lane).
<!---
Detector+Descriptor+Match Comparison Table START
-->
<table class="tg">
  <tr>
    <th class="tg-uys7"></th>
    <th class="tg-uys7">Shi-Tomasi + BRISK</th>
    <th class="tg-uys7">Harris + BRISK</th>
    <th class="tg-uys7">FAST + BRISK</th>
    <th class="tg-uys7">BRISK</th>
    <th class="tg-uys7">ORB</th>
    <th class="tg-uys7">AKAZE</th>
    <th class="tg-uys7">SIFT</th>
  </tr>
    <tr>
    <th class="tg-uys7">Image-0</th>
    <th class="tg-uys7">N/A [1370] (32.51ms)</th>
    <th class="tg-uys7">N/A [339] (22.62ms)</th>
    <th class="tg-uys7">N/A [5063] (56.44ms)</th>
    <th class="tg-uys7">N/A [2757] (445.42ms)</th>
    <th class="tg-uys7">N/A [500] (14.90ms)</th>
    <th class="tg-uys7">N/A [1351] (165.64ms)</th>
    <th class="tg-uys7">N/A [1438] (264.38ms)</th>
  </tr>
  <tr>
    <th class="tg-uys7">Image-1</th>
    <th class="tg-uys7">800 [1301] (31.10ms)</th>
    <th class="tg-uys7">172 [286] (20.59ms)</th>
    <th class="tg-uys7">2496 [4952] (52.74ms)</th>
    <th class="tg-uys7">1520 [2777] (431.47ms)</th>
    <th class="tg-uys7">297 [500] (12.97ms)</th>
    <th class="tg-uys7">1012 [1327] (151.93ms)</th>
    <th class="tg-uys7">810 [1371] (207.25ms)</th>
  </tr>
  <tr>
    <th class="tg-uys7">Image-2</th>
    <th class="tg-uys7">771 [1361] (32.31ms)</th>
    <th class="tg-uys7">169 [349] (21.58ms)</th>
    <th class="tg-uys7">2450 [4863] (51.22ms)</th>
    <th class="tg-uys7">1506 [2741] (437.16ms)</th>
    <th class="tg-uys7">295 [500] (13.29ms)</th>
    <th class="tg-uys7">1008 [1311] (156.34ms)</th>
    <th class="tg-uys7">792 [1380] (203.52ms)</th>
  </tr>
  <tr>
    <th class="tg-uys7">Image-3</th>
    <th class="tg-uys7">752 [1358] (31.32ms)</th>
    <th class="tg-uys7">192 [356] (21.72ms)</th>
    <th class="tg-uys7">2441 [4840] (54.70ms)</th>
    <th class="tg-uys7">1523 [2735] (436.81ms)</th>
    <th class="tg-uys7">312 [500] (12.68ms)</th>
    <th class="tg-uys7">1017 [1351] (151.00ms)</th>
    <th class="tg-uys7">764 [1335] (208.39ms)</th>
  </tr>
  <tr>
    <th class="tg-uys7">Image-4</th>
    <th class="tg-uys7">769 [1333] (31.48ms)</th>
    <th class="tg-uys7">205 [521] (20.72ms)</th>
    <th class="tg-uys7">2422 [4586] (50.31ms)</th>
    <th class="tg-uys7">1449 [2757] (432.42ms)</th>
    <th class="tg-uys7">296 [500] (12.93ms)</th>
    <th class="tg-uys7">1016 [1360] (152.17ms)</th>
    <th class="tg-uys7">761 [1305] (207.07ms)</th>
  </tr>  
  <tr>
    <th class="tg-uys7">Image-5</th>
    <th class="tg-uys7">750 [1284] (30.60ms)</th>
    <th class="tg-uys7">375 [2611] (44.46ms)</th>
    <th class="tg-uys7">2472 [4599] (51.20ms)</th>
    <th class="tg-uys7">1474 [2695] (432.42ms)</th>
    <th class="tg-uys7">296 [500] (13.07ms)</th>
    <th class="tg-uys7">1014 [1347] (144.35ms)</th>
    <th class="tg-uys7">761 [1370] (203.46ms)</th>
  </tr>  
  <tr>
    <th class="tg-uys7">Image-6</th>
    <th class="tg-uys7">731 [1322] (30.71ms)</th>
    <th class="tg-uys7">162 [200] (19.79ms)</th>
    <th class="tg-uys7">2381 [4870] (51.26ms)</th>
    <th class="tg-uys7">1456 [2715] (438.42ms)</th>
    <th class="tg-uys7">293 [500] (13.27ms)</th>
    <th class="tg-uys7">1028 [1363] (158.92ms)</th>
    <th class="tg-uys7">743 [1396] (200.91ms)</th>
  </tr>   
  <tr>
    <th class="tg-uys7">Image-7</th>
    <th class="tg-uys7">762 [1366] (31.56ms)</th>
    <th class="tg-uys7">160 [806] (25.62ms)</th>
    <th class="tg-uys7">2290 [4868] (50.43ms)</th>
    <th class="tg-uys7">1401 [2628] (430.04ms)</th>
    <th class="tg-uys7">309 [500] (13.18ms)</th>
    <th class="tg-uys7">1014 [1331] (152.82ms)</th>
    <th class="tg-uys7">748 [1382] (203.88ms)</th>
  </tr>   
  <tr>
    <th class="tg-uys7">Image-8</th>
    <th class="tg-uys7">794 [1389] (32.26ms)</th>
    <th class="tg-uys7">372 [572] (23.04ms)</th>
    <th class="tg-uys7">2383 [4996] (52.18ms)</th>
    <th class="tg-uys7">1402 [2639] (443.40ms)</th>
    <th class="tg-uys7">295 [500] (12.49ms)</th>
    <th class="tg-uys7">1031 [1357] (152.70ms)</th>
    <th class="tg-uys7">817 [1463] (217.67ms)</th>
  </tr>
  <tr>
    <th class="tg-uys7">Image-9</th>
    <th class="tg-uys7">766 [1339] (31.36ms)</th>
    <th class="tg-uys7">419 [1471] (31.85ms)</th>
    <th class="tg-uys7">2427 [4997] (52.50ms)</th>
    <th class="tg-uys7">1413 [2672] (431.78ms)</th>
    <th class="tg-uys7">305 [500] (12.71ms)</th>
    <th class="tg-uys7">999 [1331] (151.17ms)</th>
    <th class="tg-uys7">824 [1422] (203.55ms)</th>
  </tr>   
    <tr>
    <th class="tg-uys7">Average</th>
    <th class="tg-uys7">766 [1342] (31.52ms)</th>
    <th class="tg-uys7">247 [751] (25.20ms)</th>
    <th class="tg-uys7">2418 [4920] (52.30ms)</th>
    <th class="tg-uys7">1460 [2711] (435.94ms)</th>
    <th class="tg-uys7">299 [500] (13.01ms)</th>
    <th class="tg-uys7">1015 [1342] (153.70ms)</th>
    <th class="tg-uys7">834 [1386] (212.01ms)</th>
  </tr>   
</table>
<!---
Detector+Descriptor+Match Comparison Table END
-->


Based upon the above statistics, the top 3 descriptors can arguably be (1) SIFT, (2) ORB, and (3) (FAST+BRISK). Although old-fashioned detectors are comparatively faster, their matching quality might not be reliable enough under complex autonomous driving scenarios.
