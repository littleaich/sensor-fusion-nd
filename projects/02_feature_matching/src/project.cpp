/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;


int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis{false}, bWrite{false};            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
  	// just for bookkeeping
  	static vector<int> numKeyPoints;
  	static vector<float> exeTime;
	static vector<int> numMatches;  
  	float exeTimeSingle;
  
    for (size_t imgIndex{0}; imgIndex <= imgEndIndex - imgStartIndex; ++imgIndex)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() == dataBufferSize) {
            dataBuffer.pop_front();
        }
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        // string detectorType = "shi-tomasi";
        // string detectorType = "harris";
        // string detectorType = "fast";
        // string detectorType = "brisk";
        // string detectorType = "orb";
        // string detectorType = "akaze";
        string detectorType = "sift";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        detectKeyPoints(detectorType, keypoints, imgGray, exeTimeSingle, false);

		// just for bookkeeping
  		numKeyPoints.push_back(keypoints.size());
  		exeTime.push_back(1000 * exeTimeSingle);      
      
        //// EOF STUDENT ASSIGNMENT

      
      
        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle {true};
        cv::Rect vehicleBBox(535, 180, 180, 150); // x, y, w, h
        if (bFocusOnVehicle) {
            int lastIdxOnVehicle{-1};
            for (int i{0}; i<keypoints.size(); ++i) {
                if ( vehicleBBox.contains( keypoints[i].pt ) ) {
                    // swap last index and i
                    ++lastIdxOnVehicle;
                    cv::KeyPoint tmp = keypoints[lastIdxOnVehicle];
                    keypoints[lastIdxOnVehicle] = keypoints[i];
                    keypoints[i] = tmp;
                }
            }

            if (lastIdxOnVehicle > -1) {
                keypoints.erase( keypoints.begin()+lastIdxOnVehicle, keypoints.end() );
            } 
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts {false};
        if (bLimitKpts)
        {
            int maxKeypoints {10};
            if (keypoints.size() > maxKeypoints) { 
                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                cout << " NOTE: Keypoints have been limited!" << endl;
            }          
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        // EXTRACT KEYPOINT DESCRIPTORS

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        // std::string descriptorType = "brisk"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        // std::string descriptorType = "brief";
        // std::string descriptorType = "orb";
        // std::string descriptorType = "freak";
        // std::string descriptorType = "akaze";
        std::string descriptorType = "sift";

        extractDescriptors(descriptorType, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, exeTimeSingle);
		exeTime[imgIndex] += 1000 * exeTimeSingle; 	      
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

     
        if (dataBuffer.size() < 2) continue; // wait until at least two images have been processed
        
        // MATCH KEYPOINT DESCRIPTORS

        // define and initialize variables for matching descriptors
        vector<cv::DMatch> matches;
        string matcherTypeMatching = "MAT_BF";        // MAT_BF, MAT_FLANN
        string descriptorTypeMatching = "DES_HOG"; // DES_BINARY, DES_HOG
        string selectorTypeMatching = "SEL_NN";       // SEL_NN, SEL_KNN

        //// STUDENT ASSIGNMENT
        //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
        //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

        matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                         (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                         matches, descriptorTypeMatching, matcherTypeMatching, selectorTypeMatching);         
		
      	numMatches.push_back(matches.size());
        //// EOF STUDENT ASSIGNMENT

        // select N matches for display
        bool showAllMatch {true};
        vector<char> matchMask;
        if (!showAllMatch) {
            matchMask.resize(matches.size(), 0);
            int N{10}, stride{ static_cast<int>(matches.size()/N)};
            for (int i{0}; i<matches.size(); i+=stride) {
                matchMask[i] = 1;
            }                   
        }
     

        // store matches in current data frame
        (dataBuffer.end() - 1)->kptMatches = matches;

        cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

        // visualize matches between current and previous image
        if (bVis || bWrite) {
            cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
            cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                            (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                            matches, matchImg,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            matchMask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);     	

            if (bVis) {
                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, cv::WINDOW_NORMAL);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }

            if (bWrite) {
                cv::imwrite("./image-" + std::to_string(imgIndex-1) + "-" + std::to_string(imgIndex) + ".png", matchImg);
            }
        }
   
    } // eof loop over all images
  
    int numKeyPointsAvg {0};
    float exeTimeAvg {0.0f};
  	int numMatchesAvg {0};
    for (int i{0}; i<numKeyPoints.size(); ++i) {
        numKeyPointsAvg += numKeyPoints[i];
        exeTimeAvg += exeTime[i];
      	std::cout << "Total execution time for Image-" << i << " =" << exeTime[i] << " ms" << std::endl;         
    }
  
    for (int i{0}; i<numMatches.size(); ++i) {
        numMatchesAvg += numMatches[i];
      	std::cout << "Number of matches for Images-(" << i << "," << i+1 << ") =" << numMatches[i] << std::endl;         
    }  
    std::cout << "Average number of key points = " << numKeyPointsAvg / numKeyPoints.size() << std::endl;
    std::cout << "Average execution time for detecting key points = " << exeTimeAvg / numKeyPoints.size() << " ms" << std::endl; 
  	std::cout << "Average number of matches = " << numMatchesAvg / numMatches.size() << std::endl;

    return 0;
}