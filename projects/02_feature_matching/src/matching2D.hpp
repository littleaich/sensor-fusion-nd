#ifndef MATCHING2D_H
#define MATCHING2D_H

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"


void detectKeyPoints(std::string detectorType, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, float &exeTime, bool bVis=false);
void extractDescriptors(std::string descriptorType, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, float &exeTime);
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);

void detectKeyPointsShiTomasi(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsHarris(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsFAST(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsBRISK(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsORB(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsAKAZE(std::vector<cv::KeyPoint> &, cv::Mat &);
void detectKeyPointsSIFT(std::vector<cv::KeyPoint> &, cv::Mat &);


#endif /* matching2D_hpp */
