
#ifndef OBJECT_DETECTION_2D_H
#define OBJECT_DETECTION_2D_H

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold, 
                   std::string basePath, std::string classesFile, std::string modelConfiguration, std::string modelWeights, bool bVis);

#endif 
