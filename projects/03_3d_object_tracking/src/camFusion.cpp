
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, \
						float shrinkFactor, cv::Mat & P_rect_xx, cv::Mat & R_rect_xx, cv::Mat & RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat point_h_w(4,1,cv::DataType<double>::type); // homogeneous point in world coordinate (x,y,z,w)
    cv::Mat point_h_i(3,1,cv::DataType<double>::type); // homogeneous point in image coordinate (x,y,z)
  
    for(auto it1=lidarPoints.begin(); it1!=lidarPoints.end(); ++it1) {
        // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
		point_h_w.at<double>(0,0) = it1->x;
      	point_h_w.at<double>(1,0) = it1->y;
      	point_h_w.at<double>(2,0) = it1->z;
      	point_h_w.at<double>(3,0) = 1;
        // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera. 
        // Store the result in Y.
      	point_h_i = P_rect_xx * R_rect_xx * RT * point_h_w;

        // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point pt;
      	pt.x = point_h_i.at<double>(0,0) / point_h_i.at<double>(2,0); // x/z
      	pt.y = point_h_i.at<double>(1,0) / point_h_i.at<double>(2,0); // y/z

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check whether point is within current bounding box
            if (smallerBox.contains(pt)) {
				enclosingBoxes.push_back(it2); // save iterator for pointer manipulation later
            }
        } // eof loop over all bounding boxes
      
      	// check whether point is in a single or multiple boxes
      	if (enclosingBoxes.size() == 1) {
         	enclosingBoxes[0]->lidarPoints.push_back(*it1);
          	// check if this point has the current min in X direction
          	enclosingBoxes[0]->xminLidar = std::min(enclosingBoxes[0]->xminLidar, it1->x);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
	// create topview image
  	cv::Mat topViewImg ( imageSize, CV_8UC3, cv::Scalar(220, 220, 220) );
  
  	for (auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1) {
     	// create random color for the current 3D box
      	cv::RNG rng(it1->boxID);
      	cv::Scalar color { cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150) ) };
        
		// plot lidar points into topview image
		int top {(int)1e8}, left {(int)1e8}, bottom {0}, right {0}; 
		float xwmin {1e8}, ywmin {1e8}, ywmax {-1e8}; 
		
		for (auto it2=it1->lidarPoints.begin(); it2!=it1->lidarPoints.end(); ++it2) {
         	// world coordinates
          	float xw {it2->x};
          	float yw {it2->y};
			xwmin = (xwmin < xw ? xwmin : xw);
          	ywmin = (ywmin < yw ? ywmin : yw);
          	ywmax = (ywmax > yw ? ywmax : yw);
          
          	// topview coordinates
          	int y { (-xw * imageSize.height / worldSize.height) + imageSize.height };
          	int x { (-yw * imageSize.width / worldSize.width) + imageSize.width / 2 };
          
          	// find enclosing rectangle
          	top = (top < y ? top : y);
          	left = (left < x ? left : x);
          	bottom = (bottom > y ? bottom : y);
          	right = (right > x ? right : x);
          
          	// draw individual point
          	cv::circle(topViewImg, cv::Point(x,y), 4, color, cv::FILLED);
          	
        } // EOL for lidar points within a bounding box
                                      
		// draw enclosing rectangle
        cv::rectangle(topViewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0,0,0), 2); // last arg = thickness

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%id, #pts=%d", it1->boxID, (int)it1->lidarPoints.size() );
        cv::putText( topViewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, color);
      	std::cout << "Box=" << it1->boxID << "\tXmin " << xwmin << "\tWidth " << ywmax-ywmin << std::endl;
        sprintf(str2, "xmin=%2.2f m, width=%2.2f m", xwmin, ywmax-ywmin);
        cv::putText(topViewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, color);                                      
                                      
    } // EOL for a single bounding box
                                                                           
    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topViewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 255, 0), 3);
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topViewImg);
	if (bWait) {
    	cv::waitKey(0); // wait for key to be pressed                                      
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // remove outliers based on statistics of matching distance
  	// prev = queryIdx, curr = trainIdx
  	float meanDist {0.0f};
  	float maxDist {-1.0f};
  	// get mean distance first
  	for (const auto &kptMatch : kptMatches) {
    	meanDist += kptMatch.distance;
      	maxDist = std::max(maxDist, kptMatch.distance);
    }
  	meanDist /= kptMatches.size();
  
  	float distSpread {0.9};
  	maxDist = meanDist + distSpread * (maxDist - meanDist);
  	float shrinkFactor {0.05f}; // shrink the box by this factor on each side
  	std::vector<cv::DMatch> kptMatchesInliers;
  	for (const auto &kptMatch : kptMatches ) {
      	const int prevIdx {kptMatch.queryIdx};
      	const int currIdx {kptMatch.trainIdx};
    	meanDist += kptMatch.distance;
        // shrink current bounding box slightly to avoid having outliers on box edges
        cv::Rect smallerBox;
        smallerBox.x = boundingBox.roi.x + shrinkFactor * boundingBox.roi.width;
        smallerBox.y = boundingBox.roi.y + shrinkFactor * boundingBox.roi.height;
        smallerBox.width = boundingBox.roi.width * (1 - 2*shrinkFactor);
        smallerBox.height = boundingBox.roi.height * (1 - 2*shrinkFactor);
    	
      	if ( smallerBox.contains( kptsCurr[currIdx].pt ) && (kptMatch.distance < maxDist) ) {
        	kptMatchesInliers.push_back(kptMatch);  
        }
    }
  
  	boundingBox.kptMatches = kptMatchesInliers;
  	
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
  	int medianIdx1 = static_cast<int>((distRatios.size()-1)/2);
  	int medianIdx2 = static_cast<int>(distRatios.size()/2);
  	double medianDistRatio = (distRatios[medianIdx1] + distRatios[medianIdx2]) / 2; 
  
    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC,
                     double xminLidarPrev, double xminLidarCurr)
{
    // auxiliary variables
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
  	double yleft = -laneWidth/2;
  	double yright = laneWidth/2;
  
  	xminLidarPrev += 0.2;
  	xminLidarCurr += 0.2;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
		if ((abs(it->y) > laneWidth / 2.0) || (it->x < xminLidarPrev) )
        	continue;
        minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
		if ((abs(it->y) > laneWidth / 2.0) || (it->x < xminLidarCurr) )
        	continue;      
        minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
  	double dT {1/frameRate};
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, 
                        DataFrame &prevFrame, DataFrame &currFrame, const int threshMatch)
{
  	// query = prev, train = curr
    // define a 2D array with size #currBoxes x # prevBoxes
  	std::vector < std::vector <int> > boxAdjMat (currFrame.boundingBoxes.size(), std::vector<int>(prevFrame.boundingBoxes.size(), 0) );
  
  	for (const auto & match : matches) {
    	int prevIdx { match.queryIdx }, currIdx { match.trainIdx };
      	// iterate through all bounding boxes for prev frame
      	for (const auto & prevBox : prevFrame.boundingBoxes) {
          	// check if the prev point is in the prev box
         	if ( prevBox.roi.contains( prevFrame.keypoints[prevIdx].pt ) ) {
            	// check which curr Box contains the curr point
              	for (const auto & currBox : currFrame.boundingBoxes) {
                  	if ( currBox.roi.contains( currFrame.keypoints[currIdx].pt) ) {
                      	++boxAdjMat[currBox.boxID][prevBox.boxID];
                      	//break; // assume no other current box contains this point for efficiency
                    }
                }
                
                //break; // assume no other prev box contains this point for efficiency
            }
        }
      
    }
      
 	// print the box adjacency matrix, just for debug
  
//   	for (int i{0}; i<boxAdjMat.size(); ++i) {
//      	for (int j{0}; j<boxAdjMat[0].size(); ++j) {
//         	std::cout << boxAdjMat[i][j] << " ";
//         }
//       	std::cout << std::endl;
//     } 

  
    // iterate over the boxes, 
	// assign the first max in case of tie
	for (int ci{0}; ci<boxAdjMat.size(); ++ci) {
        // get max id
        int maxVal {-1}, maxIdx{-1};
        for (int pi{0}; pi<boxAdjMat[0].size(); ++pi) {         	
            if ( (boxAdjMat[ci][pi]>threshMatch) && (maxVal < boxAdjMat[ci][pi]) ) {
                maxVal = boxAdjMat[ci][pi];
                maxIdx = pi;
            }
        }
		
      	
      	if (maxIdx != -1) {

            // don't assign a box twice
//             for (int tmpCi{ci+1}; tmpCi<boxAdjMat.size(); ++tmpCi) {
//                 boxAdjMat[tmpCi][maxIdx] = -1;
//             }  

        	bbBestMatches[ci] = maxIdx;
        }
        
    } 
    
  	#ifdef PRINT_STATS
  		std::cout << "match size = " << bbBestMatches.size() << std::endl;
  	#endif
}


/*
float computeIoU(cv::Rect & box1, cv::Rect & box2)
{
  	float iou;
	float xmin { std::max(box1.x, box2.x) };
  	float xmax { std::min(box1.x + box1.width, box2.x + box2.width) };
	float ymin { std::max(box1.y, box2.y) };
  	float ymax { std::min(box1.y + box1.height, box2.y + box2.height) };
  
  	float interArea { std::max(0.0f, xmax - xmin +1) * std::max(0.0f, ymax-ymin+1) };
  	return interArea / ( box1.area() + box2.area() - interArea );
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, 
                        DataFrame &prevFrame, DataFrame &currFrame, const int threshMatch)
{
  	// query = prev, train = curr
    // define a 2D array with size #currBoxes x # prevBoxes
  	std::vector < std::vector <float> > boxAdjMat (currFrame.boundingBoxes.size(), std::vector<float>(prevFrame.boundingBoxes.size(), 0) );
  
    // iterate through all bounding boxes for prev frame
    for (auto & prevBox : prevFrame.boundingBoxes) {
        // get iou with each of the current boxes
        for (auto & currBox : currFrame.boundingBoxes) {
        	boxAdjMat[currBox.boxID][prevBox.boxID] = computeIoU(prevBox.roi, currBox.roi);
        }
    }
       
    // iterate over the boxes, 
	// assign the first max in case of tie
	for (int ci{0}; ci<boxAdjMat.size(); ++ci) {
        // get max id
        float maxVal {-1.0f}, maxIdx{-1};
        for (int pi{0}; pi<boxAdjMat[0].size(); ++pi) {         	
            if ( maxVal < boxAdjMat[ci][pi] ) {
                maxVal = boxAdjMat[ci][pi];
                maxIdx = pi;
            }
        }
      
      	bbBestMatches[ci] = maxIdx;
        
    } 
    

}
*/