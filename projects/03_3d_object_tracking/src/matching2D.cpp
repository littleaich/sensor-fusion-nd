#include <numeric>
#include "matching2D.hpp"
#include <boost/algorithm/string.hpp>
#include <stdexcept> // for invalid argument

using namespace std;


// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descQuery, cv::Mat &descTrain,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = true;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {

        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
      	#ifdef PRINT_EXE_TIME
        	cout << "BF matching cross-check=" << crossCheck;
      	#endif
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descQuery.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descQuery.convertTo(descQuery, CV_32F);
            descTrain.convertTo(descTrain, CV_32F);
        }

        //... TODO : implement FLANN matching
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      	#ifdef PRINT_EXE_TIME
	        cout << "FLANN matching";
      	#endif
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descQuery, descTrain, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      	#ifdef PRINT_EXE_TIME
	        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
      	#endif
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // TODO : implement k-nearest-neighbor matching
        int k{2};
        std::vector< std::vector<cv::DMatch> > knn_matches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descQuery, descTrain, knn_matches, k); // Find k best matches
        t = ( (double)cv::getTickCount() - t) / cv::getTickFrequency();
      	#ifdef PRINT_EXE_TIME
        	cout << " (kNN) with k= " << k << "and  n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
      	#endif

        // TODO : filter matches using descriptor distance ratio test
        // source : https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        const float ratio_thresh {0.8f};
        for (int i{0}; i<knn_matches.size(); ++i ) {
            if ( knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance ) {
                matches.push_back( knn_matches[i][0] );
            }
        }
    }

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void extractDescriptors(std::string descriptorType, vector<cv::KeyPoint> &keypoints, 
                        cv::Mat &img, cv::Mat &descriptors, float &exeTime)
{
    // descriptor source:
    // https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html#BRISK%20:%20public%20Feature2D
    boost::to_upper(descriptorType);
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0) {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF")==0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB")==0) {
        extractor = cv::ORB::create();
    } 
    else if (descriptorType.compare("FREAK")==0) {
        extractor = cv::xfeatures2d::FREAK::create();
    }  
    else if (descriptorType.compare("AKAZE")==0) {
        extractor = cv::AKAZE::create();
    } else if (descriptorType.compare("SIFT")==0) {
        // source: https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        extractor = cv::xfeatures2d::SIFT::create();
    } else {
        throw std::invalid_argument ("Invalid argument (" + descriptorType + ") for descriptor type.");
    }



    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  	#ifdef PRINT_EXE_TIME
    	cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
  	#endif
  	
  	exeTime = t;
}

void detectKeyPoints(std::string detectorType, vector<cv::KeyPoint> &keypoints, cv::Mat &img, float & exeTime, bool bVis)
{
    boost::to_upper(detectorType);
  
    double t = (double)cv::getTickCount();
    if (detectorType.compare("SHI-TOMASI") == 0) {
        detectKeyPointsShiTomasi(keypoints, img);
    } else if ( detectorType.compare("HARRIS") == 0){
        detectKeyPointsHarris(keypoints, img);
    } else if ( detectorType.compare("FAST") == 0) {
        detectKeyPointsFAST(keypoints, img);
    } else if ( detectorType.compare("BRISK") == 0) {
        detectKeyPointsBRISK(keypoints, img);        
    } else if ( detectorType.compare("ORB") == 0) {
        detectKeyPointsORB(keypoints, img); 
    } else if ( detectorType.compare("AKAZE") == 0) {
        detectKeyPointsAKAZE(keypoints, img);                        
    } else if ( detectorType.compare("SIFT") == 0) {
        detectKeyPointsSIFT(keypoints, img);                                
    }else {
        throw std::invalid_argument ("Invalid argument (" + detectorType + ") for detector type.");
    }

    t = ( (double)cv::getTickCount() - t) / cv::getTickFrequency();
  	#ifdef PRINT_EXE_TIME
    	std::cout << detectorType + " detection with " << keypoints.size();
    	std::cout << " keypoints in " << 1000.0 * t << "ms" << std::endl;
  	#endif

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName =  detectorType + " Corner Detector Results";
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

	exeTime = t;
}


void detectKeyPointsShiTomasi(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{    
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

}


void detectKeyPointsHarris(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    int blockSize {2}; // for every pixel, a blockSize x blockSize neighborhood is considered
    int apertureSize {3}; // aperture parameter for Sobel operator (must be odd)
    int minResponse {100}; // cornerness threshold
    double k {0.04}; // Harris parameter

    // source: https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html
    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris( img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );

    for (int i{0}; i<dst_norm.rows; ++i) {
        for (int j{0}; j<dst_norm.cols; ++j) {
            if ( (int)dst_norm.at<float>(i,j) > minResponse ) {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f( j, i );
                newKeyPoint.size = blockSize;
                keypoints.push_back( newKeyPoint );
            }
        }
    }

}


void detectKeyPointsFAST(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    int thresh {10};
    bool nonMaxSup {true};
    cv::FAST(img, keypoints, thresh, nonMaxSup);
}


void detectKeyPointsBRISK(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    // use default params for now
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
    detector->detect(img, keypoints);
}


void detectKeyPointsORB(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    // use default params for now
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    detector->detect(img, keypoints);    
}

void detectKeyPointsAKAZE(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    // use default params for now
    cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
    detector->detect(img, keypoints);    
}

void detectKeyPointsSIFT(vector<cv::KeyPoint> & keypoints, cv::Mat & img)
{
    // use default params for now
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
    detector->detect(img, keypoints);    
}