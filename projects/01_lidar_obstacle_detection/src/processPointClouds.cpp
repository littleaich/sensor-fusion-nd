// PCL lib Functions for processing point clouds 

#include <iostream>

#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


// template<typename PointT>
// void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
// {
//     std::cout << cloud->points.size() << std::endl;
// }

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return cloud->points.size();
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint )
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // source: http://pointclouds.org/documentation/tutorials/voxel_grid.php
    // create the filtering object
    typename pcl::VoxelGrid<PointT> * sor {new pcl::VoxelGrid<PointT> };
    sor->setInputCloud(cloud);
    sor->setLeafSize(filterRes, filterRes, filterRes); // 20 cm voxel, even 5cm is found to be too small for the given dataset
    sor->filter(*cloud);

    // source: http://docs.pointclouds.org/trunk/classpcl_1_1_crop_box.html#aaeb54003f0887acab286243ff7a9a0d2
    typename pcl::CropBox<PointT>::Ptr boxCropper { new pcl::CropBox<PointT> };
    boxCropper->setInputCloud(cloud);
    boxCropper->setMin( minPoint );
    boxCropper->setMax( maxPoint );
    boxCropper->filter( *cloud );

    typename pcl::CropBox<PointT>::Ptr egoCar {new pcl::CropBox<PointT> (true) };
    std::vector <int> egoIndices;
    egoCar->setInputCloud(cloud);
    // egoCar->setMin ( Eigen::Vector4f {-2.7, -2.7, -5, 1} );
    // egoCar->setMax ( Eigen::Vector4f {2.7, 2.7, 1, 1} );
    egoCar->setMin ( Eigen::Vector4f {-1.5, -1.5, -5, 1} );
    egoCar->setMax ( Eigen::Vector4f {1.5, 1.5, 1, 1} );

    egoCar->filter(egoIndices);


    pcl::PointIndices::Ptr inliers { new pcl::PointIndices };
    for (int point: egoIndices) {
        inliers->indices.push_back(point);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // inliers are ego indices, remove them
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, 
                                        typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles {new pcl::PointCloud<PointT> };
    typename pcl::PointCloud<PointT>::Ptr cloud_road {new pcl::PointCloud<PointT> };

    // create the extraction object
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // get inliers
    extract.filter(*cloud_road); // road points are inliers for plane

    extract.setNegative(true); // negative of inliers
    extract.filter(*cloud_obstacles);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    segResult(cloud_obstacles, cloud_road);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices() };
    // TODO:: Fill in this function to find inliers for the cloud.
    // define model coefficients
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients() };

    // create segmentation object
    typename pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true); // optional

    // mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // here we assume a single plane, so only extract one. 
    // in PCL documentation, it iteratively extracts all plane segments
    // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0 ) {
        std::cerr << "No points in the cloud for segmentation." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::Clustering
(typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // create kd-tree object for search 
    typename pcl::search::KdTree<PointT>::Ptr tree 
        { new typename pcl::search::KdTree<PointT> };
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract (clusterIndices);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Note that we cannot easily use pcl::ExtractIndices<> object here,
    // because ExtractIndices object needs pcl::PointIndices::Ptr object
    // whereas euclidean clustering works with pcl::PointIndices directly.
    // there might be some easy workaround for this, I just didn't dig deeper.
    for (std::vector<pcl::PointIndices>::const_iterator cit=clusterIndices.begin();
            cit != clusterIndices.end(); ++cit) {

        typename pcl::PointCloud<PointT>::Ptr cloudClusterSingle { new pcl::PointCloud<PointT> };
        for (std::vector<int>::const_iterator pit=cit->indices.begin(); 
                pit != cit->indices.end(); ++pit) {
            cloudClusterSingle->points.push_back(cloud->points[*pit]);
        }

        // make it unorganized (for no reasons for now)
        cloudClusterSingle->width = cloudClusterSingle->points.size();
        cloudClusterSingle->height = 1;
        cloudClusterSingle->is_dense = true;

        clusters.push_back(cloudClusterSingle);
    }        

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}