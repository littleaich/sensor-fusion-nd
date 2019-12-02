#ifndef RENDER_IMPL_H
#define RENDER_IMPL_H

#include "box.h"

template<typename PointT>
void renderPointCloudClusters(
	pcl::visualization::PCLVisualizer::Ptr & viewer,
	const std::vector<typename pcl::PointCloud<PointT >::Ptr> & cloudClusters,
	const std::vector<Color> & colors)
{

	int clusterIdx {0};

	for (typename pcl::PointCloud<PointT>::Ptr cluster : cloudClusters) {
	// for (auto cluster : cloudClusters) {
		std::cout << "Cluster " << clusterIdx << " size: " << cluster->points.size() << std::endl;
		renderPointCloud(viewer, cluster, "obstacle" + std::to_string(clusterIdx), colors[clusterIdx % colors.size()]);
		++clusterIdx;
	}


}


template<typename PointT>
void renderBoundingBoxes(
	pcl::visualization::PCLVisualizer::Ptr & viewer,
	ProcessPointClouds<PointT> * pointProcessor, 
	const std::vector<typename pcl::PointCloud<PointT >::Ptr> & cloudClusters,
	const std::vector<Color> & colors)
{

	int clusterIdx {0};

	for (typename pcl::PointCloud<PointT>::Ptr cluster : cloudClusters) {
	// for (auto cluster : cloudClusters) {
		Box box { pointProcessor->BoundingBox(cluster) };
		renderBox(viewer, box, clusterIdx, colors[clusterIdx % colors.size()]);
		++clusterIdx;
	}


}

#endif
