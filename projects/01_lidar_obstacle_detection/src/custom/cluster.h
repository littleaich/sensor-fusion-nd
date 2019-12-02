#ifndef CLUSTER_H
#define CLUSTER_H

#include "kdtree.h"

template<typename PointT>
void proximity(const int id, typename pcl::PointCloud<PointT>::Ptr cloud,
				const float distanceTol, std::vector<bool> & processed, 
				std::vector<int> & cluster, KdTree<PointT> * tree)
{
	/* Pseudocode:
	Proximity(point, cluster)
		MARK point as processed
		ADD point to cluster
		GET nearby points to point with tree search 
		FOR EACH point in nearby points
			IF point has not been processed
				Proximity(point, cluster)
	*/

	processed[id] = true;
	cluster.push_back(id);
	std::vector<int> idsNeighborPoints ( tree->search(cloud->points[id], distanceTol) );	
	for (const auto & id : idsNeighborPoints) {
		if (processed[id]) continue;
		proximity<PointT>(id, cloud, distanceTol, processed, cluster, tree);
	}
}


template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	KdTree<PointT> * tree, const float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	/* Pseudocode:
	INIT list of clusters
	FOR EACH point
		IF point has not been processed
			CREATE cluster
			Proximity(point, cluster)
			ADD cluster to clusters
	*/


	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed (cloud->points.size(), false);
	for (int i{0}; i<cloud->points.size(); ++i) {
		if (processed[i]) continue;
		
		std::vector<int> cluster;
		proximity<PointT>(i, cloud, distanceTol, processed, cluster, tree);
		clusters.push_back(cluster);
	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr > 
clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
			float clusterTolerance, 
			int minSize, int maxSize)
{

	// create kd-tree and insert points
	KdTree<PointT> * tree = new KdTree<PointT>;
	for (int i{0}; i<cloud->points.size(); ++i) {
		tree->insert(cloud->points[i], i);
	}

	std::vector<std::vector<int> > clusters
		= euclideanCluster<PointT>(cloud, tree, clusterTolerance);
	// std::cout << "Number of clusters " << clusters.size() << std::endl;		

	std::vector<typename pcl::PointCloud<PointT>::Ptr > cloudClusters;
	for (std::vector<int> cluster : clusters) {
		if ( (cluster.size() < minSize) || (cluster.size() > maxSize) ) {
			continue;
		}
		typename pcl::PointCloud<PointT>::Ptr cloudSingle
			{ new pcl::PointCloud<PointT> };
		for (auto i : cluster) {
			cloudSingle->points.push_back(cloud->points[i]);
		}

		cloudClusters.push_back(cloudSingle);
	}	

	return cloudClusters;

}


#endif