/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// get number of points in the cloud
	int numPoints{cloud->width}; 
	if (cloud->height>1) { // organized
		numPoints *= (cloud->height);
	}

	float Aopt{0}, Bopt{0}, Copt{0};
	int maxInliers{0};
	// For max iterations
	for (int i{0}; i<maxIterations; ++i) {
		// Randomly sample subset and fit line
		int idx1 {rand() % numPoints};
		int idx2 {rand() % numPoints};
		while (idx2 == idx1) {
			idx2 = rand() % numPoints;
		}

		// get line coefficients, Z here is set to 0 for ease
		// Ax + By + C = 0
		// (y1-y2)x + (x2-x1)y + (x1*y2 - x2*y1) = 0
		// A = y1-y2
		// B = x2-x1
		// C = x1*y2 - x2*y1
		float x1{cloud->points[idx1].x};
		float y1{cloud->points[idx1].y};
		float x2{cloud->points[idx2].x};
		float y2{cloud->points[idx2].y};

		float A{y1-y2}, B{x2-x1}, C{x1*y2 - x2*y1};
		float ABsqrt{ sqrt(A*A + B*B) };		

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier		
		int countInliers{0};
		for (int pidx{0}; pidx < numPoints; ++pidx) {
			float x{cloud->points[pidx].x};
			float y{cloud->points[pidx].y};
			float z{cloud->points[pidx].z};			

			float dist { fabs(A*x + B*y + C) / ABsqrt};
			if (dist < distanceTol) {
				++countInliers;
			}
		}
		
		if (countInliers > maxInliers) {
			maxInliers = countInliers;
			Aopt = A;
			Bopt = B;
			Copt = C;
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	inliersResult.clear();
	float A{Aopt}, B{Bopt}, C{Copt};
	float ABsqrt{ sqrt(A*A + B*B) };
	for (int pidx{0}; pidx < numPoints; ++pidx) {
		float x{cloud->points[pidx].x};
		float y{cloud->points[pidx].y};
		float z{cloud->points[pidx].z};			

		float dist { fabs(A*x + B*y + C) / ABsqrt};
		if (dist < distanceTol) {
			inliersResult.insert(pidx);
		}
	}

	return inliersResult;
}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
								 int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL) );
	
	int numReqPoints{3}; // number of points required for equations
	int numPoints {cloud->size()};
	
	for (int i{0}; i<maxIterations; ++i) {
		// Randomly sample subset of points (2 here) and fit the plane
		std::unordered_set<int> ptsIndices;
		while (ptsIndices.size() < numReqPoints ) {
			ptsIndices.insert( rand() % numPoints );
		}

		// get plane coefficients
		// Ax + By + Cz + D = 0
		// A = (y2-y1)(z3-z1) - (z2-z1)(y3-y1)
		// B = (z2-z1)(x3-x1) - (x2-x1)(z3-z1)
		// C = (x2-x1)(y3-y1) - (y2-y1)(x3-x1)
		// D = -(A*x1 + B*y1 + C*z1)
		auto cit {ptsIndices.cbegin() };
		float x1 {cloud->points[*cit].x}, y1 {cloud->points[*cit].y}, z1 {cloud->points[*cit].z};
		++cit;
		float x2 {cloud->points[*cit].x}, y2 {cloud->points[*cit].y}, z2 {cloud->points[*cit].z};
		++cit;
		float x3 {cloud->points[*cit].x}, y3 {cloud->points[*cit].y}, z3 {cloud->points[*cit].z};

		float A { (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1) };
		float B { (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1) };
		float C { (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1) };
		float D { -(A*x1 + B*y1 + C*z1) };
		float den { sqrt(A*A + B*B + C*C) };

		// Measure distance between every point and fitted plane
		// If distance < threshold, count it as inlier
		std::unordered_set<int> tmpInliers;
		for (int pidx{0}; pidx<numPoints; ++pidx) {
			float x {cloud->points[pidx].x};
			float y {cloud->points[pidx].y};
			float z {cloud->points[pidx].z};
			
			float dist { fabs(A*x + B*y + C*z + D) / den};
			if (dist < distanceTol) {
				tmpInliers.insert(pidx);
			}
		}

		if (tmpInliers.size() > inliersResult.size()) {
			// move assignment, transfers ownership
			inliersResult.insert(tmpInliers.begin(), tmpInliers.end() );
		}

	}

	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); // given
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); // create 3D data
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
