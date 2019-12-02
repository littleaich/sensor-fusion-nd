#ifndef RANSAC_H
#define RANSAC_H

#include <pcl/common/common.h>
#include <unordered_set>

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, 
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

#endif
