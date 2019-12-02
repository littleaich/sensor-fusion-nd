#ifndef KD_TREE_H
#define KD_TREE_H

#include <math.h>
#include <pcl/common/common.h>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node<PointT> * root;
	int dim;

	KdTree(int dim=2)
	: root(NULL), dim(dim)
	{}

	// with pointer reference
	void insertHelper(Node<PointT> *& root, Node<PointT> *& nodeNew, int depth)
	{
		if (root==NULL) {
			root = nodeNew;
			return;
		}else {
			int dim = depth % this->dim; // 3 is #dimensions (x,y,z)
			if (nodeNew->point.data[dim] < root->point.data[dim]) {
				insertHelper(root->left, nodeNew, depth+1);
			} else {
				insertHelper(root->right, nodeNew, depth+1);
			}
		}

		return;

	}

	// // with double pointer
	// void insertHelper(Node ** root, Node * nodePtr, int depth) 
	// {
	// 	if (*root == NULL) {
	// 		*root = nodePtr;
	// 		return;
	// 	} else {
	// 		int dim = depth % 2; // 2 is #dimensions (x,y)
	// 		if ( nodePtr->point[dim] < (*root)->point[dim] ) {
	// 			insertHelper( &(*root)->left, nodePtr, depth+1 );
	// 		} else {
	// 			insertHelper( &(*root)->right, nodePtr, depth+1 );
	// 		}
	// 	}
	// }

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node<PointT> * nodePtr { new Node<PointT>(point, id) };
		insertHelper(root, nodePtr, 0); // for pointer reference
		// insertHelper(&root, nodePtr, 0); // for double pointer
	}


	void searchHelper(Node<PointT> *& root, int depth, const PointT & target, 
						const float distanceTol, std::vector<int> & ids) 
	{
		if (root == NULL) {
			return;
		}

		int dim = depth % this->dim; // 3 is #dimensions (x,y,z)
		// check if root is within the box
		if ( (root->point.data[0] > target.data[0]-distanceTol) && (root->point.data[0] < target.data[0]+distanceTol)
			&& (root->point.data[1] > target.data[1]-distanceTol) && (root->point.data[1] < target.data[1]+distanceTol) ) { 
			// && (root->point.data[2] > target.data[2]-distanceTol) && (root->point.data[2] < target.data[2]+distanceTol) ) {

			// within the box, but corners might be outside, so check
			float dist { sqrt( pow(root->point.data[0]-target.data[0], 2) 
								+ pow(root->point.data[1]-target.data[1], 2) ) }; 
								// + pow(root->point.data[2]-target.data[2], 2) ) };
			if (dist <= distanceTol) {
				ids.push_back(root->id);
			}

		} 

		// check if the box crosses the boundary for the selected dimension
		if (target.data[dim]-distanceTol < root->point.data[dim]) {
			searchHelper(root->left, depth+1, target, distanceTol, ids);
		}

		if (target.data[dim]+distanceTol > root->point.data[dim]) {
			searchHelper(root->right, depth+1, target, distanceTol, ids);
		}

	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol) 
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;

	}
	

};

#endif 




