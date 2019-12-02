/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

#include <math.h>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// with pointer reference
	void insertHelper(Node *& root, Node *& nodeNew, int depth)
	{
		if (root==NULL) {
			root = nodeNew;
			return;
		}else {
			int dim = depth % 2; // 2 is #dimensions (x,y)
			if (nodeNew->point[dim] < root->point[dim]) {
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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node * nodePtr { new Node(point, id) };
		insertHelper(root, nodePtr, 0); // for pointer reference
		// insertHelper(&root, nodePtr, 0); // for double pointer
	}


	void searchHelper(Node *& root, int depth, const std::vector<float> & target, 
						const float distanceTol, std::vector<int> & ids) 
	{
		if (root == NULL) {
			return;
		}

		int dim = depth % 2; // 2 is #dimensions (x,y)
		// check if root is within the box
		if ( (root->point[0] > target[0]-distanceTol) && (root->point[0] < target[0]+distanceTol)
			&& (root->point[1] > target[1]-distanceTol) && (root->point[1] < target[1]+distanceTol) ) {

			// within the box, but corners might be outside, so check
			float dist { sqrt( pow(root->point[0]-target[0], 2) + pow(root->point[1]-target[1], 2) ) };
			if (dist <= distanceTol) {
				ids.push_back(root->id);
			}

		} 

		// check if the box crosses the boundary for the selected dimension
		if (target[dim]-distanceTol < root->point[dim]) {
			searchHelper(root->left, depth+1, target, distanceTol, ids);
		}

		if (target[dim]+distanceTol > root->point[dim]) {
			searchHelper(root->right, depth+1, target, distanceTol, ids);
		}

	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) 
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;

	}
	

};




