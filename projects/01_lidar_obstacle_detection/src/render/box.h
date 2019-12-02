#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 


// can be used for drawing the bounding box in any direction.
// check out this blog post that uses PCA to do that.
// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};
#endif