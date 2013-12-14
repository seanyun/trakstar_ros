/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include <string>
#include <ros/ros.h>
#include "trakstar/PointATC3DG.hpp"
#include "trakstar/TrakstarMsg.h"
#include "tf/tf.h"

// Visualization
#include <tf/transform_broadcaster.h>

using namespace trakstar;
using std::string;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trakstar_driver");
	ros::NodeHandle n, n_private("~");

	//initialize hardware
	PointATC3DG bird_;
	if( !bird_ ) {
		ROS_ERROR("can't open trakstar"); 
		return -1;
	}
	bird_.setSuddenOutputChangeLock( 0 );	
	int num_sen=bird_.getNumberOfSensors();
	ROS_INFO("Number of trakers: %d", num_sen);

	if (num_sen<2) {
		ROS_ERROR("at least 2 trackers required"); 
		return -1;
	}

	ROS_INFO("Output is set: position/quaternion");
	for (int i=0; i<num_sen; i++)
		bird_.setSensorQuaternion(i);

	double dX, dY, dZ;
	double* quat=new double[4];


	bool publish_tf = false;
	n_private.param<bool>("publish_tf", publish_tf, false);
	if(publish_tf)
		ROS_INFO("Publishing frame data to TF.");

	// Initialize ROS stuff
	ros::Publisher trakstar_pub = n.advertise<trakstar::TrakstarMsg>("trakstar_msg", 1);
	tf::TransformBroadcaster *broadcaster = 0;
	if(publish_tf) 
		broadcaster = new tf::TransformBroadcaster();

	// mangle the reported pose into the ROS frame conventions
	const tf::Matrix3x3 ros_to_trakstar( -1,  0,  0,
		                   	      0,  1,  0,
		                   	      0,  0, -1 );

	while (n.ok())
	{
		//publish data
		trakstar::TrakstarMsg msg;
		msg.header.stamp = ros::Time::now();

		std::vector<geometry_msgs::TransformStamped> transforms(num_sen);

		for( int i = 0; i <num_sen  ; ++i ) 
		{
			bird_.getCoordinatesQuaternion(i, dX, dY, dZ, quat);
			tf::Vector3 pos(dX, dY, dZ);
			pos=ros_to_trakstar*pos;
			tf::Quaternion q(-quat[1], -quat[2], -quat[3], quat[0]);
			tf::Matrix3x3 mat(q);
			mat=ros_to_trakstar*mat;

			tf::transformTFToMsg(tf::Transform(mat,pos), transforms[i].transform);
			msg.transform[i]=transforms[i].transform;
		}
		trakstar_pub.publish(msg);

		if(broadcaster)
		{
			std::string frames[4] = {"trakstar_left", "trakstar_right", "trakstar_third", "trakstar_fourth"};
			for(int kk = 0; kk < num_sen; kk++)
			{
				transforms[kk].header.stamp = msg.header.stamp;
				transforms[kk].header.frame_id = "trakstar_base";
				transforms[kk].child_frame_id = frames[kk];
			}

			broadcaster->sendTransform(transforms);
		}

		ros::spinOnce();
	}

	delete [] quat;
}









