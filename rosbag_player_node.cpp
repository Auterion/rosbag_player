/*
 * rosbag_player.cpp
 *
 *  Created on: Aug 2, 2019
 *      Author: tanja
 *
 *      launch example:
 *      roslaunch rosbag_player rosbag_player.launch "bag_name":="/path/to/file/bag_name"
 */


#include "rosbag_player_node.h"


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rosbag_player_node");

	ros::NodeHandle nh("~");
	RosbagPlayerNode player(nh);

	ros::spin();

    return 0;
}

RosbagPlayerNode::~RosbagPlayerNode() {
}

RosbagPlayerNode::RosbagPlayerNode(ros::NodeHandle& nh) {
   nh_ = nh;

   fov_sub_ = nh.subscribe("/fov", 1, &RosbagPlayerNode::fovCallback, this);
}

void RosbagPlayerNode::fovCallback(const visualization_msgs::Marker& msg) {

   //calculate localtion for display from fov
   Eigen::Vector3f middle;
   middle.x() = (msg.points[1].x + msg.points[2].x + msg.points[5].x + msg.points[8].x) / 4.f;
   middle.y() = (msg.points[1].y + msg.points[2].y + msg.points[5].y + msg.points[8].y) / 4.f;
   middle.z() = (msg.points[1].z + msg.points[2].z + msg.points[5].z + msg.points[8].z) / 4.f;

   if(middle.norm() > 0){
	   Eigen::Vector3f middle_normalized = middle.normalized();
	   Eigen::Vector3f middle_xy = {middle_normalized.x(), middle_normalized.y(), 0};
	   Eigen::Vector3f middle_xy_normalized = middle_xy.normalized();
	   float yaw = atan2(middle_xy_normalized.y(), middle_xy_normalized.x());
	   float pitch = acos(middle_xy.norm());

	   std::string frame_name = "unknown_frame";
	   if(msg.id == 0){
		   frame_name = "camera_front_display";
	   }else if(msg.id == 1){
		   frame_name = "camera_left_display";
	   }else if(msg.id == 2){
		   frame_name = "camera_right_display";
	   }

	   //TODO: account for cameras mounted upside down

	   //publish display transforms
	   static tf::TransformBroadcaster br;
	   tf::Transform transform;
	   transform.setOrigin(tf::Vector3(middle.x(), middle.y(), middle.z()));
	   tf::Quaternion q;
	   q.setRPY(1.571 - pitch, 0, 1.571 + yaw);
	   transform.setRotation(q);
	   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", frame_name));

   }else{
	   ROS_WARN("invalid FOV received");
   }


}

