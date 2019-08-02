#ifndef ROSBAG_PLAYER_NODE_H
#define ROSBAG_PLAYER_NODE_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <iostream>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

class RosbagPlayerNode {
 public:
	RosbagPlayerNode(ros::NodeHandle& nh);
  ~RosbagPlayerNode();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber fov_sub_;

  void fovCallback(const visualization_msgs::Marker& msg);

};

#endif  // ROSBAG_PLAYER_NODE_H

