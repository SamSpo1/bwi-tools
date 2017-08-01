/* convert information from rosbag that was collected by subscribing to
 * /odom, /amcl_pose, /cmd_vel, and /nav_kinect/rgb/image_raw into
 * a more readable format.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

/* 
 * FORMAT OF OUTPUT:
 * 
 * image_headers.txt
 * seconds.nanoseconds,"image_location"
 * 
 * amcl_pose.txt
 * seconds.nanoseconds,position.x,y,z,orientation.x,y,z,w
 * 
 * odom.txt
 * seconds.nanoseconds,position.x,y,z,linear.x,y,z,angular.x,y,z
 * 
 * cmd_vel.txt
 * linear.x,y,z,angular.x,y,z
 * 
 * For some reason, /odom says angular velocity is 0 even when turning.
 * 
 * 
 */

static const std::string IMAGE_DEST = "camera_images/";


std::string int_to_str(int n) {
	std::ostringstream convert;
	convert << n;
	std::string out = convert.str();
	return out;
}

std::string double_to_str(double n) {
	std::ostringstream convert;
	convert << n;
	std::string out = convert.str();
	return out;
}

int main(int argc, char** argv)
{
	if (argc != 3) {
		printf("Usage: rosrun sams_bwitools bag_extractor SOURCE DEST\n");
		return -1;
	}
	static const std::string INPUT = argv[1];
	std::string ALL_DEST = argv[2];
	std::string ALL_DEST_N;
	
	if (ALL_DEST.at(ALL_DEST.length()-1) != '/') {
		ALL_DEST_N = ALL_DEST+'/';
	}
	else {
		ALL_DEST_N = ALL_DEST;
	}
	system(("mkdir "+ALL_DEST_N).c_str());
	system(("mkdir "+ALL_DEST_N+IMAGE_DEST).c_str());
	
	ros::init(argc, argv, "bag_extractor");
	
	std::vector<std::string> topics;
	topics.push_back("/nav_kinect/rgb/image_raw");
	topics.push_back("/odom");
	topics.push_back("/amcl_pose");
	topics.push_back("/cmd_vel");
	
	
	rosbag::Bag bag;
	bag.open(INPUT, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
		
	cv_bridge::CvImagePtr cv_ptr;
	
	std::ofstream image_file;
	std::ofstream odom_file;
	std::ofstream amcl_pose_file;
	std::ofstream cmd_vel_file;
	
	std::string image_file_dest = ALL_DEST_N;
	image_file_dest.append("image_headers.txt");
	
	std::string odom_dest = ALL_DEST_N;
	odom_dest.append("odom.txt");
	
	std::string amcl_pose_dest = ALL_DEST_N;
	amcl_pose_dest.append("amcl_pose.txt");
	
	std::string cmd_vel_dest = ALL_DEST_N;
	cmd_vel_dest.append("cmd_vel.txt");
	
	image_file.open(image_file_dest.c_str());
	odom_file.open(odom_dest.c_str());
	amcl_pose_file.open(amcl_pose_dest.c_str());
	cmd_vel_file.open(cmd_vel_dest.c_str());
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		
		sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
		nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
		geometry_msgs::PoseWithCovarianceStamped::ConstPtr amcl_pose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
		geometry_msgs::Twist::ConstPtr cmd_vel = m.instantiate<geometry_msgs::Twist>();
		
		if (img != NULL){
			
			cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
			
			std::string write_to = "\"s"+int_to_str(img->header.stamp.sec)+"_ns"+int_to_str(img->header.stamp.nsec)+".jpg\"";
			
			image_file << int_to_str(img->header.stamp.sec)+"."+int_to_str(img->header.stamp.nsec)+","+write_to+"\n";
			cv::imwrite(ALL_DEST_N+IMAGE_DEST+write_to, cv_ptr->image);
		}
		
		if (odom != NULL) {
			odom_file << odom->header.stamp.sec << "." << odom->header.stamp.nsec << "," << odom->pose.pose.position.x << "," << odom->pose.pose.position.y << "," << odom->pose.pose.position.z << "," << odom->twist.twist.linear.x << "," << odom->twist.twist.linear.y << "," << odom->twist.twist.linear.z << "," << odom->twist.twist.angular.x << "," << odom->twist.twist.angular.y << "," << odom->twist.twist.angular.z << "\n";
		}
		
		if (amcl_pose != NULL) {
			amcl_pose_file << amcl_pose->header.stamp.sec << "." << amcl_pose->header.stamp.nsec << "," << amcl_pose->pose.pose.position.x << "," << amcl_pose->pose.pose.position.y << "," << amcl_pose->pose.pose.position.z << "," << amcl_pose->pose.pose.orientation.x << "," << amcl_pose->pose.pose.orientation.y << "," << amcl_pose->pose.pose.orientation.z << "," << amcl_pose->pose.pose.orientation.w << "\n";
		}
		
		if (cmd_vel != NULL) {
			cmd_vel_file << cmd_vel->linear.x << "," << cmd_vel->linear.y << "," << cmd_vel->linear.z << "," << cmd_vel->angular.x << "," << cmd_vel->angular.y << "," << cmd_vel->angular.z << "\n";
		}
		
	}
	
	image_file.close();
	odom_file.close();
	amcl_pose_file.close();
	cmd_vel_file.close();
	
	bag.close();
	
	return 0;
}
