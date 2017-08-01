/* save one image every FRAME_RATE images published
 * to /nav_kinect/rgb/image_raw. Also save amcl_pose, odom,
 * and cmd_vel info.
 */

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <signal.h>

// image tools
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// message types
//#include <sensor_msgs/CompressedImage.h>
#include <bwi_kr_execution/ExecutePlanActionResult.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// record one frame per FRAME_RATE frames (so really it's more like 1/[frame rate])
#define FRAME_RATE 1

std::string DEST;
std::ofstream images_file;
std::ofstream odom_file;
std::ofstream amcl_pose_file;
std::ofstream cmd_vel_file;

static int image_count = 0;


std::string int_to_str(int n) {
    std::ostringstream convert;
    convert << n;
    std::string out = convert.str();
    return out;
}

void image_callback (const sensor_msgs::Image::ConstPtr& img) {
    if (image_count==0) {
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		std::string write_to = "s"+int_to_str(img->header.stamp.sec)+"_ns"+int_to_str(img->header.stamp.nsec)+".jpg";
		images_file << int_to_str(img->header.stamp.sec)+"."+int_to_str(img->header.stamp.nsec)+","+"\""+write_to+"\""+"\n";
		cv::imwrite(DEST+write_to, cv_ptr->image);
    }
    image_count = (image_count+1)%FRAME_RATE;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    cmd_vel_file << cmd_vel->linear.x << "," << cmd_vel->linear.y << "," << cmd_vel->linear.z << "," << cmd_vel->angular.x << "," << cmd_vel->angular.y << "," << cmd_vel->angular.z << "\n";
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    odom_file << odom->header.stamp.sec << "." << odom->header.stamp.nsec << "," << odom->pose.pose.position.x << "," << odom->pose.pose.position.y << "," << odom->pose.pose.position.z << "," << odom->twist.twist.linear.x << "," << odom->twist.twist.linear.y << "," << odom->twist.twist.linear.z << "," << odom->twist.twist.angular.x << "," << odom->twist.twist.angular.y << "," << odom->twist.twist.angular.z << "\n";
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose) {
    amcl_pose_file << amcl_pose->header.stamp.sec << "." << amcl_pose->header.stamp.nsec << "," << amcl_pose->pose.pose.position.x << "," << amcl_pose->pose.pose.position.y << "," << amcl_pose->pose.pose.position.z << "," << amcl_pose->pose.pose.orientation.x << "," << amcl_pose->pose.pose.orientation.y << "," << amcl_pose->pose.pose.orientation.z << "," << amcl_pose->pose.pose.orientation.w << "\n";
}





void sig_handler(int sig) {
	images_file.close();
    odom_file.close();
    amcl_pose_file.close();
    cmd_vel_file.close();
    printf("\n");
    exit(1);
}


int main (int argc, char** argv) {
    // get destination from user
    if (argc != 2) { printf("Usage: rosrun sams_bwitools simple_recorder DEST\n"); return -1; }
    std::string ALL_DEST = argv[1];
    if (ALL_DEST.at(ALL_DEST.length()-1) != '/') { DEST = ALL_DEST+'/'; }
    else { DEST = ALL_DEST; }
    system(("mkdir "+DEST).c_str());
    
    ros::init(argc,argv,"simple_recorder");
    ros::NodeHandle n;
    
    ros::Subscriber image_sub = n.subscribe("/nav_kinect/rgb/image_raw",1,image_callback);
    
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel",1,cmd_vel_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom",1,odom_callback);
    ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose",1,amcl_pose_callback);
    
    // set up files for writing
    std::string images_dest = DEST;
    images_dest.append("images.txt");
    std::string odom_dest = DEST;
    odom_dest.append("odom.txt");
    std::string amcl_pose_dest = DEST;
    amcl_pose_dest.append("amcl_pose.txt");
    std::string cmd_vel_dest = DEST;
    cmd_vel_dest.append("cmd_vel.txt");
    
    images_file.open(images_dest.c_str());
    odom_file.open(odom_dest.c_str());
    amcl_pose_file.open(amcl_pose_dest.c_str());
    cmd_vel_file.open(cmd_vel_dest.c_str());
    
    ros::Rate r(1);
    
    signal(SIGINT,sig_handler);
    
    while (ros::ok()){
        //collect messages
        ros::spinOnce();
        
        //sleep to maintain framerate
        r.sleep();
    }
    
    return 0;
}
