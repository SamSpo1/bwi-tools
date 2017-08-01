// take photos when you press the space bar, save to DEST

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

std::string DEST;

static int image_count = 0;


std::string int_to_str(int n) {
    std::ostringstream convert;
    convert << n;
    std::string out = convert.str();
    return out;
}

void image_callback (const sensor_msgs::Image::ConstPtr& img) {
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	if (image_count > 0) {
		cv::destroyWindow("Photograph");
		std::string write_to = "s"+int_to_str(img->header.stamp.sec)+"_ns"+int_to_str(img->header.stamp.nsec)+".jpg";
		cv::imwrite(DEST+write_to, cv_ptr->image);
	}
	
	cv::imshow("Photograph",cv_ptr->image);
	image_count++;
}


void sig_handler(int sig) {
    printf("\n");
    exit(1);
}


int main (int argc, char** argv) {
    // get destination from user
    if (argc != 2) { printf("Usage: rosrun sams_bwitools photograph DEST\n"); return -1; }
    std::string ALL_DEST = argv[1];
    if (ALL_DEST.at(ALL_DEST.length()-1) != '/') { DEST = ALL_DEST+'/'; }
    else { DEST = ALL_DEST; }

	system(("mkdir "+DEST).c_str());
    
    ros::init(argc,argv,"photograph");
    ros::NodeHandle n;
    
    ros::Subscriber image_sub = n.subscribe("/nav_kinect/rgb/image_raw",1,image_callback);
    
    
    
    signal(SIGINT,sig_handler);
    
    ros::Rate r(1);
    r.sleep();
    ros::spinOnce();
    int key;
    while (ros::ok()){
		key = cv::waitKey(0);
		if (32==key) {
			ros::spinOnce();
		}
		else if (27==key) {
			return 0;
		}
		
    }
    
    return 0;
}
