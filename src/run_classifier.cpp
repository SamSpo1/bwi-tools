/* run a cascade classifier on video feed, display detections.
 * 
 */

#include <ros/ros.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>

using namespace std;

cv::CascadeClassifier cascade;

void image_callback (const sensor_msgs::Image::ConstPtr& img) {
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);	
	vector<cv::Rect> objects;
  cv::Mat image_gray;
  cv::cvtColor(cv_ptr->image,image_gray,CV_BGR2GRAY);
  cascade.detectMultiScale(image_gray,objects, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE );
  
  for (size_t j=0;j<objects.size();j++) {
    cv::rectangle( cv_ptr->image, objects[j], cv::Scalar(0,0,255), 2, 8, 0 );
  }
  cv::imshow("Cascade Classifier Window", cv_ptr->image);

}
      


void sig_handler(int sig) {
    printf("\n");
    exit(1);
}

int main ( int argc, char** argv ) {
  if (argc != 2) { cout << "Usage: ./run_classifier cascade_file" << endl; return -1; }
  
  // load cascade and image
  
  if (!cascade.load(argv[1])) { cout << "Error loading cascade file" << endl; return -1; }
  
  ros::init(argc,argv,"run_classifier");
  ros::NodeHandle n;
    
  ros::Subscriber image_sub = n.subscribe("/nav_kinect/rgb/image_raw",1,image_callback);
  
  signal(SIGINT,sig_handler);
  
  ros::Rate r(1);
  while (ros::ok()) {
	  ros::spinOnce();
	  cv::waitKey(100);
	  //cv::destroyWindow("Cascade Classifier Window");
  }
  return 0;
}
