
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <alignment.hpp>

// Constructor
Alignment::Alignment() : _done(false), _counter(0) {
  //this->_f = Features(Features::CV_ORB);

  // -----------------
  // ROS subscribers and sync
  // -----------------
  this->_obj_sub = 
    new message_filters::Subscriber<sensor_msgs::Image>( this->_nh, 
							 "/camera/rgb/image_rect_color", 
							 1 ); 
  this->_scene_sub = 
    new message_filters::Subscriber<sensor_msgs::Image>( this->_nh, 
							 "/camera_hd/image_raw", 
							 1 );
  this->_sync = 
    new message_filters::Synchronizer<ImagesSyncPolicy>( ImagesSyncPolicy(10), 
							 *this->_obj_sub, 
							 *this->_scene_sub);
  this->_sync->registerCallback( boost::bind( &Alignment::process, this, _1,_2) );
 
  // -----------------
  // Read ROS params
  // -----------------
  int rows = 0, cols = 0;
  if( !ros::param::get("~rows", rows) ) {
    ROS_ERROR("Could not read pattern size (number of rows).");
  }
  if( !ros::param::get("~cols", cols) ) {
    ROS_ERROR("Could not read pattern size (number of rows).");
  }
  this->_pattern_sz = cv::Size( rows, cols);
  ROS_INFO("Pattern: %dx%d", rows, cols);
}

// Clean up..
Alignment::~Alignment() {
  delete _sync;
  delete _obj_sub;
  delete _scene_sub;
}


// --------------------------------------------------------
// Recive and extract features from  two syncronized images
// ---------------------------------------------------------
void Alignment::process( const sensor_msgs::ImageConstPtr &obj_msg,
			 const sensor_msgs::ImageConstPtr &scene_msg ) {
  // OpenCV images and features
  cv::Mat img_obj, img_scene, gray;
  /*
  std::vector<cv::KeyPoint> keypoints_obj, keypoints_scene;
  cv::Mat descriptor_obj, descriptor_scene;
  std::vector<cv::DMatch> matches;
  */

  // Receive images
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( obj_msg, 
				  sensor_msgs::image_encodings::BGR8 );
    cv_ptr->image.copyTo(img_obj);
    cv_ptr = cv_bridge::toCvCopy( scene_msg, 
				  sensor_msgs::image_encodings::BGR8 );
    cv_ptr->image.copyTo(img_scene);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[alignment_node] receiving images: %s", e.what());
    return;
  }
  
  // Convert to gray and extract corners
  std::vector<cv::Point2f> corners_obj, corners_scene;
  cv::cvtColor( img_obj, gray, CV_BGR2GRAY);
  bool obj_ok = cv::findChessboardCorners( gray, this->_pattern_sz, corners_obj, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  cv::cvtColor( img_scene, gray, CV_BGR2GRAY);
  bool scene_ok = cv::findChessboardCorners( gray, this->_pattern_sz, corners_scene, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

  // Check the result
  if( obj_ok && scene_ok ) {
    for( uint i = 0; i < corners_obj.size(); i++) {
      this->_src.push_back(corners_obj[i]);
      this->_dst.push_back(corners_scene[i]);
    }
    this->_counter++;
    ROS_INFO("Collected %d (of 50) pairs.", this->_counter);    
    if( this->_counter >= 50 ) {
      this->_done = true;
    }
  }

  /*
  // Convert to gray scale and extract features
  cv::cvtColor( img_obj, gray, CV_BGR2GRAY);
  this->_f.detect( gray, keypoints_obj);
  descriptor_obj = this->_f.extract( gray, keypoints_obj);
  cv::cvtColor( img_scene, gray, CV_BGR2GRAY);
  this->_f.detect( gray, keypoints_scene);
  descriptor_scene = this->_f.extract( gray, keypoints_scene);

  // Saftey check
  if( descriptor_obj.rows == 0 || descriptor_scene.rows == 0 ) {
    return;
  }

  // Match descriptor and filter matches (both ratio test and symmetrical)
  this->_f.match( descriptor_obj, descriptor_scene, matches, true);

  // Add matching keypoints to stored point arrays
  std::vector<cv::DMatch>::iterator ite = matches.begin();
  for( ; ite != matches.end(); ++ite) {
    this->_src.push_back(keypoints_obj[ite->queryIdx].pt);
    this->_dst.push_back(keypoints_scene[ite->trainIdx].pt);
  }

  // IF more than 1000 points - exit
  ROS_INFO("Collected %d (of 250) point pairs.", (int)this->_src.size());
  if( this->_src.size() >= 250 ) {
    this->_done = true;
  }
  */
}

// ------------------
// Finds a perspective transformation between two planes.
// Save transformation to main dor of this ROS package. 
// ------------------
void Alignment::calculate() {
  if( this->_done ) {
    std::string path = ros::package::getPath("qbo_calibration");
    std::string filename = "transformation.yaml";
    ROS_INFO("Start saving perspective transformation to '%s'... ", filename.c_str());

    // Find perspective transformation
    cv::Mat H = cv::findHomography( this->_src, 
				    this->_dst, 
				    CV_RANSAC); // Methods: CV_RANSAC or CV_LMEDS
    // Save transformation
    cv::FileStorage fs( path + "/config/" + filename, cv::FileStorage::WRITE);
    fs << "transformation" << H;
    fs.release();
    ROS_INFO("[Done.]");
  }
}

// ------------------
// Main function
// ------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_node");

  Alignment _a;

  // Run ROS loop - collecting image points 
  ros::Rate _loop_rate(30); 
  while(_a.ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }

  // Calculate the transformation
  _a.calculate();
  return 0;
}
