#ifndef ALIGNMENT_HPP_
#define ALIGNMENT_HPP_

#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//#include <features.hpp>

class Alignment {
  bool _done; 

  // Features and transformation 
  //Features _f;
  cv::Size _pattern_sz;
  std::vector<cv::Point2f> _src, _dst;
  int _counter;

  // Type define the sync policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, 
    sensor_msgs::Image> ImagesSyncPolicy;
  
  // ROS
  ros::NodeHandle _nh;
  message_filters::Synchronizer<ImagesSyncPolicy> *_sync;
  message_filters::Subscriber<sensor_msgs::Image> *_obj_sub;
  message_filters::Subscriber<sensor_msgs::Image> *_scene_sub;
  
 void process( const sensor_msgs::ImageConstPtr &obj_msg,
	       const sensor_msgs::ImageConstPtr &scene_msg );

public:
  Alignment();
  ~Alignment();

  void calculate();

  // Access for the ROS handle function
  bool ok() { return _nh.ok() && !this->_done; }
};

#endif // ALIGNMENT_HPP_
