#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>

class HeadTransformer {

  message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

public:

  HeadTransformer() : tf_(),  target_frame_("head") {
    point_sub_.subscribe(n_, "/head/pose", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(point_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind( &HeadTransformer::msgCallback, this, _1) );
  } ;
  
  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose_ptr) {
    //geometry_msgs::PoseStamped pose_out;
    tf::TransformBroadcaster br;
    tf::Transform transform;

    try {
      transform.setOrigin( tf::Vector3( pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z) );
      transform.setRotation( tf::Quaternion( pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y, pose_ptr->pose.orientation.z, pose_ptr->pose.orientation.w) );
      //transform.setRotation( tf::Quaternion() );
      //tf_.transformPose(target_frame_, *pose_ptr, pose_out);
	
      /*
	printf("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
	       pose_out.pose.position.x,
	       pose_out.pose.position.y,
	       pose_out.pose.position.z);
      */
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "head") );
	
    }
    catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };

};


int main(int argc, char ** argv) {
  ros::init(argc, argv, "qbo_head_transform_node"); //Init ROS
  HeadTransformer ht; // Construct class
  ros::spin(); // Run until interupted 
};
