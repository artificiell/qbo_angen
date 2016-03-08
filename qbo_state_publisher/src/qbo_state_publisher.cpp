#include <string>
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
//#include <urdf/joint.h>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <sensor_msgs/JointState.h>

class QboStatePublisher {

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map;
  joint_map joints_;
  std::map<std::string, double> positions_;
  robot_state_publisher::RobotStatePublisher *publisher_;
  ros::Time last_;
  ros::NodeHandle nh_, priv_nh_;
  ros::Subscriber sub_;
  
  // ROS control params
  bool shutdown_;
  double rate_;

  // Private callback function
  void msgCallback(const sensor_msgs::JointState::ConstPtr& ptr) {
    last_ = ptr->header.stamp;

    // Set new positions
    for( uint i = 0; i < ptr->name.size(); i++) { 
      for(joint_map::iterator it =  joints_.begin(); it != joints_.end(); ++it) {
       	if( ptr->name[i] == it->first ) {
	  urdf::Joint* joint = it->second.get();
	  if( joint->type==urdf::Joint::REVOLUTE || joint->type==urdf::Joint::CONTINUOUS ||
	      joint->type==urdf::Joint::PRISMATIC ) {
	    positions_[ptr->name[i]] = ptr->position[i];
	  }
	}
      }
    }
  }

public:
  QboStatePublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), shutdown_(false) {

    // Set up ROS subscriber
    this->sub_ = nh_.subscribe("/head_joints", 10, &QboStatePublisher::msgCallback, this);

    // Read ROS parameters
    if( !ros::param::get("~publish_frequency", rate_) ) {
      rate_ = 30.0;  // Default ROS loop rate
    }

    urdf::Model model;
    if (!model.initParam("/robot_description")){
      ROS_ERROR("Failed to parse urdf robot model");
      shutdown_ =  true;
    }
    
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel( model, tree)){
      ROS_ERROR("Failed to construct kdl tree");
      shutdown_ =  true;
    }

    // Init the state publischer 
    publisher_ = new robot_state_publisher::RobotStatePublisher(tree);
    joints_ = model.joints_;
  }

  ~QboStatePublisher() {
    delete publisher_;
  }
  
  // Publish the tf stack
  void publishTransformation() {
    publisher_->publishTransforms(positions_, last_, "");
    publisher_->publishFixedTransforms(""); 
  }
  

  // For 'ROS loop' access                                                                    
  void spin() {
    ros::Rate rate(this->rate_);
    while (ros::ok() && !this->shutdown_) {
      this->publishTransformation();
      ros::spinOnce();
      rate.sleep();
    }
  }

};

// ------------------------------------------                                                             
// Main function                                                                                          
// ------------------------------------------ 
int main(int argc, char** argv) {
  ros::init(argc, argv, "qbo_state_publisher");
  ros::NodeHandle nh;
  QboStatePublisher node(nh);
  node.spin();
  return 0;
};
