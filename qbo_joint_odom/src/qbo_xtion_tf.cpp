#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "qbo_xtion_tf_node");
  ros::NodeHandle _nh;

  // Read parameter...
  double tilt = 0.0;
  if( argc == 2 ) {
    tilt = atof(argv[1]);
  }
  else {
    if( !ros::param::get( "~xtion_tilt", tilt) ) {
      ROS_ERROR("Could not read ~xtion_tilt param.");
      return -1;
    }
  }
  double offx=0.05, offz=0.15;

  ros::param::set( "~xtion_offx", offx);
  ros::param::set( "~xtion_offz", offz);

  tf::TransformBroadcaster _br;
  tf::Transform _transform;

  // Running ROS loop
  ros::Rate _loop_rate(10);
  while (_nh.ok()){

    // Set transformation
    _transform.setOrigin( tf::Vector3(offx, 0.0, offz) );
    _transform.setRotation(tf::createQuaternionFromRPY( 0.0, tilt, 0.0) );  // Right axis?

    // Broadcast transformation
    _br.sendTransform(tf::StampedTransform( _transform, ros::Time::now(), "head", "camera_link"));
    
    _loop_rate.sleep();
  }
  return 0;
};
