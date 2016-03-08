 << Changes for Ängen >>

-- /script/qbo_joint_odom.py

Filter in 'on_head_pan_joint' and 'on_head_tilt_joint' because 
of noise in motor state readings. Angle should never be more/less 
than +/- 45 degrees.

The tilt angle is not accurate.                              
Compensation added in '_publish_state' in order to get a correct 
tf transformation for the Asus Xtion. 


-- src/qbo_xtion_tf.cpp

New ROS node for broadcasting the transform (tf) for the Asus Xtion.
The transorm is based on the center of the head ('head' link), and 
assumes that the Xtion is pointing downwards at 0.568 rad.


-- launch/joint_odom_with_base_link.launch

Added a node for launching the Xtion tf transform.