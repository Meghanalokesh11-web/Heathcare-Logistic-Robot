#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler

class AutonomousMapper:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('autonomous_mapper', anonymous=True)
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Subscribe to the Odometry topic
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publish the map to odom transform
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Autonomous Mapper initialized")
    
    def odom_callback(self, msg):
        # Get the current time
        current_time = rospy.Time.now()
        
        # Extract position and orientation from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Assuming we want to publish the transform from odom to base_footprint
        # In this case, we need to convert the quaternion orientation into roll, pitch, yaw
        euler = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # Here we use the 'odom' frame as the parent and 'base_footprint' frame as the child
        # You can modify this based on your specific frame names
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        
        # Convert Euler angles to quaternion
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
    
    def run(self):
        # Keep the node running
        rospy.loginfo("Running autonomous mapper node...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mapper = AutonomousMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
