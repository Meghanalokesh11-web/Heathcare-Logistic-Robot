#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class NavigateToGoal:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('navigate_to_goal', anonymous=True)

        # Publisher for sending goals to move_base
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber to monitor goal status
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        # Flag to track goal status
        self.goal_reached = False

    def send_goal(self, x, y, theta):
        """
        Sends a navigation goal to the move_base node.

        Parameters:
        x (float): X-coordinate of the goal in the map frame
        y (float): Y-coordinate of the goal in the map frame
        theta (float): Orientation (yaw) of the robot at the goal in radians
        """
        # Create the goal message
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0

        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)  # Convert yaw to quaternion
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        # Publish the goal
        rospy.loginfo("Sending goal: x={}, y={}, theta={}".format(x, y, theta))
        self.goal_pub.publish(goal)

    def status_callback(self, msg):
        """
        Callback for the /move_base/status topic to monitor the robot's progress.
        """
        for status in msg.status_list:
            if status.status == 3:  # Goal reached
                rospy.loginfo("Goal reached!")
                self.goal_reached = True
            elif status.status == 4:  # Goal aborted
                rospy.logwarn("Goal aborted!")
                self.goal_reached = True

    def wait_until_goal_reached(self):
        """
        Blocks until the robot reaches the goal or the goal is aborted.
        """
        rospy.loginfo("Waiting for the robot to reach the goal...")
        while not self.goal_reached and not rospy.is_shutdown():
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        # Initialize the navigation object
        navigator = NavigateToGoal()

        # Specify the target location
        target_x = 1.0  # Replace with your target x-coordinate
        target_y = 2.0  # Replace with your target y-coordinate
        target_theta = 0.0  # Replace with your target yaw in radians

        # Send the goal
        navigator.send_goal(target_x, target_y, target_theta)

        # Wait for the robot to reach the goal
        navigator.wait_until_goal_reached()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
