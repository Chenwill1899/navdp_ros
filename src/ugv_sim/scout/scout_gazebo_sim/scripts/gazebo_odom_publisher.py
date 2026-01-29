#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

class GazeboOdomPublisher:
    def __init__(self):
        rospy.init_node('gazebo_odom_publisher')
        
        self.model_name = rospy.get_param('~model_name', 'scout/')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.last_time = rospy.Time(0)
        self.min_update_period = rospy.Duration(0.01)  # 100Hz max
        
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        rospy.loginfo(f"Gazebo odometry publisher started for model: {self.model_name}")
    
    def model_states_callback(self, msg):
        try:
            # Find the robot model in the list
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        
        current_time = rospy.Time.now()
        
        # Skip if not enough time has passed since last update
        if (current_time - self.last_time) < self.min_update_period:
            return
        
        self.last_time = current_time
        
        pose = msg.pose[index]
        twist = msg.twist[index]
        
        # Publish TF
        self.tf_broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            current_time,
            self.base_frame,
            self.odom_frame
        )
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose = pose
        odom.twist.twist = twist
        
        self.odom_pub.publish(odom)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GazeboOdomPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
