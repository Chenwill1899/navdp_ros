#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit")
        
        self.lookahead_dist = rospy.get_param("~lookahead_dist", 0.5)
        self.v_max = rospy.get_param("~v_max", 0.5)
        self.k_w = rospy.get_param("~k_w", 2.0)
        
        self.path = None
        self.sub_path = rospy.Subscriber("/navdp/trajectory", Path, self.path_cb, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.Timer(rospy.Duration(0.1), self.loop)
        
    def path_cb(self, msg):
        # Assume base_link frame
        pts = []
        for p in msg.poses:
            pts.append([p.pose.position.x, p.pose.position.y])
        self.path = np.array(pts)
        
    def loop(self, event):
        if self.path is None or len(self.path) == 0:
            return
            
        # Find point at lookahead distance
        target = None
        for p in self.path:
            dist = np.linalg.norm(p)
            if dist > self.lookahead_dist:
                target = p
                break
        
        if target is None:
            target = self.path[-1]
            
        # Calculate curriculum
        # x is forward, y is left in base_link
        # curvature = 2*y / L^2
        x, y = target
        L2 = x**2 + y**2
        
        if L2 < 0.001:
            v = 0.0
            w = 0.0
        else:
            v = self.v_max
            # curavture kappa = 2*y / L^2
            # w = v * kappa = 2*v*y / L^2
            w = 2 * v * y / L2
            
            # clamp
            w = np.clip(w, -1.0, 1.0)
            
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

if __name__ == "__main__":
    node = PurePursuit()
    rospy.spin()
