#!/usr/bin/env python3
import rospy
import numpy as np
import cvxpy as cp
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class MPCController:
    def __init__(self):
        rospy.init_node("mpc_controller")
        
        # Parameters
        self.dt = rospy.get_param("~dt", 0.1)
        self.N = rospy.get_param("~horizon", 10)
        self.v_max = rospy.get_param("~v_max", 0.5)
        self.omega_max = rospy.get_param("~omega_max", 1.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.2)
        
        self.path = None
        self.path_frame = "base_link"
        self.latest_odom = None
        
        # Subscribers
        self.sub_path = rospy.Subscriber("/navdp/trajectory", Path, self.path_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        
        # Publisher
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self.tf_listener = tf.TransformListener()
        
        # Helper for transforming path
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
    def path_cb(self, msg):
        self.path = msg
        self.path_frame = msg.header.frame_id
        
    def odom_cb(self, msg):
        self.latest_odom = msg

    def get_robot_pose(self):
        # Return [x, y, theta] in odom frame (or world frame depending on config)
        if self.latest_odom is None:
            return None
        
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return np.array([p.x, p.y, yaw])

    def transform_path_to_robot(self):
        # The path from NavDP is usually in "base_link" (relative to robot at time T0)
        # BUT, if path is Latched or older, and robot moved, we need to transform it?
        # NavDP constantly republishes path relative to CURRENT robot.
        # So points are [x, y] where x is forward, y is left relative to current robot.
        # So target state for robot (which is at 0,0,0 in base_link) is just these points.
        # We can run MPC in ROBOT FRAME directly.
        
        if self.path is None:
            return None
            
        # Extract points
        wps = []
        for p in self.path.poses:
            # Assume poses are in base_link already if node sets it so
            # If path_frame != base_link, we might need transform. 
            # NavDP node sets frame_id="base_link".
            wps.append([p.pose.position.x, p.pose.position.y])
            
        return np.array(wps)

    def solve_mpc(self, ref_traj):
        # State: [x, y, theta]
        # Control: [v, omega]
        # Robot is at [0, 0, 0] in base_link
        
        if len(ref_traj) == 0:
            return 0.0, 0.0

        # Horizon
        curr_N = min(self.N, len(ref_traj))
        if curr_N < 1:
            return 0.0, 0.0

        # Variables
        X = cp.Variable((curr_N + 1, 3))
        U = cp.Variable((curr_N, 2))
        
        cost = 0
        constraints = []
        
        # Initial State (Robot Frame)
        constraints += [X[0, :] == np.array([0.0, 0.0, 0.0])]
        
        for k in range(curr_N):
            # Cost: Distance to reference point
            # We want position matches. Theta? Maybe align with path tangent?
            # For simplicity, just position tracking.
            ref_pos = ref_traj[k] # [x, y]
            
            cost += cp.sum_squares(X[k+1, :2] - ref_pos) * 10.0
            cost += cp.sum_squares(U[k, :]) * 0.1 # Minimize control effort
            
            # Dynamics (Linearized around 0,0,0)
            # x_k+1 = x_k + v*dt * cos(theta_k) ~ x_k + v*dt (small theta)
            # y_k+1 = y_k + v*dt * sin(theta_k) ~ y_k + v*dt * theta_k (bilinear!)
            # theta_k+1 = theta_k + w*dt
            
            # This is non-convex. 
            # Standard approach: Linearize around Ref Trajectory or Operating Point.
            # Simple approach for low speed:
            # x' = x + v*dt
            # y' = y + v_nominal * theta * dt  (approximate)
            # th' = th + w*dt
            
            # BETTER: Use a library or specific linearization. 
            # Here I use a very simplified non-holonomic model approximation for cold start
            # Or just formulate as P-Control for now via Optimization?
            
            # Let's use a "Point Mass" model for XY optimization and then map to V/Omega?
            # No, that fails non-holonomic constraint.
            
            # Linearization around current robot (0,0,0) and v=v_target?
            # A_k = I, B_k = [[dt, 0], [0, 0], [0, dt]] -> Only moves in X and Theta? No Y motion?
            # At theta=0, y_dot = v * sin(0) = 0. You CANNOT move in Y locally.
            # You must turn first.
            
            # This is why MPC needs trajectory linearization.
            # Linearizing around the Reference Path is best.
            # Ref point: xr, yr
            # Ref theta needs to be computed? atan2(dy, dx)
            
            # Due to complexity of implementing Full Nonlinear MPC in a single script with cvxpy (which requires DPP rules),
            # I will assume the trajectory provided by NavDP is feasible and we try to track it.
            # But creating a robust MPC from scratch here is risky.
            
            # ALTERNATIVE: Use DWA or Trajectory Rollout.
            # Or simplified MPC: 
            # Optimize sequence of v, w to minimize distance to lookahead point?
            
            pass

        # FALLBACK: Pure Pursuit is often better than a broken MPC.
        # But user asked for MPC.
        # I will implement a "Predictive Controller" that just optimizes U to minimize error at lookahead.
        
        # Simplified MPC Logic (Receding Horizon of 1 step or multiple steps with simplified model)
        # Let's use a shooting method with `scipy.optimize`? No, slow.
        # Let's use `cvxpy` with Fixed Linearization around Reference.
        
        # Build Reference Full State [x, y, theta]
        ref_full = []
        for i in range(len(ref_traj)):
            x, y = ref_traj[i]
            if i < len(ref_traj)-1:
                dx = ref_traj[i+1][0] - x
                dy = ref_traj[i+1][1] - y
                th = np.arctan2(dy, dx)
            elif i > 0:
                th = ref_full[-1][2]
            else:
                th = 0 # Pointing at point
            ref_full.append([x, y, th])
        
        # Linear MPC
        constraints = [X[0,:] == np.zeros(3)]
        
        for k in range(curr_N):
            # Linearize around Ref[k]
            xr, yr, thr = ref_full[k]
            vr = 0.3 # Nominal velocity? Or estimated from path dists?
            wr = 0.0
            
            # Error Dynamics Model? Too complex for this snippet.
            
            # Let's try to stick to the CVXPY snippet provided in thought trace, 
            # but noting the linearization issue.
            # x_{k+1} = x_k + B u_k
            # B = [[dt, 0], [0, 0], [0, dt]] (Linearized at theta=0)
            # This allows control of x and theta. It assumes y stays 0.
            # But the path might have Y != 0.
            # If y != 0, we need to correct theta.
            # This linearization is valid only for straight lines.
            
            # Correct Approach for Custom Code: 
            # Just use `Control Lyapunov Function` or `Pure Pursuit` style logic but formatted as an optimization problem?
            # Or use Iterative LQR / Latency Consideration.
            
            pass 
        return None

    def simple_mpc_approx(self, ref_traj):
        # A simple cost checking valid trajectories (DWA-like) might be more robust here 
        # but technically is MPC (Model Predictive Control).
        
        best_u = [0, 0]
        min_cost = float('inf')
        
        # Sampling based MPC (common in robotics)
        # v in [0, v_max], w in [-w_max, w_max]
        # Predict T steps
        # This is robust and easy to implement without solvers.
        
        vs = np.linspace(0, self.v_max, 5)
        ws = np.linspace(-self.omega_max, self.omega_max, 11)
        
        dt = self.dt
        n_steps = min(self.N, 10)
        
        # Reference path
        ref = np.array(ref_traj)
        
        for v in vs:
            for w in ws:
                # Simulate
                x, y, th = 0.0, 0.0, 0.0
                cost = 0
                
                for k in range(n_steps):
                    # update
                    x += v * np.cos(th) * dt
                    y += v * np.sin(th) * dt
                    th += w * dt
                    
                    # Distance to nearest point or k-th point?
                    # Tracking k-th point of ref
                    if k < len(ref):
                        rx, ry = ref[k]
                        dist_sq = (x - rx)**2 + (y - ry)**2
                        cost += dist_sq
                    else:
                        # Stay near end
                        rx, ry = ref[-1]
                        dist_sq = (x - rx)**2 + (y - ry)**2
                        cost += dist_sq
                
                # Penalties
                cost += 0.1 * abs(w)  # Smoothness
                cost += 0.01 * (self.v_max - v) # Prefer speed
                
                if cost < min_cost:
                    min_cost = cost
                    best_u = [v, w]
                    
        return best_u

    def control_loop(self, event):
        path_points = self.transform_path_to_robot()
        if path_points is None or len(path_points) == 0:
            # Stop
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        # Check goal reached
        dist = np.linalg.norm(path_points[0]) # Dist to first point? No, path_points is full traj
        # If last point is close?
        dist_end = np.linalg.norm(path_points[-1])
        if dist_end < self.goal_tolerance:
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        # Run "MPC"
        # Since CVXPY setup for non-holonomic without extensive library is error-prone,
        # I use a Sampling-based MPC (Model Predictive Control via sampling).
        # It optimizes a trajectory over a horizon.
        v, w = self.simple_mpc_approx(path_points)
        
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

if __name__ == "__main__":
    node = MPCController()
    rospy.spin()
