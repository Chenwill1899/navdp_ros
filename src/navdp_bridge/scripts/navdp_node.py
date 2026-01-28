#!/usr/bin/env python3
import rospy
import sys
import os
import numpy as np
import cv2
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError

# Add NavDP source to path
# Assuming this script is in src/navdp_bridge/scripts/
# And NavDP is in src/NavDP/baselines/navdp/
script_dir = os.path.dirname(os.path.abspath(__file__))
navdp_path = os.path.join(script_dir, "../../NavDP/baselines/navdp")
sys.path.append(navdp_path)

try:
    from policy_agent import NavDP_Agent
except ImportError as e:
    rospy.logerr(f"Failed to import NavDP_Agent from {navdp_path}. Error: {e}")
    sys.exit(1)

def get_color(value, min_val, max_val):
    # Map value to color (R,G,B)
    # Simple heatmap: Blue (low) -> Green -> Red (high)
    norm = (value - min_val) / (max_val - min_val + 1e-6)
    # Clamp
    norm = max(0.0, min(1.0, norm))
    
    r, g, b = 0.0, 0.0, 0.0
    if norm < 0.5:
        # Blue to Green
        b = 1.0 - 2 * norm
        g = 2 * norm
    else:
        # Green to Red
        g = 1.0 - 2 * (norm - 0.5)
        r = 2 * (norm - 0.5)
        
    return r, g, b

class NavDPNode:
    def __init__(self):
        rospy.init_node("navdp_node")
        
        # Parameters
        self.checkpoint = rospy.get_param("~checkpoint", os.path.join(navdp_path, "checkpoints/cross-waic-final4-125.ckpt"))
        self.stop_threshold = rospy.get_param("~stop_threshold", 0.5) 
        self.device = rospy.get_param("~device", "cuda:0")
        
        # 相机内参：优先从参数获取，否则从camera_info话题订阅
        fx = rospy.get_param("~camera_fx", None)
        fy = rospy.get_param("~camera_fy", None)
        cx = rospy.get_param("~camera_cx", None)
        cy = rospy.get_param("~camera_cy", None)
        
        # Current status
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.goal_robot_frame = None # [x, y]
        self.current_goal_msg = None
        self.current_odom = None  # 存储最新的里程计信息
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()  # 发布TF变换
        self.intrinsics = None 
        self.agent = None  # 必须在订阅器之前初始化
        
        # 如果提供了完整的内参参数，直接使用
        if all(v is not None for v in [fx, fy, cx, cy]):
            self.intrinsics = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float32)
            rospy.loginfo(f"✓ Using intrinsics from parameters:\n{self.intrinsics}")
            self._init_agent()
        
        # Subscribers
        self.sub_rgb = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb, queue_size=1)
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb, queue_size=1)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber("/Odometry", Odometry, self.odom_cb, queue_size=1)
        
        # 只有在没有手动配置内参时才订阅camera_info
        if self.intrinsics is None:
            self.sub_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_cb, queue_size=1)
            rospy.loginfo("Waiting for camera_info to get intrinsics...")
        
        # Publishers
        self.pub_path = rospy.Publisher("/navdp/trajectory", Path, queue_size=1)
        self.pub_all_trajs = rospy.Publisher("/navdp/all_trajectories", MarkerArray, queue_size=1)
        self.pub_goal_marker = rospy.Publisher("/navdp/current_goal", Marker, queue_size=1)
        self.pub_debug_img = rospy.Publisher("/navdp/debug_image", Image, queue_size=1)
        self.pub_rgb_viz = rospy.Publisher("/navdp/rgb_viz", Image, queue_size=1)
        self.pub_depth_viz = rospy.Publisher("/navdp/depth_viz", Image, queue_size=1)
        self.pub_traj_overlay = rospy.Publisher("/navdp/trajectory_overlay", Image, queue_size=1)
        
        # Timer for inference loop
        # On AGX Orin, we might want to go as fast as possible or limit to 10Hz
        rospy.Timer(rospy.Duration(0.1), self.inference_loop) 
        
        rospy.loginfo("NavDP Node Initialized. Waiting for Camera Info and First Goal...")

    def info_cb(self, msg):
        if self.intrinsics is None:
            K = np.array(msg.K).reshape(3,3)
            self.intrinsics = K
            rospy.loginfo(f"✓ Got Camera Intrinsics:\n{self.intrinsics}")
            self._init_agent()

    def _init_agent(self):
        if self.agent is not None:
            return
        
        try:
            rospy.loginfo(f"✓ Initializing NavDP agent on {self.device}...")
            self.agent = NavDP_Agent(
                image_intrinsic=self.intrinsics,
                image_size=224,
                memory_size=8,  # For checkpoint: expect 8*16+4=132 but got 130, use strict=False
                predict_size=24,
                temporal_depth=16,
                heads=8,
                token_dim=384,
                navi_model=self.checkpoint,
                device=self.device
            )
            self.agent.reset(batch_size=1, threshold=self.stop_threshold)
            rospy.loginfo("✓✓ NavDP agent initialized successfully and READY!")
            rospy.loginfo("NavDP Agent Loaded and Reset. Ready for goals.")
        except Exception as e:
            rospy.logerr(f"Error initializing NavDP Agent: {e}")

    def rgb_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb = cv_image
            rospy.loginfo_once(f"✓ RGB callback working: shape={cv_image.shape}")
        except CvBridgeError as e:
            rospy.logerr_throttle(5, f"RGB Bridge Error: {e}")

    def depth_cb(self, msg):
        try:
            if msg.encoding == "16UC1":
                cv_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                cv_depth = cv_depth.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            else:
                rospy.logwarn_throttle(10, f"Unsupported depth encoding: {msg.encoding}")
                return
            self.latest_depth = cv_depth
            rospy.loginfo_once(f"✓ Depth callback working: shape={cv_depth.shape}, range=[{cv_depth.min():.2f}, {cv_depth.max():.2f}]m")
        except CvBridgeError as e:
            rospy.logerr_throttle(5, f"Depth Bridge Error: {e}")

    def goal_cb(self, msg):
        self.current_goal_msg = msg
        rospy.loginfo(f"✓ Received goal: frame={msg.header.frame_id}, pos=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        if self.agent:
            self.agent.reset_env(0) 

    def odom_cb(self, msg):
        """存储最新的里程计信息并发布TF变换"""
        self.current_odom = msg
        rospy.loginfo_once(f"✓ Odometry callback working: frame={msg.header.frame_id}")
        
        # 从Odometry消息发布TF: map -> base_link
        # Odometry消息包含机器人在世界坐标系中的位姿
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            msg.header.stamp,
            "base_link",  # 机器人坐标系
            "map"  # 世界坐标系（静止）
        )

    def get_goal_in_robot_frame(self):
        """将世界坐标系的目标点转换到相机坐标系"""
        if self.current_goal_msg is None:
            return None
        
        if self.current_odom is None:
            rospy.logwarn_throttle(5, "No odometry available, cannot transform goal")
            return None
        
        # 目标点在世界坐标系中的位置
        goal_world_x = self.current_goal_msg.pose.position.x
        goal_world_y = self.current_goal_msg.pose.position.y
        
        # 机器人当前位姿
        robot_x = self.current_odom.pose.pose.position.x
        robot_y = self.current_odom.pose.pose.position.y
        ori = self.current_odom.pose.pose.orientation
        
        # 机器人朝向（yaw角）
        euler = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        robot_yaw = euler[2]
        
        # 世界坐标系下目标相对机器人的向量
        dx_world = goal_world_x - robot_x
        dy_world = goal_world_y - robot_y
        
        # 转换到机器人坐标系（逆旋转）
        cos_yaw = np.cos(-robot_yaw)
        sin_yaw = np.sin(-robot_yaw)
        dx_robot = cos_yaw * dx_world - sin_yaw * dy_world
        dy_robot = sin_yaw * dx_world + cos_yaw * dy_world
        
        # NavDP期望：x前（正值），y左
        # 如果目标在后方，翻转坐标
        if dx_robot < 0:
            dx_robot = abs(dx_robot)
            dy_robot = -dy_robot
            rospy.logwarn_throttle(2, f"Goal behind robot, flipping coordinates")
        
        # 限制在模型训练范围内
        dx_robot = np.clip(dx_robot, 0.5, 10.0)
        dy_robot = np.clip(dy_robot, -10.0, 10.0)
        
        distance = np.sqrt(dx_robot**2 + dy_robot**2)
        rospy.loginfo_throttle(2, f"Goal in robot frame: forward={dx_robot:.2f}m, lateral={dy_robot:.2f}m, dist={distance:.2f}m")
        rospy.loginfo_throttle(2, f"Goal in world: ({goal_world_x:.2f}, {goal_world_y:.2f}), Robot at: ({robot_x:.2f}, {robot_y:.2f})")
        
        return np.array([dx_robot, dy_robot])

    def transform_point_to_world(self, x, y, z=0.0):
        """将相机坐标系下的点转换到世界坐标系（odom frame）"""
        if self.current_odom is None:
            rospy.logwarn_throttle(5, "No odometry available for coordinate transform")
            return None
        
        # 获取机器人当前位姿
        pos = self.current_odom.pose.pose.position
        ori = self.current_odom.pose.pose.orientation
        
        # 转换四元数到欧拉角（只需要yaw）
        euler = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw = euler[2]
        
        # 旋转矩阵（2D）
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # 相机坐标系的点转到世界坐标系
        # 假设相机朝前（x轴向前，y轴向左）
        world_x = pos.x + cos_yaw * x - sin_yaw * y
        world_y = pos.y + sin_yaw * x + cos_yaw * y
        world_z = pos.z + z
        
        return (world_x, world_y, world_z)

    def publish_goal_marker(self, goal_world_x, goal_world_y):
        """发布目标点标记（直接使用世界坐标）"""
        marker = Marker()
        marker.header.frame_id = "map"  # 使用map作为世界坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navdp_goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_world_x
        marker.pose.position.y = goal_world_y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.pub_goal_marker.publish(marker)

    def publish_all_trajectories(self, all_trajs, values):
        """发布所有候选轨迹（世界坐标系）"""
        # all_trajs shape: (Batch=1, Heads=16, Horizon=24, 3) 
        # or (1, Heads, Horizon, 3) depending on policy_network output.
        # values: (Batch=1, Heads=16)
        
        if self.current_odom is None:
            rospy.logwarn_throttle(5, "No odometry for trajectory transform")
            return
        
        # Based on policy_network.py inspection:
        # return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), ...
        # all_trajectory reshaped to (B, sample_num=16, T, 3)
        
        if len(all_trajs.shape) < 3:
            return

        trajs = all_trajs[0] # (Heads, Horizon, 3)
        vals = values[0] # (Heads,)

        ma = MarkerArray()
        
        max_val = np.max(vals)
        min_val = np.min(vals)

        for i, traj in enumerate(trajs):
            marker = Marker()
            marker.header.frame_id = "map"  # 使用map作为世界坐标系
            marker.header.stamp = rospy.Time.now()
            marker.ns = "navdp_candidates"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.03 # Thin lines
            
            score = vals[i]
            r, g, b = get_color(score, min_val, max_val)
            
            marker.color.a = 0.5
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            
            # 将每个轨迹点转换到世界坐标系
            for pt in traj:
                world_pt = self.transform_point_to_world(pt[0], pt[1], 0.05)
                if world_pt is None:
                    continue
                p = Point()
                p.x = world_pt[0]
                p.y = world_pt[1]
                p.z = world_pt[2]
                marker.points.append(p)
                
            ma.markers.append(marker)
        
        self.pub_all_trajs.publish(ma)

    def inference_loop(self, event):
        # 始终发布输入图像可视化（即使没有目标）
        if self.latest_rgb is not None:
            rgb_viz = cv2.resize(self.latest_rgb, (640, 480))
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_viz, "bgr8")
            self.pub_rgb_viz.publish(rgb_msg)
        
        if self.latest_depth is not None:
            depth_viz = self.latest_depth.copy()
            depth_viz = np.clip(depth_viz * 1000, 0, 5000).astype(np.uint16)
            depth_color = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_viz, alpha=0.05), 
                cv2.COLORMAP_JET
            )
            depth_msg = self.bridge.cv2_to_imgmsg(depth_color, "bgr8")
            self.pub_depth_viz.publish(depth_msg)
        
        # 以下需要agent和目标点
        if self.agent is None or self.latest_rgb is None or self.latest_depth is None:
            rospy.logwarn_throttle(10, f"⏳ Waiting... agent={self.agent is not None}, rgb={self.latest_rgb is not None}, depth={self.latest_depth is not None}")
            return

        goal_xy = self.get_goal_in_robot_frame()
        if goal_xy is None:
            rospy.logwarn_throttle(5, "⏳ No valid goal available (set with '2D Nav Goal' in RViz)")
            return
        
        rospy.loginfo_throttle(5, f"🚀 Running inference with goal: forward={goal_xy[0]:.2f}m, lateral={goal_xy[1]:.2f}m")
        
        # 发布目标点标记（使用原始世界坐标）
        self.publish_goal_marker(
            self.current_goal_msg.pose.position.x,
            self.current_goal_msg.pose.position.y
        )
            
        goal_input = np.array([[goal_xy[0], goal_xy[1], 0.0]], dtype=np.float32)
        
        # 输入数据 - agent内部会处理归一化和resize
        # RGB保持原始0-255范围，agent.process_image会除以255
        rgb_input = self.latest_rgb[np.newaxis, ...].astype(np.uint8)
        depth_input = self.latest_depth[np.newaxis, :, :, np.newaxis].astype(np.float32)
        
        # 调试：打印输入形状和统计信息
        rospy.loginfo_throttle(10, f"Input shapes - RGB: {rgb_input.shape}, Depth: {depth_input.shape}, Goal: {goal_input.shape}")
        rospy.loginfo_throttle(10, f"RGB range: [{rgb_input.min()}, {rgb_input.max()}], Depth range: [{depth_input.min():.2f}, {depth_input.max():.2f}]")
        
        # 检查NaN和无效值
        if np.isnan(rgb_input).any() or np.isnan(depth_input).any() or np.isnan(goal_input).any():
            rospy.logerr("Invalid input: NaN detected")
            return
        
        # 限制深度范围
        depth_input = np.clip(depth_input, 0.0, 10.0)
        depth_input = np.nan_to_num(depth_input, nan=0.0, posinf=10.0, neginf=0.0) 
        
        try:
            # Output from `step_pointgoal`: good_trajectory[:,0], all_trajectory, all_values, trajectory_mask
            # good_trajectory is the selected trajectory to execute.
            traj_execute, all_traj, all_vals, mask = self.agent.step_pointgoal(goal_input, rgb_input, depth_input)
            
            # 1. Publish Best Trajectory (Path) in world coordinates
            # traj_execute structure check:
            # policy_agent.py: return good_trajectory[:,0]
            # policy_network: positive_trajectory = all_trajectory[indices of topk] (Top 2)
            # So good_trajectory is (B, 2, T, 3).
            # good_trajectory[:,0] is (B, T, 3) -> Taking the absolute best one.
            # So traj_execute[0] is (T, 3).
            
            if self.current_odom is None:
                rospy.logwarn_throttle(5, "No odometry for path publishing")
                return
            
            if len(traj_execute.shape) > 1 and traj_execute.shape[0] == 1:
                # Unwrap batch
                best_traj = traj_execute[0]
            else:
                best_traj = traj_execute

            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"  # 使用map作为世界坐标系
            
            for pt in best_traj:
                world_pt = self.transform_point_to_world(pt[0], pt[1], 0.0)
                if world_pt is None:
                    continue
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = world_pt[0]
                pose.pose.position.y = world_pt[1]
                pose.pose.position.z = world_pt[2]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.pub_path.publish(path_msg)
            rospy.loginfo_throttle(2, f"Published trajectory with {len(path_msg.poses)} waypoints in world frame")
            
            # 2. Publish All Candidates (MarkerArray)
            self.publish_all_trajectories(all_traj, all_vals)
            
            # 3. Publish Debug Image (trajectory overlay)
            if mask is not None:
                vis_msg = self.bridge.cv2_to_imgmsg(mask, "bgr8")
                self.pub_debug_img.publish(vis_msg)
                self.pub_traj_overlay.publish(vis_msg)

        except Exception as e:
            import traceback
            rospy.logerr_throttle(1, f"Inference Error: {e}")
            rospy.logerr_throttle(10, traceback.format_exc())

if __name__ == "__main__":
    node = NavDPNode()
    rospy.spin()
