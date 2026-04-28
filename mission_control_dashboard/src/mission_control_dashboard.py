"""
ROS 2 Lifecycle Web Mission Control Interface (2D Canvas & Video Streaming)
"""

import os
import threading
import base64
import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# --- Camera Imports ---
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

from custom_map_interfaces.srv import GetMatrices

from flask import Flask, render_template
from flask_socketio import SocketIO

pkg_share = get_package_share_directory('mission_control_dashboard')
template_dir = os.path.join(pkg_share, 'templates')

app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'lunar_mission_secure'

socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*", max_http_buffer_size=10000000)

ros_node = None


class LifecycleMissionControlUI(rclpy.lifecycle.Node):
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        
        # Core Subscriptions
        self._sub_odom = None
        self._sub_global_path = None
        self._sub_trajectory = None
        
        # Camera Subscriptions
        self._sub_color = None
        self._sub_depth = None
        self.bridge = CvBridge()
        
        self._pub_goal = None
        self._cli_get_maps = None
        
        self.declare_parameter('global_res', 1.0)
        self.map_resolution = self.get_parameter('global_res').value

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: Node configured. Initializing interfaces.")
        
        self._pub_goal = self.create_lifecycle_publisher(PoseStamped, 'goal_pose', 10)
        self._cli_get_maps = self.create_client(GetMatrices, 'get_matrices')
            
        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self._sub_global_path = self.create_subscription(Path, 'global_path', self.global_path_callback, 10)
        self._sub_trajectory = self.create_subscription(Path, 'driven_trajectory', self.trajectory_callback, 10)

        # --- NEW: Camera Subscriptions ---
        self._sub_color = self.create_subscription(
            CompressedImage, '/unreal/camera0/image_color/compressed', self.color_callback, 10)
            
        self._sub_depth = self.create_subscription(
            Image, '/unreal/camera0/image_depth', self.depth_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: Node active. Telemetry stream engaged.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: Node deactivated. Telemetry stream suspended.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._sub_global_path: self.destroy_subscription(self._sub_global_path)
        if self._sub_trajectory: self.destroy_subscription(self._sub_trajectory)
        if self._sub_color: self.destroy_subscription(self._sub_color)
        if self._sub_depth: self.destroy_subscription(self._sub_depth)
        if self._pub_goal: self.destroy_publisher(self._pub_goal)
        if self._cli_get_maps: self.destroy_client(self._cli_get_maps)
        return TransitionCallbackReturn.SUCCESS

    # =========================================================================
    # MAP & PATHING CALLBACKS
    # =========================================================================

    def fetch_maps(self) -> None:
        if self._cli_get_maps is None:
            return

        if not self._cli_get_maps.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Planner map service is not available yet.")
            return
            
        self.get_logger().info("Requesting map from planner...")
        req = GetMatrices.Request()
        future = self._cli_get_maps.call_async(req)
        future.add_done_callback(self.map_response_callback)

    def map_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Successfully downloaded map matrices from Planner.")
            
            map_data_list = list(response.elevation_map.data)
            
            socketio.emit('map_update', {
                'data': map_data_list,
                'width': response.elevation_map.layout.dim[1].size,
                'height': response.elevation_map.layout.dim[0].size,
                'resolution': self.map_resolution
            })
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def global_path_callback(self, msg: Path) -> None:
        if not self._is_active(): return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
        socketio.emit('global_path_update', {'path': coords})

    def trajectory_callback(self, msg: Path) -> None:
        if not self._is_active(): return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
        socketio.emit('trajectory_update', {'path': coords})

    def odom_callback(self, msg: Odometry) -> None:
        if not self._is_active(): return
        socketio.emit('odom_update', {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        })

    # =========================================================================
    # CAMERA STREAMING CALLBACKS
    # =========================================================================

    def color_callback(self, msg: CompressedImage) -> None:
        if not self._is_active(): return
        # Pass the compressed JPEG bytes directly to the browser
        b64_img = base64.b64encode(msg.data).decode('utf-8')
        socketio.emit('color_frame', {'image': b64_img})

    def depth_callback(self, msg: Image) -> None:
        if not self._is_active(): return
        try:
            # 1. Convert raw ROS 32FC1 Image to an OpenCV float matrix
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_img = np.array(cv_img, dtype=np.float32)
            
            # 2. Find the sky value (since your publisher clamps posinf to max_distance, 
            # the sky is guaranteed to be the absolute highest value in this array)
            sky_val = np.max(cv_img)
            
            # 3. Normalize the raw float depths to 0-255 for the colormap
            if sky_val > 0.0:
                norm_img = (cv_img / sky_val * 255.0).astype(np.uint8)
            else:
                norm_img = cv_img.astype(np.uint8)

            # 4. Apply the JET colormap (Close = Dark Blue, Far = Dark Red)
            color_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)

            # 5. Mask the sky! Any pixel that equals the max distance becomes pure black.
            # (We subtract 0.001 as a tiny safety margin for 32-bit float rounding errors)
            sky_mask = (cv_img >= sky_val - 0.001)
            color_img[sky_mask] = [0, 0, 0]

            # 6. Compress to JPEG and send to the web browser
            _, buffer = cv2.imencode('.jpg', color_img)
            b64_img = base64.b64encode(buffer).decode('utf-8')
            socketio.emit('depth_frame', {'image': b64_img})
            
        except Exception as e:
            self.get_logger().error(f"Depth conversion fault: {e}")

    # =========================================================================
    # UI DISPATCH
    # =========================================================================

    def dispatch_goal(self, x: float, y: float) -> None:
        if self._pub_goal and self._pub_goal.is_activated:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            
            # Explicit float casts to prevent C-bridge Assertion PyFloat_Check failure
            goal_msg.pose.position.x = float(x)
            goal_msg.pose.position.y = float(y)
            goal_msg.pose.position.z = 0.0
            
            self._pub_goal.publish(goal_msg)
        else:
            self.get_logger().warn("Goal dispatch rejected: Node not in ACTIVE state.")

    def _is_active(self) -> bool:
        return self._state_machine.current_state[1] == 'active'


# =========================================================================
# FLASK APPLICATION ROUTING
# =========================================================================

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect(auth=None):
    if ros_node is not None:
        ros_node.get_logger().info("New browser connected. Triggering Service Call...")
        ros_node.fetch_maps()

@socketio.on('send_goal')
def handle_goal(json_data):
    if ros_node is not None:
        ros_node.dispatch_goal(json_data['x'], json_data['y'])

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = LifecycleMissionControlUI('mission_control_dashboard')
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()