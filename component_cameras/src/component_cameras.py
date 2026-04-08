from typing import Optional
import time
import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

# Using standard ROS Image message
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

# UnrealCV imports
from unrealcv import Client
from unrealcv.util import read_png, read_npy

import numpy as np
import warnings
warnings.filterwarnings("ignore")


class LifecycleUnrealCameras(rclpy.lifecycle.Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        
        # Define publishers for Camera 3 (RGB and Depth)
        self._pub_rgb:   Optional[Publisher] = None
        self._pub_depth: Optional[Publisher] = None
        
        self._timer: Optional[Timer] = None
        self.unreal_client: Optional[Client] = None
        self.cv_bridge = CvBridge()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip',                    rclpy.Parameter.Type.STRING),
                ('port',                  rclpy.Parameter.Type.INTEGER),
                ('debug_mode',            rclpy.Parameter.Type.BOOL),
                ('image_reading_hz_rate', rclpy.Parameter.Type.DOUBLE),
            ])

    def connect_unrealcv(self, ip: str, port: int):
        """Establish TCP connection with Unreal Engine via UnrealCV."""
        self.get_logger().info(f'Trying to connect to UnrealCV at {ip}:{port}...')
        self.unreal_client = Client((ip, port), 'inet')
        
        while not self.unreal_client.isconnected():
            self.unreal_client.connect()
            time.sleep(1)
        self.get_logger().info('UnrealCV Connection Established!')

    def read_unreal_cameras(self):
        """Read RGB and Depth from Camera 3 and publish."""
        if not self.unreal_client or not self.unreal_client.isconnected():
            self.get_logger().warn("UnrealCV not connected. Skipping read.")
            return

        try:
            # Timestamp to sync the frames across both topics
            current_time = self.get_clock().now().to_msg()

            # --- Read and Publish Camera 3 (Center RGB) ---
            res_rgb = self.unreal_client.request('vget /camera/3/lit png')
            if res_rgb:
                img_rgb_np = read_png(res_rgb)
                msg_rgb = self.cv_bridge.cv2_to_imgmsg(img_rgb_np, encoding="rgba8")
                msg_rgb.header.stamp = current_time
                msg_rgb.header.frame_id = "unreal_camera_3"
                self._pub_rgb.publish(msg_rgb)

            # --- Read and Publish Camera 3 (Center Depth) ---
            res_depth = self.unreal_client.request('vget /camera/3/depth npy')
            if res_depth:
                depth_np = read_npy(res_depth)
                
                # Basic cleaning: Convert Infs/NaNs and clip range
                depth_np = np.nan_to_num(depth_np, posinf=1000.0, nan=0.0, neginf=0.0)
                depth_np = np.clip(depth_np, 0, 1000.0)
                
                # Ensure the data type is 32-bit float for ROS compatibility
                depth_np = depth_np.astype(np.float32)

                # Use 32FC1 encoding for single-channel float images
                msg_depth = self.cv_bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
                msg_depth.header.stamp = current_time
                msg_depth.header.frame_id = "unreal_camera_3"
                self._pub_depth.publish(msg_depth)

            if self.debug_mode:
                self.get_logger().info('Published Camera 3 RGB and Depth frames.')

        except Exception as e:
            self.get_logger().error(f"Error reading or publishing from UnrealCV: {e}")


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.debug_mode = self.get_parameter('debug_mode').value
        config_image_reading_hz_rate = self.get_parameter('image_reading_hz_rate').value
        config_ip = self.get_parameter('ip').value
        config_port = self.get_parameter('port').value

        # 1. Establish the TCP connection
        self.connect_unrealcv(config_ip, config_port)

        # 2. Define publishers for Camera 3
        self._pub_rgb   = self.create_lifecycle_publisher(RosImage, 'unreal/camera3/image_color', 10)
        self._pub_depth = self.create_lifecycle_publisher(RosImage, 'unreal/camera3/image_depth', 10)

        # 3. Create the timer
        self._timer = self.create_timer(1.0 / config_image_reading_hz_rate, self.read_unreal_cameras)

        return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)


    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub_rgb)
        self.destroy_publisher(self._pub_depth)
        
        if self.unreal_client:
            self.unreal_client.disconnect()
            
        return TransitionCallbackReturn.SUCCESS


    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        
        if self._timer:
            self.destroy_timer(self._timer)
        if self._pub_rgb:
            self.destroy_publisher(self._pub_rgb)
        if self._pub_depth:
            self.destroy_publisher(self._pub_depth)
            
        if self.unreal_client:
            self.unreal_client.disconnect()
            
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecycleUnrealCameras('lc_component_cameras')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()