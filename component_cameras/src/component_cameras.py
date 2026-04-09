"""
ROS 2 Lifecycle Node for Unreal Engine Camera Integration.

This module bridges simulated camera data from Unreal Engine (via UnrealCV) 
into standard ROS 2 image pipelines, publishing both raw and compressed RGB streams, 
as well as 32-bit floating-point depth maps.
"""

from typing import Optional
import time

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from unrealcv import Client
from unrealcv.util import read_png, read_npy

import numpy as np
import cv2  
import warnings

warnings.filterwarnings("ignore")


class LifecycleUnrealCameras(rclpy.lifecycle.Node):
    """
    Lifecycle node responsible for polling and publishing UnrealCV camera data.

    Subscribes to UnrealCV TCP endpoints, processes the raw NumPy arrays to ensure 
    memory contiguity and standard ROS encodings, and broadcasts the data across 
    designated image topics.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecycleUnrealCameras node and declare ROS 2 parameters.

        Args:
            node_name (str): The designated name of the ROS 2 node.
        """
        super().__init__(node_name, **kwargs)
        
        # Publisher handles
        self._pub_rgb:            Optional[Publisher] = None
        self._pub_rgb_compressed: Optional[Publisher] = None
        self._pub_depth:          Optional[Publisher] = None
        
        # Node utilities
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

    def connect_unrealcv(self, ip: str, port: int) -> None:
        """
        Establish a blocking TCP connection to the UnrealCV server.

        Args:
            ip (str): The IPv4 address of the Unreal Engine host instance.
            port (int): The designated UnrealCV communication port.
        """
        self.get_logger().info(f'Attempting connection to UnrealCV server at {ip}:{port}...')
        self.unreal_client = Client((ip, port), 'inet')
        
        while not self.unreal_client.isconnected():
            self.unreal_client.connect()
            time.sleep(1.0)
            
        self.get_logger().info('UnrealCV TCP connection established successfully.')

    def read_unreal_cameras(self) -> None:
        """
        Timer callback to fetch, process, and publish camera frames.

        Performs synchronization, data type casting, and memory realignment 
        to ensure safe serialization into ROS 2 DDS middleware. 
        Execution is halted if the node is not in the ACTIVE lifecycle state.
        """
        if not self.unreal_client or not self.unreal_client.isconnected():
            self.get_logger().warn("UnrealCV connection lost. Bypassing read cycle.")
            return

        try:
            # Synchronize timestamp for all resulting image messages
            current_time = self.get_clock().now().to_msg()

            # ---------------------------------------------------------
            # Camera 3: RGB Processing & Publishing
            # ---------------------------------------------------------
            res_rgb = self.unreal_client.request('vget /camera/3/lit png')
            if res_rgb:
                img_rgb_np = read_png(res_rgb)
                
                # Cast payload to unsigned 8-bit integers for OpenCV/ROS compatibility
                img_rgb_np = img_rgb_np.astype(np.uint8)
                
                # Truncate the alpha channel to prevent rendering faults in RQT and standard tools
                if img_rgb_np.shape[2] == 4:
                    img_rgb_np = img_rgb_np[:, :, :3]
                    
                # Reallocate memory to ensure C-contiguous layout, preventing serialization crashes
                img_rgb_np = np.ascontiguousarray(img_rgb_np) 

                # Publish uncompressed RGB payload
                msg_rgb = self.cv_bridge.cv2_to_imgmsg(img_rgb_np, encoding="rgb8")                
                msg_rgb.header.stamp = current_time
                msg_rgb.header.frame_id = "unreal_camera_3"
                self._pub_rgb.publish(msg_rgb)
                if self.debug_mode:
                    self.get_logger().info('Broadcasted RGB image.')

                # Publish JPEG compressed RGB payload
                # Note: OpenCV compression implicitly requires BGR formatting
                img_bgr_np = cv2.cvtColor(img_rgb_np, cv2.COLOR_RGB2BGR)
                msg_rgb_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(img_bgr_np, dst_format='jpg')
                msg_rgb_compressed.header.stamp = current_time
                msg_rgb_compressed.header.frame_id = "unreal_camera_3"
                self._pub_rgb_compressed.publish(msg_rgb_compressed)
                if self.debug_mode:
                    self.get_logger().info('Broadcasted RGB compressed image.')

            # ---------------------------------------------------------
            # Camera 3: Depth Processing & Publishing
            # ---------------------------------------------------------
            res_depth = self.unreal_client.request('vget /camera/3/depth npy')
            if res_depth:
                depth_np = read_npy(res_depth)
                
                # Sanitize invalid depth values (NaN/Inf) and clamp to predefined 10m range
                depth_np = np.nan_to_num(depth_np, posinf=1000.0, nan=0.0, neginf=0.0)
                depth_np = np.clip(depth_np, 0, 1000.0)
                
                # Cast to 32-bit floating point for standard 32FC1 depth map encoding
                depth_np = depth_np.astype(np.float32)

                msg_depth = self.cv_bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
                msg_depth.header.stamp = current_time
                msg_depth.header.frame_id = "unreal_camera_3"
                self._pub_depth.publish(msg_depth)
                if self.debug_mode:
                    self.get_logger().info('Broadcasted Depth image.')

        except Exception as e:
            self.get_logger().error(f"UnrealCV integration fault during read/publish cycle: {e}")

    # =========================================================================
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        Initializes TCP sockets, parameters, publishers, and read timers.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        self.debug_mode = self.get_parameter('debug_mode').value
        config_image_reading_hz_rate = self.get_parameter('image_reading_hz_rate').value
        config_ip = self.get_parameter('ip').value
        config_port = self.get_parameter('port').value

        self.connect_unrealcv(config_ip, config_port)

        self._pub_rgb            = self.create_lifecycle_publisher(RosImage, 'unreal/camera3/image_color', 10)
        self._pub_rgb_compressed = self.create_lifecycle_publisher(CompressedImage, 'unreal/camera3/image_color/compressed', 10)
        self._pub_depth          = self.create_lifecycle_publisher(RosImage, 'unreal/camera3/image_depth', 10)

        self._timer = self.create_timer(1.0 / config_image_reading_hz_rate, self.read_unreal_cameras)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Transition callback for node activation."""
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Transition callback for node deactivation."""
        self.get_logger().info("Lifecycle state transition: DEACTIVATED.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node cleanup.
        Destroys existing publishers, timers, and active TCP connections to free memory.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub_rgb)
        self.destroy_publisher(self._pub_rgb_compressed)
        self.destroy_publisher(self._pub_depth)
        
        if self.unreal_client:
            self.unreal_client.disconnect()
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node shutdown.
        Ensures graceful termination of hardware and network interfaces.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        if self._timer:
            self.destroy_timer(self._timer)
        if self._pub_rgb:
            self.destroy_publisher(self._pub_rgb)
        if self._pub_rgb_compressed:
            self.destroy_publisher(self._pub_rgb_compressed)
        if self._pub_depth:
            self.destroy_publisher(self._pub_depth)
            
        if self.unreal_client:
            self.unreal_client.disconnect()
            
        return TransitionCallbackReturn.SUCCESS


def main():
    """
    Entry point for the executable. Initializes the ROS 2 context, 
    allocates a multithreaded executor, and spins the node.
    """
    rclpy.init()

    # Employ a MultiThreadedExecutor to segregate lifecycle API polling from blocking I/O calls
    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node  = LifecycleUnrealCameras('lc_component_cameras')
    executor.add_node(lc_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()