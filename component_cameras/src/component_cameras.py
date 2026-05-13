"""
ROS 2 Lifecycle Node for Unreal Engine Camera Integration.

This module bridges simulated camera data from Unreal Engine (via UnrealCV) 
into standard ROS 2 image pipelines. It publishes synchronized streams for 
raw RGB, compressed RGB, 32-bit floating-point depth maps, and conditionally 
semantic object masks alongside a JSON string representing the object-to-color mapping.
"""

from typing import Optional
import time
import re
import json

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
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

    Subscribes to UnrealCV TCP endpoints, processes raw NumPy arrays to ensure 
    memory contiguity and standard ROS encodings, and dynamically handles 
    scene-wide semantic color maps and depth truncation based on runtime parameters.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecycleUnrealCameras node and declare ROS 2 parameters.

        Args:
            node_name (str): The designated name of the ROS 2 node.
        """
        super().__init__(node_name, **kwargs)
        
        # Core image publishers
        self._pub_rgb:            Optional[Publisher] = None
        self._pub_rgb_compressed: Optional[Publisher] = None
        self._pub_depth:          Optional[Publisher] = None
        self._pub_depth_vis:      Optional[Publisher] = None
        
        # Semantic segmentation publishers
        self._pub_mask:           Optional[Publisher] = None
        self._pub_mask_mapping:   Optional[Publisher] = None
        
        # Node utilities and state variables
        self._timer: Optional[Timer] = None
        self.unreal_client: Optional[Client] = None
        self.cv_bridge = CvBridge()
        
        # Cache for the scene object-to-color mapping (JSON formatted)
        self._cached_color_map_json: str = "{}"
        
        # Runtime configuration variables
        self.enable_object_mask: bool = False
        self.depth_max_distance: float = 1000.0
        self.debug_mode: bool = False

        # Declare ROS 2 Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip',                    rclpy.Parameter.Type.STRING),
                ('port',                  rclpy.Parameter.Type.INTEGER),
                ('camera_id',             rclpy.Parameter.Type.INTEGER),
                ('debug_mode',            rclpy.Parameter.Type.BOOL),
                ('image_reading_hz_rate', rclpy.Parameter.Type.DOUBLE),
                ('enable_object_mask',    rclpy.Parameter.Type.BOOL),
                ('depth_max_distance',    rclpy.Parameter.Type.DOUBLE),
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

    def _build_color_mapping(self) -> str:
        """
        Retrieves all scene objects and maps their names to their segmentation color.

        This operation requires hundreds of individual TCP queries to the Unreal Engine 
        API. It is executed strictly once during node configuration to prevent massive 
        CPU and network bottlenecking during the high-frequency read cycle.

        Returns:
            str: A JSON-serialized dictionary mapping object IDs to [R, G, B] lists.
        """
        self.get_logger().info('Downloading object color mapping from Unreal Engine... This may take a moment.')
        
        res_objects = self.unreal_client.request('vget /objects')
        if not res_objects:
            self.get_logger().warn('No objects found in scene. Semantic mapping will be empty.')
            return "{}"

        scene_objects = res_objects.split(' ')
        color_mapping = {}
        
        # Regex to parse UnrealCV's proprietary color string format: e.g., (R=255,G=128,B=0,A=255)
        color_regex = re.compile(r'\(R=(.*),G=(.*),B=(.*),A=(.*)\)')

        for obj_id in scene_objects:
            color_str = self.unreal_client.request(f'vget /object/{obj_id}/color')
            if color_str:
                match = color_regex.match(color_str)
                if match:
                    r = int(match.group(1))
                    g = int(match.group(2))
                    b = int(match.group(3))
                    # Storing as an RGB list array to facilitate clean JSON parsing downstream
                    color_mapping[obj_id] = [r, g, b]

        self.get_logger().info(f'Successfully mapped {len(color_mapping)} objects for semantic segmentation.')
        return json.dumps(color_mapping)

    def read_unreal_cameras(self) -> None:
        """
        Timer callback to fetch, process, and publish synchronized camera frames.

        Performs synchronization, data type casting, and memory realignment 
        to ensure safe serialization into ROS 2 DDS middleware. 
        Execution is halted if the node is not in the ACTIVE lifecycle state.
        """
        if not self.unreal_client or not self.unreal_client.isconnected():
            self.get_logger().warn("UnrealCV connection lost. Bypassing read cycle.")
            return

        try:
            # Generate a unified timestamp to guarantee synchronization across all published topics
            current_time = self.get_clock().now().to_msg()

            # ---------------------------------------------------------
            # Camera: RGB Processing & Publishing
            # ---------------------------------------------------------
            frame_id = f"unreal_camera_{self.camera_id}"
            res_rgb = self.unreal_client.request(f'vget /camera/{self.camera_id}/lit png')
            if res_rgb:
                img_rgb_np = read_png(res_rgb)
                
                # Cast payload to unsigned 8-bit integers for OpenCV/ROS compatibility
                img_rgb_np = img_rgb_np.astype(np.uint8)
                
                # Truncate the alpha channel to prevent rendering faults in standard ROS tools
                if img_rgb_np.shape[2] == 4:
                    img_rgb_np = img_rgb_np[:, :, :3]
                
                # Reallocate memory to ensure C-contiguous layout, preventing serialization crashes
                img_rgb_np = np.ascontiguousarray(img_rgb_np) 

                # Construct and publish uncompressed RGB payload
                msg_rgb = self.cv_bridge.cv2_to_imgmsg(img_rgb_np, encoding="rgb8")                
                msg_rgb.header.stamp = current_time
                msg_rgb.header.frame_id = frame_id
                self._pub_rgb.publish(msg_rgb)

                # Construct and publish JPEG compressed RGB payload
                img_bgr_np = cv2.cvtColor(img_rgb_np, cv2.COLOR_RGB2BGR)
                msg_rgb_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(img_bgr_np, dst_format='jpg')
                msg_rgb_compressed.header.stamp = current_time
                msg_rgb_compressed.header.frame_id = frame_id
                self._pub_rgb_compressed.publish(msg_rgb_compressed)

            # ---------------------------------------------------------
            # Camera: Depth Processing & Publishing
            # ---------------------------------------------------------
            res_depth = self.unreal_client.request(f'vget /camera/{self.camera_id}/depth npy')
            if res_depth:
                depth_np = read_npy(res_depth)
                
                # UnrealCV vget /camera/depth npy returns depth in millimetres
                # (float16 values; 65504 = float16 max, used for background/sky).
                # Convert mm → m and clamp to the configured maximum range.
                depth_np = np.nan_to_num(depth_np, nan=0.0,
                                         posinf=self.depth_max_distance * 1000.0,
                                         neginf=0.0)
                depth_np = depth_np / 1000.0
                depth_np = np.clip(depth_np, 0.0, self.depth_max_distance)
                
                # Cast to 32-bit floating point for standard 32FC1 ROS depth map encoding
                depth_np = depth_np.astype(np.float32)

                msg_depth = self.cv_bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
                msg_depth.header.stamp = current_time
                msg_depth.header.frame_id = frame_id
                self._pub_depth.publish(msg_depth)

                # Colourised visualisation — JET colormap so RViz shows a readable
                # gradient without manual min/max tuning. Near = blue, far = red.
                depth_norm = (depth_np / self.depth_max_distance * 255).astype(np.uint8)
                depth_coloured = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                depth_coloured = np.ascontiguousarray(depth_coloured)
                msg_depth_vis = self.cv_bridge.cv2_to_imgmsg(depth_coloured, encoding="bgr8")
                msg_depth_vis.header.stamp = current_time
                msg_depth_vis.header.frame_id = frame_id
                self._pub_depth_vis.publish(msg_depth_vis)

            # ---------------------------------------------------------
            # Camera: Semantic Object Mask & JSON Mapping (Conditional)
            # ---------------------------------------------------------
            if self.enable_object_mask:
                res_mask = self.unreal_client.request(f'vget /camera/{self.camera_id}/object_mask png')
                if res_mask:
                    img_mask_np = read_png(res_mask)
                    img_mask_np = img_mask_np.astype(np.uint8)
                    
                    # Enforce standard RGB formatting by dropping the alpha mask
                    if img_mask_np.shape[2] == 4:
                        img_mask_np = img_mask_np[:, :, :3]
                    img_mask_np = np.ascontiguousarray(img_mask_np)

                    # Publish visual mask image
                    msg_mask = self.cv_bridge.cv2_to_imgmsg(img_mask_np, encoding="rgb8")
                    msg_mask.header.stamp = current_time
                    msg_mask.header.frame_id = frame_id
                    self._pub_mask.publish(msg_mask)
                    
                    # Publish the cached JSON string payload corresponding to the colors in the mask
                    msg_map = String()
                    msg_map.data = self._cached_color_map_json
                    self._pub_mask_mapping.publish(msg_map)

            if self.debug_mode:
                self.get_logger().info('Synchronized broadcast completed.')

        except Exception as e:
            self.get_logger().error(f"UnrealCV integration fault during read/publish cycle: {e}")

    # =========================================================================
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        Initializes TCP sockets, parameters, and dynamic network publishers.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        # Read Parameters
        self.debug_mode           = self.get_parameter('debug_mode').value
        self.camera_id            = self.get_parameter('camera_id').value
        self.enable_object_mask   = self.get_parameter('enable_object_mask').value
        self.depth_max_distance   = self.get_parameter('depth_max_distance').value
        config_image_hz_rate      = self.get_parameter('image_reading_hz_rate').value
        config_ip                 = self.get_parameter('ip').value
        config_port               = self.get_parameter('port').value

        self.connect_unrealcv(config_ip, config_port)
        topic_base = f'unreal/camera{self.camera_id}'

        # 1. Allocate Base DDS endpoints (Publishers)
        self._pub_rgb            = self.create_lifecycle_publisher(RosImage, f'{topic_base}/image_color', 10)
        self._pub_rgb_compressed = self.create_lifecycle_publisher(CompressedImage, f'{topic_base}/image_color/compressed', 10)
        self._pub_depth          = self.create_lifecycle_publisher(RosImage, f'{topic_base}/image_depth', 10)
        self._pub_depth_vis      = self.create_lifecycle_publisher(RosImage, f'{topic_base}/image_depth/visualization', 10)

        # 2. Conditionally allocate Object Mask resources
        if self.enable_object_mask:
            self.get_logger().info("Object masking is ENABLED. Initializing semantic mapping...")
            self._pub_mask         = self.create_lifecycle_publisher(RosImage, f'{topic_base}/object_mask', 10)
            self._pub_mask_mapping = self.create_lifecycle_publisher(String, f'{topic_base}/object_mask_mapping', 10)
            self._cached_color_map_json = self._build_color_mapping()
        else:
            self.get_logger().info("Object masking is DISABLED. Semantic mapping bypassed.")

        # 3. Initialize high-frequency execution timer
        self._timer = self.create_timer(1.0 / config_image_hz_rate, self.read_unreal_cameras)

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
        Destroys existing publishers, timers, and active TCP connections to release system resources.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        
        if self._timer:
            self.destroy_timer(self._timer)
            
        if self._pub_rgb:
            self.destroy_publisher(self._pub_rgb)
        if self._pub_rgb_compressed:
            self.destroy_publisher(self._pub_rgb_compressed)
        if self._pub_depth:
            self.destroy_publisher(self._pub_depth)
        if self._pub_depth_vis:
            self.destroy_publisher(self._pub_depth_vis)

        # Conditionally destroy mask publishers
        if self._pub_mask:
            self.destroy_publisher(self._pub_mask)
        if self._pub_mask_mapping:
            self.destroy_publisher(self._pub_mask_mapping)
        
        if self.unreal_client:
            self.unreal_client.disconnect()
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node shutdown.
        Ensures graceful termination of hardware and network interfaces.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        # Cleanup logic is identical to on_cleanup here
        if self._timer:
            self.destroy_timer(self._timer)
            
        if self._pub_rgb:
            self.destroy_publisher(self._pub_rgb)
        if self._pub_rgb_compressed:
            self.destroy_publisher(self._pub_rgb_compressed)
        if self._pub_depth:
            self.destroy_publisher(self._pub_depth)
        if self._pub_depth_vis:
            self.destroy_publisher(self._pub_depth_vis)

        if self._pub_mask:
            self.destroy_publisher(self._pub_mask)
        if self._pub_mask_mapping:
            self.destroy_publisher(self._pub_mask_mapping)
            
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