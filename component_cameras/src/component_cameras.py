from typing import Optional
import rclpy
import rclpy.lifecycle

from custom_msgs.msg import FusionMatrix
from common_libs.depth_transform import CameraTransform, MergedImage
from common_libs.utils import Stack
from .utils import multiImg
from std_msgs.msg import MultiArrayDimension
from PIL import Image

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")


class LifecycleUnrealCameras(rclpy.lifecycle.Node):

  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)
    
    self._pub              : Optional[rclpy.lifecycle.Publisher] = None
    self._timer            : Optional[rclpy.timer.Timer]         = None

    self.declare_parameters(
      namespace='',
      parameters=[
        ('ip',                    rclpy.Parameter.Type.STRING),
        ('port',                  rclpy.Parameter.Type.INTEGER),
        ('debug_mode',            rclpy.Parameter.Type.BOOL),
        ('image_reading_hz_rate', rclpy.Parameter.Type.DOUBLE),
        ])


  def read_unreal_cameras(self):
    """Read images from Unreal cameras."""
    
    multi_msg = None

    # Filling aligned msg to send
    multi_msg = FusionMatrix()

      
    if multi_msg is not None:
      self.get_logger().info(f'Publishing multi array in topic, with seq: {multi_msg.seq}.')
      self._pub.publish(multi_msg)

      # Check if stack is bigger than 10 images to clean it
      if len(self._input_img_stack) > 10:
        # We keep the most recent img
        self._input_img_stack.clear(num_to_keep = 1)


  def on_configure(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Configure the node, after a configuring transition is requested.
    """
    self.get_logger().info("on_configure() is called.")

    self.debug_mode = self.get_parameter('debug_mode').value

    # Extracting params
    config_image_reading_hz_rate   = self.get_parameter('image_reading_hz_rate').value
    
    # Global size parameters
    self.config_ip    = self.get_parameter('ip').value
    self.config_port  = self.get_parameter('port').value

    # Defining subscriber and publisher
    self._pub         = self.create_lifecycle_publisher(FusionMatrix, config_image_reading_hz_rate, 10)
    self._timer       = self.create_timer(1 / config_image_reading_hz_rate, self.readUnrealCameras)

    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS



  def on_activate(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    # Log, only for demo purposes
    self.get_logger().info("on_activate() is called.")

    return super().on_activate(state)
  


  def on_deactivate(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    # Log, only for demo purposes
    self.get_logger().info("on_deactivate() is called.")

    return super().on_deactivate(state)



  def on_cleanup(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Cleanup the node, after a cleaning-up transition is requested.
    """

    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)

    self.get_logger().info('on_cleanup() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS



  def on_shutdown(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Shutdown the node, after a shutting-down transition is requested.
    """
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)

    self.get_logger().info('on_shutdown() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS


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