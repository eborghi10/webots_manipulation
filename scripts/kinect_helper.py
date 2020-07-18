import math
import rospy
import numpy as np

from controller import RangeFinder, Camera
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


# https://github.com/HASHRobotics/lunar-env/blob/d99dc2452e0d8c9423dd2151c8ea3fcaa747ba67/controllers/pioneer_3at_controller/pioneer_3at_controller.py
class KinectHelper(object):
  def __init__(self, color_cam, range_cam, timestep):
    self.kinect_range = range_cam
    self.kinect_color = color_cam
    # Enable cameras
    self.kinect_range.enable(2 * timestep)
    self.kinect_color.enable(2 * timestep)
    # Init messages
    self.init_messages()
    # ROS publishers
    self.camera_rgb_pub = rospy.Publisher('rgb/image_raw', Image, queue_size=1)
    self.camera_info_rgb_pub = rospy.Publisher('rgb/camera_info', CameraInfo, queue_size=1)
    self.camera_depth_pub = rospy.Publisher('depth/image_raw', Image, queue_size=1)
    self.camera_info_depth_pub = rospy.Publisher('depth/camera_info', CameraInfo, queue_size=1)
    # OpenCV bridge
    self.bridge = CvBridge()

  def init_image(self, camera):
    msg = CameraInfo()
    msg.width = camera.getWidth()
    msg.height = camera.getHeight()
    f = camera.getWidth()/(2*math.tan(camera.getFov()/2))
    msg.K[0] = f
    msg.K[4] = f
    msg.K[2] = camera.getWidth()/2
    msg.K[5] = camera.getHeight()/2
    msg.K[8] = 1
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = [f, 0.0, camera.getWidth()/2, 0.0, 0.0, f, camera.getHeight()/2, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg

  def init_messages(self):
    self.info_rgb_msg = self.init_image(self.kinect_color)
    self.info_depth_msg = self.init_image(self.kinect_range)

  def broadcast_depth_image(self):
    time_now = rospy.Time.now()
    raw = self.kinect_range.getRangeImageArray()
    if raw is None: return
    img = np.transpose(np.array(raw, dtype=np.float32))
    image = self.bridge.cv2_to_imgmsg(img, "passthrough")
    image.header.stamp = time_now
    image.header.frame_id = "camera_depth_optical_frame"
    self.camera_depth_pub.publish(image)
    # Camera info
    self.info_depth_msg.header.stamp = time_now
    self.info_depth_msg.header.frame_id = "camera_depth_optical_frame"
    self.camera_info_depth_pub.publish(self.info_depth_msg)

  def broadcast_color_image(self):
    time_now = rospy.Time.now()
    raw = self.kinect_color.getImageArray()
    if raw is None: return
    img = np.transpose(np.array(raw, dtype=np.uint8), (1,0,2))
    image = self.bridge.cv2_to_imgmsg(img, "rgb8")
    image.header.stamp = time_now
    image.header.frame_id = "camera_rgb_optical_frame"
    self.camera_rgb_pub.publish(image)
    # Camera info
    self.info_rgb_msg.header.stamp = time_now
    self.info_rgb_msg.header.frame_id = "camera_rgb_optical_frame"
    self.camera_info_rgb_pub.publish(self.info_rgb_msg)
