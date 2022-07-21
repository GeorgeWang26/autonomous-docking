import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

bridge = CvBridge()
def callback(msg):
    # frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = bridge.imgmsg_to_cv2(msg, "mono8")
    cv2.imshow("ros_to_cv", frame)
    cv2.waitKey(1)
rospy.init_node("ros_to_cv")
# sub = rospy.Subscriber("/image_raw", Image, callback)
sub = rospy.Subscriber("/image_rect", Image, callback)
rospy.spin()