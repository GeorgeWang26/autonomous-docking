import rospy
from sensor_msg.msg import Image
import cv2

def callback(msg):
    frame = cv2.imdecode(msg.data, cv2.IMREAD_GRAYSCALE)
    cv2.imshow(frame)
    cv2.waitKey(1)

rospy.init_node("decode_ros")
sub = rospy.Subscriber("/image_encoded", Image, callback, queue_size = 1)