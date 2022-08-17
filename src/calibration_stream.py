import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("calibration_stream")
stream = rospy.Publisher("/image", Image, queue_size = 1)
bridge = CvBridge()

# webcam
cam = cv2.VideoCapture(0)
# jetson
# cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink drop=true")
# non-jetson
# cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true")

while True:
    ret, frame = cam.read()
    msg = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
    stream.publish(msg)