import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


rospy.init_node("stream_only")
pub = rospy.Publisher("/image_raw", Image, queue_size = 1)
print("hi")
cam = cv2.VideoCapture(0)
# cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink")
print("done")
bridge = CvBridge()

while True:
    ret, frame = cam.read()
    print(frame.shape)
    msg = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
    pub.publish(msg)
    cv2.imshow("stream", frame)
    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        cam.release()
        break
