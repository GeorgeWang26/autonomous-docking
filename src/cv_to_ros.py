# https://stackoverflow.com/questions/66895102/how-to-apply-distortion-on-an-image-using-opencv-or-any-other-library
# https://blog.csdn.net/u010949023/article/details/116597057
# https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-do-in-opencv

# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.

import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import numpy as np

rospy.init_node("cv_to_ros")
# raw_pub = rospy.Publisher("/image_raw", Image, queue_size=1)
# rect_color_pub = rospy.Publisher("/image_rect_color", Image, queue_size=1)
rect_pub = rospy.Publisher("/image_rect", Image, queue_size=1)

bridge = CvBridge()

# cam = cv2.VideoCapture(0)
cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink")

# with open("resources/webcam_calibration.yaml") as f:
with open("resources/ptz_calibration.yaml") as f:
    calibration = yaml.safe_load(f)
    f.close()


# ros calibration pkg uses cv2.getOptimalNewCameraMatrix(alpha = 0) to calculate projection_mtx
# don't need to re-calculate, just read from yml
# new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_cef, (w,h), 0, (w,h))
w = calibration["image_width"]
h = calibration["image_height"]
dist_cef = np.array(calibration["distortion_coefficients"]["data"])
cam_mtx = np.array(calibration["camera_matrix"]["data"]).reshape((3, 3))
projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
new_cam_mtx = np.delete(projection_mtx, -1, axis=1)
# new_cam_params = (new_cam_mtx[0,0], new_cam_mtx[1,1], new_cam_mtx[0,2], new_cam_mtx[1,2])


while True:
    # rectify
    ret, raw_frame = cam.read()
    rect_color_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)
    rect_frame = cv2.cvtColor(rect_color_frame, cv2.COLOR_BGR2GRAY)

    # convert to ros msg format
    # raw_msg = bridge.cv2_to_imgmsg(raw_frame, encoding = "bgr8")
    # rect_color_msg = bridge.cv2_to_imgmsg(rect_color_frame, encoding = "bgr8")
    rect_msg = bridge.cv2_to_imgmsg(rect_frame, encoding = "mono8")

    # publish to topics
    # raw_pub.publish(raw_msg)
    # rect_color_pub.publish(rect_color_msg)
    rect_pub.publish(rect_msg)
    # info_pub.publish(info_msg)

    # display using cv
    # cv2.imshow("raw", raw_frame)
    cv2.imshow("rect_color", rect_color_frame)
    # cv2.imshow("rect", rect_frame)
    
    if cv2.waitKey(1) == 27:
        break