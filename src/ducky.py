import cv2
# import rospy
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
import yaml
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R


# rospy.init_node("cv_to_ros")
# raw_pub = rospy.Publisher("/image_raw", Image, queue_size=1)
# rect_color_pub = rospy.Publisher("/image_rect_color", Image, queue_size=1)
# rect_pub = rospy.Publisher("/image_rect", Image, queue_size=1)
# info_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=1)

# bridge = CvBridge()

# cam = cv2.VideoCapture(0)
cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink")

# with open("resources/webcam_calibration.yaml") as f:
with open("resources/ptz_calibration.yaml") as f:
    calibration = yaml.safe_load(f)
    f.close()

# /camera_info
# info_msg = CameraInfo()
# info_msg.header.frame_id = "ptz"
# info_msg.width = calibration["image_width"]
# info_msg.height = calibration["image_height"]
# info_msg.distortion_model = calibration["distortion_model"]
# info_msg.K = calibration["camera_matrix"]["data"]
# info_msg.D = calibration["distortion_coefficients"]["data"]
# info_msg.R = calibration["rectification_matrix"]["data"]
# info_msg.P = calibration["projection_matrix"]["data"]



# ros calibration pkg uses cv2.getOptimalNewCameraMatrix(alpha = 0) to calculate projection_mtx
# don't need to re-calculate, just read from yml
# new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_cef, (w,h), 0, (w,h))
w = calibration["image_width"]
h = calibration["image_height"]
dist_cef = np.array(calibration["distortion_coefficients"]["data"])
cam_mtx = np.array(calibration["camera_matrix"]["data"]).reshape((3, 3))
projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
new_cam_mtx = np.delete(projection_mtx, -1, axis=1)
new_cam_params = (new_cam_mtx[0,0], new_cam_mtx[1,1], new_cam_mtx[0,2], new_cam_mtx[1,2])


tag_size = 0.166
at_detector = Detector(families='tag36h11',
                            nthreads=4,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

while True:
    # rectify
    ret, raw_frame = cam.read()
    rect_color_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)
    rect_frame = cv2.cvtColor(rect_color_frame, cv2.COLOR_BGR2GRAY)

    # convert to ros msg format
    # raw_msg = bridge.cv2_to_imgmsg(raw_frame, encoding = "bgr8")
    # rect_color_msg = bridge.cv2_to_imgmsg(rect_color_frame, encoding = "bgr8")
    # rect_msg = bridge.cv2_to_imgmsg(rect_frame, encoding = "bgr8")

    # time for header
    # now = rospy.Time.now()
    # info_msg.header.stamp = now
    # raw_msg.header.stamp = now
    # rect_color_msg.header.stamp = now
    # rect_msg.header.stamp = now

    # publish to topics
    # raw_pub.publish(raw_msg)
    # rect_color_pub.publish(rect_color_msg)
    # rect_pub.publish(rect_msg)
    # info_pub.publish(info_msg)

    # display using cv
    # cv2.imshow("raw", raw_frame)
    cv2.imshow("rect_color", rect_color_frame)
    # cv2.imshow("rect", rect_frame)

    # use new_cam_params since rect_frame is undistorted
    tags = at_detector.detect(rect_frame, True, new_cam_params, tag_size)
    for tag in tags:
        # print(tag)
        r = R.from_matrix(tag.pose_R)
        euler = r.as_euler("zxy", degrees=True)
        x = tag.pose_t[2][0]
        y = -1 * tag.pose_t[0][0]
        z = -1 * tag.pose_t[1][0]
        print(x, y, z)
        print(euler)
        print("--------------------------------------")

    if cv2.waitKey(1) == 27:
        break
