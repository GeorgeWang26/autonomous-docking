from operator import truediv
from time import time

from cv2 import phase
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import numpy as np
# import numba
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R
from ptz_pkg.msg import PTZPosition
from geometry_msgs.msg import Twist
import time
import math

rx = 0
ry = 0
cx0 = 0
cy0 = 0
x0 = 0
y0 = 0
alpha = 0
direction = 0
tag_visible = False
is_docking = False
phase_one = False
first_time = False
bot_cam_together = False

bridge = CvBridge()

rospy.init_node("docking")
cam_pub = rospy.Publisher("/ptz/move", PTZPosition, queue_size=1)
bot_pub = rospy.Publisher("/vel_mux/cmd_vel", Twist, queue_size=1)

bot_msg = Twist()

cam_msg = PTZPosition()
cam_msg.tilt.data = 90
cam_msg.degrees.data = True

# with open("resources/webcam_calibration.yaml") as f:
with open("resources/ptz_calibration.yaml") as f:
    calibration = yaml.safe_load(f)
    f.close()

# ros calibration pkg uses cv2.getOptimalNewCameraMatrix(alpha = 0) to calculate projection_mtx
# don't need to re-calculate, just read from yml
# new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_cef, (w,h), 0, (w,h))
w = calibration["image_width"]
h = calibration["image_height"]
projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
new_cam_mtx = np.delete(projection_mtx, -1, axis=1)
new_cam_params = (new_cam_mtx[0,0], new_cam_mtx[1,1], new_cam_mtx[0,2], new_cam_mtx[1,2])

tag_size = 0.166
at_detector = Detector(families='tag36h11',
                            nthreads=12,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)


def detect_tag(msg):
    global x0, y0, cx0, cy0, alpha, tag_visible
    rect_frame = bridge.imgmsg_to_cv2(msg, "mono8")
    tags = at_detector.detect(rect_frame, True, new_cam_params, tag_size)
    """
    maybe will cause problem with split second false?
    """
    tag_visible = False
    for tag in tags:
        if tag.tag_id == "0":
            tag.visible = True
            # in cv frame, top left (0,0), horizontal is x, vertical is y
            cx0 = tag.center[0]
            cy0 = tag.center[1]
            # in ros frame, x is foward, y is left, z is up
            x0 = tag.pose_t[2][0]
            y0 = -1 * tag.pose_t[0][0]
            z = -1 * tag.pose_t[1][0]
            # angle (-90 to 90) between heading of camera and heading of tag
            r = R.from_matrix(tag.pose_R)
            euler = r.as_euler("zxy", degrees=True)
            alpha = euler[2]
            print(cx0, cy0)
            print(x0, y0, z)
            print(alpha, euler)
            print("--------------------------------------")


rect_sub = rospy.Subscriber("/image_rect", Image, detect_tag, queue_size=1)


def docking(event):
    global y0, cx0, w, alpha, cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, first_time, bot_cam_together, direction
    if not is_docking:
        print("out")
        return
    if phase_one:
        if tag_visible and (0.45 * w < cx0) and (cx0 < 0.55 * w):
            if bot_msg.angular.z == 0:
                if bot_cam_together:
                    """
                    ASSUMING when facing tag, and robot is on right side of tag, alpha > 0, WANT 90 +++ |alpha|
                    """
                    # to left spin camera, increase pan angle
                    cam_msg.pan.data += 90 + (alpha > 0) * math.abs(alpha)
                    # used for phase two, in case tag is not visible
                    """
                    ASSUMING when facing tag, and robot is on right side of tag, alpha > 0, WANT -1
                    so robot backoff -1 when at right, positive alpha
                    and dirve forward 1 when at left, negative alpha
                    """
                    direction = alpha < 0
                    bot_cam_together = False
                else:
                    # spin camera to face left
                    cam_msg.pan.data = 180
                    phase_one = False
                    phase_two = True
                cam_pub.publish(cam_msg)
            else:
                # tag in center, stop robot spin
                bot_msg.angular.z = 0
                bot_pub.publish(bot_msg)
            # wait for cam and/or bot spin to finish before detect tag again
            time.sleep(3)
        else:
            """
            ASSUMING robot turn RIGHT when z > 0
            report error if cant find tag after 360
            """
            bot_msg.angular.z = 0.2
            bot_pub.publish(bot_msg)
        return

    # phase two
    # crab walk with cam ALWAYS 90 deg left of bot until is linned up with y offset
    # camera stay STILL
    if phase_two:
        if tag_visible:
            if math.abs(alpha) < 5:
                if y0 < 0.1:
                    cam_msg.pan.data = 270
                    cam_pub.publish(cam_msg)
                    bot_msg.linear.x = 0
                    bot_msg.angular.z = 0
                    """
                    remove this
                    """
                    is_docking = False
                # y0 > 0 => tag is on left side of camera => robot need to drive backward => direction = -1
                # y0 < 0 => tag is on right side of camera => robot need to drive forward => direction = 1
                direction = -1 * y0
                bot_msg.linear.x = direction * 0.3
                bot_msg.angular.z = 0
            else:
                # tag is in vision, spin robot to make |alpha| < 5
                bot_msg.linear = 0
                bot_msg.angular = 0.2
        else:
            # drive blind based on previous direction, hope to dirve parallel to tag plane
            bot_msg.linear = direction * 0.3
            bot_msg.angular = 0
        bot_pub.publish(bot_msg)
        return

"""
use a service call to active docking
"""
def start_docking():
    global cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, first_time, bot_cam_together
    cam_msg.pan.data = 90
    cam_pub.publish(cam_msg)

    bot_msg.linear.x = 0
    bot_msg.angular.z = 0
    bot_pub.publish(bot_msg)
    
    time.sleep(3)
    # only start docking after cam and bot are together
    is_docking = True
    phase_one = True
    first_time = False
    bot_cam_together = True




# 20hz cycle
# time.sleep() inside the timer callback does not affect the main cycle such as tag detection
rospy.Timer(rospy.Duration(0.05), docking)



"""
for testing purpose
"""
start_docking()

rospy.spin()