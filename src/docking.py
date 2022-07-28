import rospy
from auto_docking.msg import TagInfo
from ptz_pkg.msg import PTZPosition
from geometry_msgs.msg import Twist
import time



width = 0
height = 0
rx = 0
ry = 0
cx = 0
cy = 0
tx = 0
ty = 0
alpha = 0
direction = 0
tag_visible = False
is_docking = False
phase_one = False
first_time = False
bot_cam_together = False
is_charging = False

bot_msg = Twist()

cam_msg = PTZPosition()
cam_msg.tilt.data = 90
cam_msg.degrees.data = True




def tag_update(msg):
    global tx, ty, cx, cy, alpha, tag_visible, width
    if msg.family == "":
        tag_visible = False
        return
    width = msg.width
    tx = msg.tx
    ty = msg.ty
    cx = msg.cx
    cy = msg.cy
    alpha = msg.yaw
    tag_visible = True


"""
use a service call to active docking
"""
def start_docking():
    global cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, first_time, bot_cam_together, phase_two, phase_three, is_charging
    cam_msg.pan.data = 90
    cam_pub.publish(cam_msg)
    rospy.sleep(1)
    cam_pub.publish(cam_msg)
    print(cam_msg)
    bot_msg.linear.x = 0
    bot_msg.angular.z = 0
    bot_pub.publish(bot_msg)
    print("start docking")
    rospy.sleep(3)
    print("camera face forward together with robot")
    # only start docking after cam and bot are together
    is_docking = True
    phase_one = True
    first_time = False
    bot_cam_together = True
    phase_two = False
    phase_three = False
    is_charging = False

def docking(event):
    global ty, cx, width, alpha, cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, phase_two, phase_three, first_time, bot_cam_together, direction, is_charging
    if not is_docking:
        # print("not docking")
        return
    # print("is docking")
    if phase_one:
        if tag_visible and (0.45 * width < cx) and (cx < 0.55 * width):
            if bot_msg.angular.z == 0:
                if bot_cam_together:
                    # when facing tag, and robot is on right side of tag, alpha > 0
                    # WANT 90 + |alpha| to left spin camera, increase pan angle
                    cam_msg.pan.data += 90 + (alpha > 0) * abs(alpha)
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
                print("moving camera in phase 1")
                cam_pub.publish(cam_msg)
            else:
                # tag in center, stop robot spin
                bot_msg.angular.z = 0
                bot_pub.publish(bot_msg)
            # wait for cam and/or bot spin to finish before detect tag again
            time.sleep(3)
        else:
            # robot turn right when z < 0
            print("spin robot")
            bot_msg.angular.z = -0.2
            bot_pub.publish(bot_msg)
            # rospy.sleep(1)
            # bot_pub.publish(bot_msg)
        return

    # phase two
    # crab walk with cam ALWAYS 90 deg left of bot until is linned up with y offset
    # camera stay STILL
    if phase_two:
        if tag_visible:
            if abs(alpha) < 5:
                """
                add abort feature, is_docking=False if move further than |ry|
                """
                if ty < 0.1:
                    cam_msg.pan.data = 270
                    cam_pub.publish(cam_msg)
                    bot_msg.linear.x = 0
                    bot_msg.angular.z = 0
                    phase_two = False
                    phase_three = True
                else:
                    # y0 > 0 => tag is on left side of camera => robot need to drive backward => direction = -1
                    # y0 < 0 => tag is on right side of camera => robot need to drive forward => direction = 1
                    direction = -1 * ty
                    bot_msg.linear.x = direction * 0.3
                    bot_msg.angular.z = 0
            else:
                # tag is in vision, spin robot to make |alpha| < 5
                bot_msg.linear.x = 0
                bot_msg.angular.z = 0.2
        else:
            # drive blind based on previous direction, hope to dirve parallel to tag plane
            bot_msg.linear.x = direction * 0.3
            bot_msg.angular.z = 0
        bot_pub.publish(bot_msg)
        return

    if phase_three:
        """"
        is_charging is never updated, need to physically stop the process
        also stop if move too far, further than x0
        """
        if is_charging:
            bot_msg.linear.x = 0
            bot_msg.angular.z = 0
            phase_three = False
            is_docking = False
        elif tag_visible and abs(alpha) < 5:
            # wireless reciever on the back
            bot_msg.linear.x = -0.2
            bot_msg.angular.z = 0
        else:
            bot_msg.linear.x = 0
            bot_msg.angular.z = -0.2
        bot_pub.publish(bot_msg)
        return




rospy.init_node("docking")
cam_pub = rospy.Publisher("/ptz/move", PTZPosition, queue_size=1)
bot_pub = rospy.Publisher("/vel_mux/cmd_vel", Twist, queue_size=1)
tag_info_sub = rospy.Subscriber("/apriltag_detection", TagInfo, tag_update, queue_size=1)

# 20hz cycle
# time.sleep() inside the timer callback does not affect the main cycle such as tag detection
rospy.Timer(rospy.Duration(0.05), docking)


"""
for testing purpose, start docking manually
"""
start_docking()
print("start spin")
rospy.spin()