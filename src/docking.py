import rospy
from auto_docking.msg import TagInfo
from ptz_pkg.msg import PTZPosition
from geometry_msgs.msg import Twist



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
bot_cam_together = False
is_charging = False
# y_offset = 0.085
y_offset = 0.03

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
    global cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, bot_cam_together, phase_two, phase_three, is_charging
    print("camera face forward together with robot\n\n")
    cam_msg.pan.data = 90
    cam_pub.publish(cam_msg)
    rospy.sleep(1)
    cam_pub.publish(cam_msg)
    rospy.sleep(3)
    # print(cam_msg)
    bot_msg.linear.x = 0
    bot_msg.angular.z = 0
    bot_pub.publish(bot_msg)
    # only start docking after cam and bot are together
    is_docking = True
    bot_cam_together = True
    phase_one = True
    phase_two = False
    phase_three = False
    is_charging = False

def docking(event):
    global ty, cx, width, alpha, cam_pub, cam_msg, bot_pub, bot_msg, is_docking, phase_one, phase_two, phase_three, bot_cam_together, direction, is_charging, tag_visible
    if not is_docking:
        # print("not docking")
        return
    # print("is docking")
    if phase_one:
        # if tag_visible and (abs(cx - 0.5 * width) / width) < 0.03:
        if tag_visible and abs(ty) < 0.03:
            bot_msg.angular.z = 0
            print("phase 1      stoping robot before spinning camera\n")
            bot_pub.publish(bot_msg)
            rospy.sleep(1)

            if bot_cam_together:
                # robot at left, negative alpha, direction = 1, drive forward, camera spin left by 90 - |alpha|
                # robot at right, positive alpha, direction = -1, drive backward, camera spin left by 90 + |alpha|
                """
                add a counter here due to alpha jump pos/neg
                """
                direction = 1 if alpha < 0 else -1
                # camera spin left with increased pan
                print("=============================================")
                print("original pan (expect 90):", cam_msg.pan.data)
                print("alpha:", alpha)
                print("direction:", direction)
                cam_msg.pan.data += 90 - direction * abs(alpha)
                print("first cam spin pan:", cam_msg.pan.data)
                print("=============================================\n")
                bot_cam_together = False

            else:
                # spin camera to face left
                cam_msg.pan.data = 180
                phase_one = False
                phase_two = True
                # is_docking = False
                print("current alpha:", alpha)
                print("exiting phase 1, pan set to 180 (face left of robot)\n\n\n")

            cam_pub.publish(cam_msg)
            rospy.sleep(1)
            cam_pub.publish(cam_msg)
            rospy.sleep(3)
        else:
            # robot turn right when z < 0
            bot_msg.angular.z = -0.03 if tag_visible else -0.4
            bot_pub.publish(bot_msg)
        return

    # phase two
    # crab walk (only foward and backward) with cam ALWAYS 90 deg left of bot until is linned up with y offset
    # camera stay STILL, angular.z REMAIN 0 (set to 0 at the end of phase one already)
    # if alpha is too big, set phase_one = true and redo phase one
    if phase_two:
        if tag_visible:
            # MAY NEED TO CHANGE THIS BASED ON TESTING
            # if abs(alpha) < 20:


            # when camera is facing left 90deg of the robot, rotation center is 8.5cm (0.085m) to the right of optical camera
            if abs(ty + y_offset) < 0.03:
                bot_msg.linear.x = 0
                bot_pub.publish(bot_msg)
                print("bot stop")
                cam_msg.pan.data = 270
                cam_pub.publish(cam_msg)
                rospy.sleep(1)
                cam_pub.publish(cam_msg)
                rospy.sleep(3)
                print("camera face backward")
                phase_two = False
                phase_three = True
                is_docking = False
                print("phase 2 finished successfully, robot rotation center stop on normal line\n\n\n")
            else:
                # move slowly when tag is in picture
                direction = 1 if (ty + y_offset) < 0 else -1
                speed = 0.04 if abs(ty + y_offset) < 0.3 else 0.2
                # print(speed)
                bot_msg.linear.x = direction * speed


            # else:
            #     # |alpha| is too large, redo phase one
            #     print("alpha:", alpha)
            #     print("exit phase 2, |alpha| is too large, redo phase_one now\n\n\n")
            #     start_docking()
            #     return
                # phase_one = True
                # phase_two = False
                # bot_msg.linear.x = 0
        else:
            # drive blind based on previous direction, hope to dirve parallel to tag plane
            # robot at left, negative alpha, direction = 1, drive forward
            # robot at right, positive alpha, direction = -1, drive backward
            """
            add abort feature, is_docking=False if move further than |ty| withought seeing tag, ry not needed, only 8.5cm difference
            """
            bot_msg.linear.x = direction * 0.5
        # if bot_msg.linear.x > 0:
        #     print("forward")
        # elif bot_msg.linear.x < 0:
        #     print("back")
        # else:
        #     print("000000000000000")
        bot_pub.publish(bot_msg)
        return


    if phase_three:
        """"
        add abort feature, if move too far, further than tx
        or have camera face down a bit, and terminate when tag is too big
        is_charging is never updated, need to physically stop the process
        """
        if is_charging:
            bot_msg.linear.x = 0
            bot_msg.angular.z = 0
            phase_three = False
            is_docking = False
            print("robot is charging, docking complete\n\n\n")
        elif tag_visible:
            if abs(alpha) < 3:
                # wireless reciever on the back
                bot_msg.linear.x = -0.2
                bot_msg.angular.z = 0
            else:
                # robot spin right slowly, tag in vision
                bot_msg.linear.x = 0
                bot_msg.angular.z = -0.05
        else:
            # robot spin right fast, tag NOT in vision
            bot_msg.linear.x = 0
            bot_msg.angular.z = -0.4

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
print("start rospy spin\n\n")
rospy.spin()