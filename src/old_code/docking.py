import rospy
from auto_docking.msg import TagInfo
from ptz_pkg.msg import PTZPosition
from geometry_msgs.msg import Twist


tag_visible = False
width = 0
cx = 0
tx = 0
ty = 0
alpha = 0
alpha_pos_count = 0
alpha_neg_count = 0
alpha_move = []
t_smooth_lock = False
alpha_smooth_lock = False
first_visible = False

is_charging = False
is_docking = False
bot_cam_together = False
direction = 0
wait_alpha = True

phase_one = False
phase_two = False
phase_two_half = False
phase_three = False

bot_msg = Twist()

cam_msg = PTZPosition()
cam_msg.tilt.data = 90
cam_msg.degrees.data = True


def tag_update(msg):
    global tag_visible, width, cx, tx, ty, alpha, alpha_pos_count, alpha_neg_count, alpha_move, t_smooth_lock, alpha_smooth_lock, first_visible
    if msg.family == "":
        tag_visible = False
        alpha_move = []
        first_visible = True
        return
    width = msg.width
    cx = msg.cx

    if t_smooth_lock:
        if first_visible:
            tx = msg.tx
            ty = msg.ty
        else:
            if abs(tx - msg.tx) < 0.3:
                tx = msg.tx
            if abs(ty - msg.ty) < 0.3:
                ty = msg.ty
    else:
        tx = msg.tx
        ty = msg.ty
    # # ONLY set first_visible to false after alpha_smooth_lock, because it is used in two places
    # if alpha_smooth_lock:
    #     if first_visible or abs(alpha - msg.yaw) < 5:
    #         first_visible = False
    #         yaw = msg.yaw
    #         if alpha > 0:
    #             alpha_pos_count += 1
    #         else:
    #             alpha_neg_count += 1

    #         alpha_move.append(yaw)
    #         alpha_move_len = len(alpha_move)
    #         if alpha_move_len > 5:
    #             alpha_move.pop(0)
    #             alpha_move_len -= 1
    #         alpha_sum = 0
    #         for val in alpha_move:
    #             alpha_sum += val
    #         alpha = alpha_sum / alpha_move_len
    # else:
    #     yaw = msg.yaw
    #     if alpha > 0:
    #         alpha_pos_count += 1
    #     else:
    #         alpha_neg_count += 1

    #     alpha_move.append(yaw)
    #     alpha_move_len = len(alpha_move)
    #     if alpha_move_len > 5:
    #         alpha_move.pop(0)
    #         alpha_move_len -= 1
    #     alpha_sum = 0
    #     for val in alpha_move:
    #         alpha_sum += val
    #     alpha = alpha_sum / alpha_move_len
    
    yaw = msg.yaw
    if alpha > 0:
        alpha_pos_count += 1
    else:
        alpha_neg_count += 1

    alpha_move.append(yaw)
    alpha_move_len = len(alpha_move)
    if alpha_move_len > 5:
        alpha_move.pop(0)
        alpha_move_len -= 1
    alpha_sum = 0
    for val in alpha_move:
        alpha_sum += val
    alpha = alpha_sum / alpha_move_len


    first_visible = False
    # only set tag_visible=True after all status are updated
    tag_visible = True




def start_docking():
    global cam_pub, cam_msg, bot_pub, bot_msg, is_charging, is_docking, bot_cam_together, wait_alpha, \
        phase_one, phase_two, phase_two_half, phase_three
    print("camera face forward together with robot\n")
    cam_msg.pan.data = 90
    cam_pub.publish(cam_msg)
    rospy.sleep(1)
    cam_pub.publish(cam_msg)
    rospy.sleep(3)

    bot_msg.linear.x = 0
    bot_msg.angular.z = 0
    bot_pub.publish(bot_msg)

    is_charging = False
    bot_cam_together = True
    wait_alpha = True

    phase_one = True
    phase_two = False
    phase_two_half = False
    phase_three = False
    
    # only start docking last, after cam and bot are together and all status are setup correctly
    is_docking = True


def docking(event):
    global tag_visible, width, cx, tx, ty, alpha, alpha_pos_count, alpha_neg_count, \
        cam_pub, cam_msg, bot_pub, bot_msg, is_charging, is_docking, bot_cam_together, direction, wait_alpha, \
        phase_one, phase_two, phase_two_half, phase_three, alpha_move, alpha_move, t_smooth_lock, alpha_smooth_lock
    if not is_docking:
        return
    if phase_one:
        print("ty:", ty, "      alpha:", alpha)
        t_smooth_lock = True
        alpha_smooth_lock = True
        # if tag_visible and (abs(cx - 0.5 * width) / width) < 0.03:
        if tag_visible and abs(ty) < 0.05:
            bot_msg.angular.z = 0
            print("phase 1      stoping robot before spinning camera")
            bot_pub.publish(bot_msg)
            rospy.sleep(1)

            if bot_cam_together:
                # return instead of if/else to avoid cam_pub
                if wait_alpha:
                    alpha_pos_count = 0
                    alpha_neg_count = 0
                    wait_alpha = False
                    return
                if abs(alpha_pos_count - alpha_neg_count) < 20:
                    return
                print("alpha_pos_count:", alpha_pos_count, "    alpha_neg_count:", alpha_neg_count)
                # robot at left, negative alpha, direction = 1, drive forward, camera spin left by 90 - |alpha|
                # robot at right, positive alpha, direction = -1, drive backward, camera spin left by 90 + |alpha|
                direction = 1 if alpha_neg_count > alpha_pos_count else -1
                # camera spin left with increased pan
                print("=============================================")
                print("original pan (expect 90):", cam_msg.pan.data)
                print("alpha:", alpha)
                print("direction:", direction)
                cam_msg.pan.data += 90 - direction * abs(alpha)
                print("first cam spin pan:", cam_msg.pan.data)
                print("=============================================")
                bot_cam_together = False
                # in case phase_one again, also need to have cam_bot_together physically
                wait_alpha = True
            else:
                # spin camera to face left
                cam_msg.pan.data = 180
                phase_one = False
                phase_two = True
                print("alpha:", alpha)
                print("exiting phase 1, pan set to 180 (face left of robot)\n\n\n")
                # is_docking = False

            cam_pub.publish(cam_msg)
            rospy.sleep(1)
            cam_pub.publish(cam_msg)
            rospy.sleep(3)
        else:
            # robot turn right when z < 0
            bot_msg.angular.z = -0.08 if tag_visible else -0.4
            bot_pub.publish(bot_msg)
        return

    # if alpha is too big, set phase_one = true and redo phase one
    if phase_two:
        t_smooth_lock = True
        alpha_smooth_lock = False
        if tag_visible:
            # when camera is facing left 90deg of the robot, rotation center is 8.5cm (0.085m) to the right of optical camera
            ry = ty + 0.03
            print("ty:", ty, "  ry:", ry)
            if abs(ry) < 0.03:
                bot_msg.linear.x = 0
                bot_pub.publish(bot_msg)
                print("alpha:", alpha)
                cam_msg.pan.data = 270
                cam_pub.publish(cam_msg)
                rospy.sleep(1)
                cam_pub.publish(cam_msg)
                rospy.sleep(3)
                phase_two = False
                phase_two_half = True
                alpha_move = []
                alpha_move_len = 0
                alpha = 999
                print("exiting phase 2, robot stop sideways on normal line\n\n\n")
                # is_docking = False
            else:
                direction = 1 if (ry) < 0 else -1
                speed = 0.04 if abs(ry) < 0.3 else 0.2
                bot_msg.linear.x = direction * speed
        else:
            # drive blind based on previous direction, hope to dirve parallel to tag plane
            bot_msg.linear.x = direction * 0.5
        bot_pub.publish(bot_msg)
        return

    if phase_two_half:
        t_smooth_lock = False
        alpha_smooth_lock = True
        print("alpha:", alpha)
        if abs(alpha) < 1:
            bot_msg.angular.z = 0
            phase_two_half = False
            phase_three = True
            print("========================")
            print("alpha:", alpha)
            print("exiting phase 2.5, robot face backwards to tag\n\n\n")
            rospy.sleep(3)
            # is_docking = False
        else:
            bot_msg.angular.z = -0.08 if tag_visible else -0.4
        bot_pub.publish(bot_msg)
        return

    if phase_three:
        t_smooth_lock = True
        alpha_smooth_lock = False
        print("tx:", tx)
        terminate = False
        if is_charging:
            terminate = True
            print("\n===========================")
            print("robot is charging")
        elif tx < 0.8:
            terminate = True
            print("\n===========================")
            print("too close to station and still NOT charging")
        else:
            bot_msg.linear.x = -0.06 if tx < 1 else -0.4
        if terminate:
            bot_msg.linear.x = 0
            phase_three = False
            is_docking = False
            print("exiting phase 3\n\n\n")
        bot_pub.publish(bot_msg)
        return




rospy.init_node("docking")
cam_pub = rospy.Publisher("/ptz/move", PTZPosition, queue_size=1)
bot_pub = rospy.Publisher("/vel_mux/cmd_vel", Twist, queue_size=1)
tag_info_sub = rospy.Subscriber("/apriltag_detection", TagInfo, tag_update, queue_size=1)

# 20hz cycle
# time.sleep() inside the timer callback does not affect the main cycle such as tag detection
rospy.Timer(rospy.Duration(0.05), docking)


start_docking()
print("start rospy spin\n\n")
rospy.spin()