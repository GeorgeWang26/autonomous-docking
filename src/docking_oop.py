import rospy
from auto_docking.msg import TagInfo
from ptz_pkg.msg import PTZPosition
from geometry_msgs.msg import Twist


class Docking():
    def __init__(self):
        self.tag_visible = False
        self.width = 0
        # self.cx = 0
        self.tx = 0
        self.ty = 0
        self.alpha = 0
        self.alpha_pos_count = 0
        self.alpha_neg_count = 0
        self.alpha_move = []
        self.t_smooth_lock = False
        self.first_visible = False

        self.is_charging = False
        self.is_docking = False
        self.bot_cam_together = False
        self.direction = 0
        self.wait_alpha = True
        # self.alpha_lock = 0

        self.second_time = False
        self.phase_one = False
        self.phase_one_second_time = False
        self.phase_two = False
        self.phase_two_half = False
        self.phase_three = False
        self.phase_four = False

        self.bot_msg = Twist()
        self.cam_msg = PTZPosition()

        rospy.init_node("docking")
        self.cam_pub = rospy.Publisher("/ptz/move", PTZPosition, queue_size=1)
        self.bot_pub = rospy.Publisher("/vel_mux/cmd_vel", Twist, queue_size=1)
        self.tag_info_sub = rospy.Subscriber("/apriltag_detection", TagInfo, self.tag_update, queue_size=1)
        # 20hz cycle
        # time.sleep() inside the timer callback does not affect the main cycle such as tag detection
        rospy.Timer(rospy.Duration(0.05), self.docking)


    def tag_update(self, msg):
        if msg.family == "":
            self.tag_visible = False
            self.alpha_move = []
            self.first_visible = True
            return

        self.width = msg.width
        # self.cx = msg.cx

        if self.t_smooth_lock:
            if self.first_visible:
                self.tx = msg.tx
                self.ty = msg.ty
            else:
                if abs(self.tx - msg.tx) < 0.3:
                    self.tx = msg.tx
                if abs(self.ty - msg.ty) < 0.3:
                    self.ty = msg.ty
        else:
            self.tx = msg.tx
            self.ty = msg.ty
        
        self.alpha_move.append(msg.yaw)
        alpha_move_len = len(self.alpha_move)
        if alpha_move_len > 5:
            self.alpha_move.pop(0)
            alpha_move_len -= 1
        alpha_sum = 0
        for val in self.alpha_move:
            alpha_sum += val

        self.alpha = alpha_sum / alpha_move_len
        if self.alpha > 0:
            self.alpha_pos_count += 1
        else:
            self.alpha_neg_count += 1

        self.first_visible = False
        # only set tag_visible=True after all status are updated
        self.tag_visible = True


    def start_docking(self):
        print("camera face forward together with robot\n")
        self.cam_msg.pan.data = 90
        self.cam_msg.tilt.data = 90
        self.cam_msg.zoom.data = 0
        self.cam_msg.degrees.data = True
        self.cam_pub.publish(self.cam_msg)
        rospy.sleep(1)
        self.cam_pub.publish(self.cam_msg)
        rospy.sleep(3)

        self.bot_msg.linear.x = 0
        self.bot_msg.angular.z = 0
        self.bot_pub.publish(self.bot_msg)

        self.is_charging = False
        self.bot_cam_together = True
        self.wait_alpha = True
        # self.alpha_lock = 0

        self.phase_one = True
        self.phase_one_second_time = False
        self.phase_two = False
        self.phase_two_half = False
        self.phase_three = False
        self.phase_four = False

        # only start docking last, after cam and bot are together and all status are setup correctly
        self.is_docking = True

    
    def start_docking_second_time(self):
        self.bot_msg.linear.x = 0
        self.bot_pub.publish(self.bot_msg)

        self.cam_msg.pan.data = 180
        self.cam_pub.publish(self.cam_msg)
        rospy.sleep(1)
        self.cam_pub.publish(self.cam_msg)
        rospy.sleep(3)

        self.second_time = True
        # self.alpha_lock = 0

        self.phase_one = False
        self.phase_one_second_time = True
        self.phase_two = False
        self.phase_two_half = False
        self.phase_three = False
        self.phase_four = False
        

    def docking(self, event):
        if not self.is_docking:
            return
        if self.phase_one:
            print("ty:", self.ty, "      alpha:", self.alpha)
            self.t_smooth_lock = False
            # if self.tag_visible and (abs(self.cx - 0.5 * self.width) / self.width) < 0.03:
            # if self.tag_visible and ((not self.second_time and abs(self.ty) < 0.05) or (self.second_time and abs(self.ty) < 0.05)):
            if self.tag_visible and abs(self.ty) < 0.05:
                self.bot_msg.angular.z = 0
                print("phase 1      stoping robot before spinning camera")
                self.bot_pub.publish(self.bot_msg)
                rospy.sleep(1)

                if self.bot_cam_together:
                    # return instead of if/else to avoid cam_pub and sleep
                    if self.wait_alpha:
                        self.alpha_pos_count = 0
                        self.alpha_neg_count = 0
                        self.wait_alpha = False
                        return
                    if abs(self.alpha_pos_count - self.alpha_neg_count) < 20:
                        return
                    print("alpha_pos_count:", self.alpha_pos_count, "    alpha_neg_count:", self.alpha_neg_count)
                    # robot at left, negative alpha, direction = 1, drive forward, camera spin left by 90 - |alpha|
                    # robot at right, positive alpha, direction = -1, drive backward, camera spin left by 90 + |alpha|
                    self.direction = 1 if self.alpha_neg_count > self.alpha_pos_count else -1
                    # camera spin left with increased pan
                    print("=============================================")
                    print("original pan (expect 90):", self.cam_msg.pan.data)
                    print("alpha:", self.alpha)
                    print("direction:", self.direction)
                    self.cam_msg.pan.data += 90 - self.direction * abs(self.alpha)
                    print("first cam spin pan:", self.cam_msg.pan.data)
                    print("=============================================")
                    self.bot_cam_together = False
                else:
                    # spin camera to face left
                    self.cam_msg.pan.data = 180
                    self.phase_one = False
                    self.phase_two = True
                    print("alpha:", self.alpha)
                    print("exiting phase 1, pan set to 180 (face left of robot)\n\n\n")
                    # self.is_docking = False

                self.cam_pub.publish(self.cam_msg)
                rospy.sleep(1)
                self.cam_pub.publish(self.cam_msg)
                rospy.sleep(3)
            else:
                # in case robot miss the tag after it stopped
                # alpha count need to reset next time when robot stop again
                self.wait_alpha = True
                # robot turn right when z < 0
                self.bot_msg.angular.z = -0.04 if self.tag_visible else -0.2
                self.bot_pub.publish(self.bot_msg)
            return

        if self.phase_one_second_time:
            print("alpha: ", self.alpha)
            if self.tag_visible and abs(self.alpha) < 2.5:
                self.bot_msg.angular.z = 0
                self.phase_one_second_time = False
                self.phase_two = True
                print("exiting phase 1 second time\n\n\n")
                rospy.sleep(3)
            else:
                # spin left when z > 0
                self.bot_msg.angular.z = 0.04 if self.tag_visible else 0.2
                self.bot_pub.publish(self.bot_msg)
            return

        if self.phase_two:
            self.t_smooth_lock = True
            if self.tag_visible:
                ry = self.ty - 0.11 if not self.second_time else self.ty + 0.21
                print("ty:", self.ty, "  ry:", ry)
                if abs(ry) < 0.01:
                    self.bot_msg.linear.x = 0
                    self.bot_pub.publish(self.bot_msg)
                    print("alpha:", self.alpha)
                    self.cam_msg.pan.data = 270
                    self.cam_pub.publish(self.cam_msg)
                    rospy.sleep(1)
                    self.cam_pub.publish(self.cam_msg)
                    rospy.sleep(3)
                    self.phase_two = False
                    self.phase_two_half = True
                    # reset all alpha related data to have fresh spin on phase 2.5
                    self.alpha_move = []
                    self.alpha = 999
                    print("exiting phase 2, robot stop sideways on normal line\n\n\n")
                    # self.is_docking = False
                else:
                    self.direction = 1 if (ry) < 0 else -1
                    speed = 0.04 if abs(ry) < 0.3 else 0.2
                    self.bot_msg.linear.x = self.direction * speed
            else:
                # drive blind based on previous direction, hope to dirve parallel to tag plane
                print("tag not visible, speed: 0.3")
                self.bot_msg.linear.x = self.direction * 0.3
            self.bot_pub.publish(self.bot_msg)
            return

        if self.phase_two_half:
            # may enable t_smooth_lock if needed in testing
            self.t_smooth_lock = False
            print("alpha:", self.alpha)
            if (not self.second_time and abs(self.alpha) < 1) or (self.second_time and abs(self.alpha) < 3):
                # print("stop now with count:", self.alpha_lock)
                self.bot_msg.angular.z = 0
                # if self.alpha_lock < 4:
                #     self.bot_pub.publish(self.bot_msg)
                #     self.alpha_lock += 1
                #     return
                self.phase_two_half = False
                self.phase_three = True
                print("========================")
                print("alpha:", self.alpha)
                print("exiting phase 2.5, robot face backwards to tag\n\n\n")
                rospy.sleep(3)
                # self.is_docking = False
            else:
                # self.alpha_lock = 0
                self.bot_msg.angular.z = -0.04 if self.tag_visible else -0.2
            self.bot_pub.publish(self.bot_msg)
            return

        if self.phase_three:
            self.t_smooth_lock = True
            print("tx:", self.tx)
            terminate = False

            if not self.tag_visible:
                terminate = True
                self.is_docking = False
                print("\nphase 33333, abort docking, tag is not visible")
            elif self.is_charging:
                # wont do anything for now
                terminate = True
                print("\nrobot is charging")
            elif self.tx < 0.75:
                terminate = True
                print("\ntoo close to station and still NOT charging")
            else:
                self.bot_msg.linear.x = -0.06 if self.tx < 1.3 else -0.2
                if not self.second_time and self.tx < 1.2:
                    self.start_docking_second_time()
                    return

            if terminate:
                self.bot_msg.linear.x = 0
                self.phase_three = False
                self.phase_four = True
                print("exiting phase 3\n\n\n")
                rospy.sleep(3)
                # self.is_docking = False
            self.bot_pub.publish(self.bot_msg)
            return

        if self.phase_four:
            self.t_smooth_lock = False
            print("alpha:", self.alpha)
            if not self.tag_visible:
                self.bot_msg.angular.z = 0
                self.is_docking = False
                print("\nphase 44444, abort docking, tag is not visible")
            if abs(self.alpha) < 0.5:
                self.bot_msg.angular.z = 0
                self.phase_four = False
                print("exiting phase 4")
                self.is_docking = False
            else:
                # turn left if face right side of tag
                self.bot_msg.angular.z = 0.04 if self.alpha < 0 else -0.04
            self.bot_pub.publish(self.bot_msg)
            return


if __name__ == "__main__":
    docking = Docking()
    # make a service that call start_docking
    docking.start_docking()
    print("start rospy spin\n\n")
    rospy.spin()