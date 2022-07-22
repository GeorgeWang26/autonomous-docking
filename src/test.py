import rospy
import time


def my_callback(event):
    print('Timer called at ' + str(event.current_real))
    # time.sleep(4)
if __name__ == "__main__":
    rospy.init_node("timer")
    rospy.Timer(rospy.Duration(2), my_callback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("from loop")
        r.sleep()