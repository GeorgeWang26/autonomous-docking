import rospy
from std_msgs.msg import String
import time

rate = None
def callback_receive_data1(msg):
    print("1:", msg.data)
    counter = 0
    while(True):
        counter += 1

def callback_receive_data2(msg):
    print("2:",  msg.data)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('subscriber_node')
    sub1 = rospy.Subscriber("/my_topic", String, callback_receive_data1, queue_size = 1)
    sub2 = rospy.Subscriber("/my_topic", String, callback_receive_data2, queue_size = 1)
    rate = rospy.Rate(0.3)

    rospy.spin() #keep the script waiting for the callback