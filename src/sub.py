import rospy
from std_msgs.msg import String
import time

def callback_receive_data(msg):
    print(msg.data)
    time.sleep(3)

if __name__ == '__main__':
    rospy.init_node('subscriber_node')
    sub = rospy.Subscriber("/my_topic", String, callback_receive_data, queue_size = 1)

    rospy.spin() #keep the script waiting for the callback