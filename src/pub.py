import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node('publisher_node')
    pub = rospy.Publisher("my_topic", String, queue_size = 1)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        msg=String()
        msg.data = str(count)
        print(count)
        pub.publish(msg)
        rate.sleep()
        count += 1
    rospy.loginfo("Node was stopped")