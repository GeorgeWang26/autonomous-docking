import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

def callback(msg):
    #print(msg)
    msg_data = msg.data
    #print(msg_data)
    #print(msg.width)
    #print(type(msg_data), type(msg_data[0]), len(msg_data), msg_data[0])
    
    #data = np.array(msg_data)
    data = np.frombuffer(msg_data, dtype=np.uint8)
    #print(data)
    #print(type(data))
    #print(type(data[0]))
    #print(data.shape)
    
    frame = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
    #print(frame.shape)
    cv2.imshow("decoded from sub", frame)
    cv2.waitKey(1)

rospy.init_node("decode_ros")
sub = rospy.Subscriber("/image_encoded", Image, callback, queue_size = 1)
rospy.spin()
