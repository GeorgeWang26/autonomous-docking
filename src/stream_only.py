import cv2
import rospy
import yaml
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

# arm gray, by default read in nv12 format, single channle
#cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink")
cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink drop=true")

# with open("resources/webcam_calibration.yaml") as f:
with open("resources/ptz_calibration.yaml") as f:
    calibration = yaml.safe_load(f)
    f.close()


# ros calibration pkg uses cv2.getOptimalNewCameraMatrix(alpha = 0) to calculate projection_mtx
# don't need to re-calculate, just read from yml
# new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_cef, (w,h), 0, (w,h))
w = calibration["image_width"]
h = calibration["image_height"]
dist_cef = np.array(calibration["distortion_coefficients"]["data"])
cam_mtx = np.array(calibration["camera_matrix"]["data"]).reshape((3, 3))
projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
new_cam_mtx = np.delete(projection_mtx, -1, axis=1)
print("===========================")
#cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
print("===========================")
print("<<", cam.get(cv2.CAP_PROP_BUFFERSIZE), ">>")
print("===========================")


rospy.init_node("stream_only")
pub = rospy.Publisher("/image_encoded", Image, queue_size = 1)
msg = Image()
while True:
    # rectify
    ret, raw_frame = cam.read()
    rect_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)
    
    #print(rect_frame.shape)
    
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    result, encimg = cv2.imencode('.jpg', rect_frame, encode_param)
    #print(type(encimg[0]))
    decimg = cv2.imdecode(encimg, cv2.IMREAD_GRAYSCALE)
    #print(encimg.shape)
    #print(decimg.shape)
    #print("===========================================")
    msg.data = encimg.tolist()
    #print(msg)
    #print(type(encimg.tolist()))
    pub.publish(msg)
    # display using cv
    #cv2.imshow("raw", raw_frame)
    #cv2.imshow("rect", rect_frame)
    #cv2.imshow("decoded", decimg)
    if cv2.waitKey(1) == 27:
        break
