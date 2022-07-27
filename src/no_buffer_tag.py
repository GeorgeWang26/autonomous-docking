import cv2
import rospy
import yaml
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R


cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink drop=true")

with open("resources/ptz_calibration.yaml") as f:
    calibration = yaml.safe_load(f)
    f.close()

w = calibration["image_width"]
h = calibration["image_height"]
dist_cef = np.array(calibration["distortion_coefficients"]["data"])
cam_mtx = np.array(calibration["camera_matrix"]["data"]).reshape((3, 3))
projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
new_cam_mtx = np.delete(projection_mtx, -1, axis=1)
new_cam_params = (new_cam_mtx[0,0], new_cam_mtx[1,1], new_cam_mtx[0,2], new_cam_mtx[1,2])

tag_size = 0.166
at_detector = Detector(families='tag36h11',
                            nthreads=12,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

while True:
    ret, raw_frame = cam.read()
    rect_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)
    cv2.imshow("rect", rect_frame)

    # use new_cam_params since rect_frame is undistorted
    tags = at_detector.detect(rect_frame, True, new_cam_params, tag_size)
    #tags = []
    for tag in tags:
        r = R.from_matrix(tag.pose_R)
        euler = r.as_euler("zxy", degrees=True)
        x = tag.pose_t[2][0]
        y = -1 * tag.pose_t[0][0]
        z = -1 * tag.pose_t[1][0]
        print(x, y, z)
        print(euler)
        print("--------------------------------------")

    if cv2.waitKey(1) == 27:
        break
