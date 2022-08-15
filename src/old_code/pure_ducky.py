import cv2
import yaml
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R


# cam = cv2.VideoCapture(0)

# x86 color
# cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink")

# arm color
# cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, format=BGRx ! appsink")

# arm gray, by default read in nv12 format, single channle
cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink")

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
new_cam_params = (new_cam_mtx[0,0], new_cam_mtx[1,1], new_cam_mtx[0,2], new_cam_mtx[1,2])


tag_size = 0.169
at_detector = Detector(families='tag36h11',
                            nthreads=12,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

while True:
    # rectify
    ret, raw_frame = cam.read()
    # x86
    # rect_color_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)
    # rect_frame = cv2.cvtColor(rect_color_frame, cv2.COLOR_BGR2GRAY)
    rect_frame = cv2.undistort(raw_frame, cam_mtx, dist_cef, None, new_cam_mtx)


    # print(rect_color_frame.shape)

    # display using cv
    cv2.imshow("raw", raw_frame)
    # cv2.imshow("rect_color", rect_color_frame)
    cv2.imshow("rect", rect_frame)

    # use new_cam_params since rect_frame is undistorted
    #tags = at_detector.detect(rect_frame, True, new_cam_params, tag_size)
    tags = []
    for tag in tags:
        # print(tag)
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
