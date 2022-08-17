import cv2
import rospy
import yaml
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R
from auto_docking.msg import TagInfo


class TagDetect():
    """
    Publish realtime AprilTag detection result.
    """
    def __init__(self):
        """
        Open streaming pipeline and load camera params from calibration result.
        """
        # webcam
        # self.cam = cv2.VideoCapture(0)
        # jetson
        self.cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! appsink drop=true")
        # non-jetson
        # self.cam = cv2.VideoCapture("rtspsrc location=rtsp://192.168.42.120:554/snl/live/1/1 latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true")


        # ros calibration pkg uses cv2.getOptimalNewCameraMatrix(alpha = 0) to calculate projection_mtx
        # don't need to re-calculate, just read from yml. roi is region of interest
        # new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_cef, (w,h), 0, (w,h))
        with open("resources/ptz_calibration.yaml") as f:
            calibration = yaml.safe_load(f)
            f.close()

        w = calibration["image_width"]
        h = calibration["image_height"]
        self.dist_cef = np.array(calibration["distortion_coefficients"]["data"])
        self.cam_mtx = np.array(calibration["camera_matrix"]["data"]).reshape((3, 3))
        projection_mtx = np.array(calibration["projection_matrix"]["data"]).reshape((3,4))
        self.new_cam_mtx = np.delete(projection_mtx, -1, axis = 1)
        self.new_cam_params = (self.new_cam_mtx[0,0], self.new_cam_mtx[1,1], self.new_cam_mtx[0,2], self.new_cam_mtx[1,2])

        self.tag_size = 0.169
        self.at_detector = Detector(families = 'tag36h11',
                                    nthreads = 12,
                                    quad_decimate = 1.0,
                                    quad_sigma = 0,
                                    refine_edges = 1,
                                    decode_sharpening = 0.25,
                                    debug = 0)

        self.tag_msg = TagInfo()
        self.tag_msg.width = w
        self.tag_msg.height = h

        rospy.init_node("vision_processing")
        self.tag_pub = rospy.Publisher("/apriltag_detection", TagInfo, queue_size = 1)


    def activate(self):
        """
        Constantly publish detection result of undistorted frames.
        """
        while True:
            ret, raw_frame = self.cam.read()
            rect_frame = cv2.undistort(raw_frame, self.cam_mtx, self.dist_cef, None, self.new_cam_mtx)

            # use new_cam_params since rect_frame is undistorted
            tags = self.at_detector.detect(rect_frame, True, self.new_cam_params, self.tag_size)
            self.tag_msg.family = ""
            # assuming only one tag in vision
            # tag = tags[0]
            for tag in tags:
                # print(tag)
                r = R.from_matrix(tag.pose_R)
                euler = r.as_euler("zxy", degrees = True)
                x = tag.pose_t[2][0]
                y = -1 * tag.pose_t[0][0]
                z = -1 * tag.pose_t[1][0]
                # print("x:", x, "y:", y, "z:", z)
                # print("euler", euler)
                # print("--------------------------------------")
                self.tag_msg.family = tag.tag_family.decode("utf-8")
                self.tag_msg.id = tag.tag_id
                self.tag_msg.cx = tag.center[0]
                self.tag_msg.cy = tag.center[1]
                self.tag_msg.tx = x
                self.tag_msg.ty = y
                self.tag_msg.tz = z
                self.tag_msg.roll = euler[0]
                self.tag_msg.pitch = euler[1]
                self.tag_msg.yaw = euler[2]
                
            self.tag_pub.publish(self.tag_msg)

            # cv2.imshow("rect", rect_frame)
            # if cv2.waitKey(1) == 27:
            #     break


if __name__ == "__main__":
    tag_detect = TagDetect()
    tag_detect.activate()