import numpy as np
import cv2 as cv

import tracker
import argparse
import constants


class MultipleTracker(object):
    BASE_VID_FILE = "files/callibration/callib"

    DEFAULT_CAMS = 2
    DEFAULT_INNER = [
        5.38614e+02, 0., 3.10130e+02,
        0., 5.38112e+02, 2.27066e+02,
        0., 0., 1.
    ]
    DEFAULT_CAM_INFO = [
        {
            "flip" : False,
            "inner" : DEFAULT_INNER
        },
        {
            "flip" : True,
            "inner" : DEFAULT_INNER
        },
    ]

    def __init__(
        self,
        num_of_cams=DEFAULT_CAMS,
        cam_info=DEFAULT_CAM_INFO
    ):
        self._cam_info = cam_info
        self._num_of_cams = num_of_cams

        self._ext_callib = {}

        self._trackers = [
            tracker.Tracker(
                mode=tracker.Tracker.TRACKING_MODE
            )
            for index in range(self._num_of_cams)
        ]
    def read_frame(self, camera_index):
        camera_tracker = self._trackers[camera_index]
        ret, frame = camera_tracker._cap.read()
        if self._cam_info[camera_index]["flip"]:
            frame = cv.flip(frame, -1)
        return ret, frame

    def init_trackers(self):
        for index, camera_tracker in enumerate(self._trackers):
            camera_tracker._mode = tracker.Tracker.SCANNING_MODE
            camera_tracker._frame_name = "frame - %d" % index

            try:
                os.remove(camera_tracker._write_model_file)
            except:
                pass

            camera_tracker._cap = cv.VideoCapture(index + 1)
            size = (
                constants.WIDTH - 2 * constants.X_BORDER,
                constants.HEIGHT - 2 * constants.Y_BORDER,
            )
            fourcc = cv.VideoWriter_fourcc(*'MPEG')  # 'x264' doesn't work
            camera_tracker._out = cv.VideoWriter(
                "%s%d.avi" % (MultipleTracker.BASE_VID_FILE, index),
                fourcc,
                20.0,
                size
            )
            ret, camera_tracker._old_frame = self.read_frame(index)

        self.external_calib()

    def external_calib(self):
        count = 0

        #criteria =(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((9*6,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        objpoints = []   # 3d points in real world space
        corners = []

        ###cv.FindExtrinsicCameraParams()

        for frame in map(lambda x: x._old_frame, self._trackers):
            frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
            find, corner = cv.findChessboardCorners(frame,(9,6),None)
            viz_frame = frame.copy()
            if find:
                #cv.cornerSubPix(frame,corner,(11,11),(-1,-1),criteria)
                corners.append(corner)
                print "camera ", count
                for point in corner:
                    cv.circle(viz_frame,tuple(map(int ,point.tolist()[0])),5,(0,255,0),-1)
                cv.imshow(("Corners " + str(count)), viz_frame)
                k = cv.waitKey(1) & 0xff
                if k == 27:
                    return False
            else:
                print "not camera", count
            count += 1
        """
        if len(corners) == self._num_of_cams:
            objpoints.append(objp)
            for frame, corner in zip(map(lambda x: x._old_frame, self._trackers), corners):
                print frame
                frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
                print frame.shape
                retR, mtxR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints,
                                                        corner,
                                                        frame.shape[:],None,None)
                hR,wR= frame.shape[:2]
                OmtxR, roiR= cv.getOptimalNewCameraMatrix(mtxR,distR,
                                                   (wR,hR),1,(wR,hR))
                print "shit : ",retR, mtxR, distR, rvecsR, tvecsR,hR,wR ,OmtxR, roiR
        """

    def match_points(self, point_positions):
        """
        Args: point_positions, list of numpy arrays
        Output: list of tuples, each tuple is made up of the dots by indexes
        """

        output = []

        return output

    def triangulate(self, point_positions):
        return [0,0,0]

    def run(self):
        self.init_trackers()

        keep_running = True
        while keep_running:
            point_positions = []

            # grab our point from ech camera
            for index, camera_tracker in enumerate(self._trackers):
                ret, frame = self.read_frame(index)
                keep_running = camera_tracker.run_once(frame)
                if not keep_running:
                    break

                point_positions.append(camera_tracker._curr_points)

            # match between the points
            matched_points = self.match_points(point_positions)

            # find the relative position to the first cameras
            rel_position = self.triangulate(point_positions)



def parse_args():
    parser = argparse.ArgumentParser(description='Callibrating camera')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    point_tracker = MultipleTracker()

    # mapping phase
    point_tracker.run()


if __name__ == "__main__":
    main()
