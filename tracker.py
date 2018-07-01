import numpy as np
import cv2 as cv
import os
import sys
import time

import descriptor
import model
import util
import constants

DEFAULT_SCAN = "files/jacobs/4.mp4"
DEFAULT_TRACK = "files/jacobs/4.mp4"


class Tracker(object):
    FEATURES_TO_SCAN = 10
    FEATURES_TO_TRACK = 10

    SAME_DIST = 15

    CLOSE_CANDIDATES = 20
    QUALITY_LEVEL = 0.2

    DESCRIPTOR_THRESH = 2500

    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = FEATURES_TO_SCAN,
                            qualityLevel = QUALITY_LEVEL,
                           minDistance = 50,
                           blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))


    SCAN_OUTPUT_FILE = 'files/outputs/jacob_scan_output.avi'
    TRACK_OUTPUT_FILE = 'files/outputs/jacob_track_output.avi'
    CREATE_MODEL_FILE = "models/jacobs/model.csv"
    USE_MODEL_FILE = "models/jacobs/model.csv"

    (
        NEW_POINTS,
        OLD_POINTS
    )=range(2)

    (
        SCANNING_MODE,
        TRACKING_MODE
    )=range(2)

    MODE_NAMES = {
        SCANNING_MODE : "SCANNING",
        TRACKING_MODE : "TRACKING"
    }

    def __init__(
        self,
        mode=SCANNING_MODE,
        scan_file=DEFAULT_SCAN,
        track_file=DEFAULT_TRACK,
        write_model_file=CREATE_MODEL_FILE,
        read_model_file=USE_MODEL_FILE,
        start_file="models/start.csv"
    ):
        self._scan_file = scan_file
        self._track_file = track_file
        self._write_model_file = write_model_file
        self._read_model_file = read_model_file
        self._start_file = start_file

        if mode==Tracker.SCANNING_MODE:
            self._features_to_find = Tracker.FEATURES_TO_SCAN
        else:
            self._features_to_find = Tracker.FEATURES_TO_SCAN
        Tracker.feature_params["maxCorners"] = self._features_to_find

        self._frame_name = "frame"

        self._mode = mode

        self._curr_points = []
        self._old_points = []

        self._curr_frame = None
        self._old_frame = None

        self._curr_num = 1
        self._curr_name = 'AAA'

        self._st = None
        self._last_track = Tracker.NEW_POINTS

        self._documented_points = []

        self._working_set = {}
        self._model = {}
        self._old_descs = []

        # maps from index to label
        self._labels = []

        self._first_frame = True

        self._color = np.random.randint(0,255,(100,3))
        self._sift = cv.xfeatures2d.SIFT_create()
        self._desc_obj = descriptor.descriptor()
        self._bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    def run(self):
        if self._mode == Tracker.SCANNING_MODE:
            self.scan_points()
        else:
            self.track_points()

    def label_points(self, viz_frame):
        for name, point in self._working_set.items():
            screen_point = tuple(map(int, point.tolist()[0]))
            util.add_text(viz_frame, name, screen_point)
        return viz_frame

    def next_name(self):
        self._curr_num += 1
        return (
            str(chr(ord('A') + self._curr_num / 26**2))
            + str(chr(ord('A') + (self._curr_num / 26) % 26))
            + str(chr(ord('A') + self._curr_num % 26))
        )

    def find_features(self):
        # Check if we're out of features
        if len(self._curr_points) < self._features_to_find - 2:
            self._last_track = Tracker.NEW_POINTS

            # Find again good features to track
            new_points = []
            while True:
                new_points = cv.goodFeaturesToTrack(
                    self._curr_frame[16:-16, 16:-16],
                    mask = None,
                    **Tracker.feature_params
                )
                if new_points is not None:
                    break
                Tracker.feature_params["qualityLevel"] -= 0.1
            Tracker.feature_params["qualityLevel"] = Tracker.QUALITY_LEVEL

            for point in new_points:
                point[0][0]+= 16
                point[0][1]+= 16


            self._labels = [None] * len(new_points)


            # Add points / match points to model
            new_working_set = {}
            for index, point in enumerate(new_points):
                closest_point = min([
                    (name, np.linalg.norm(point - visited_p))
                    for name, visited_p in self._working_set.items()
                ] + [("First", Tracker.SAME_DIST*100)],
                key=lambda x: x[1]
                )

                # check if this is a new point
                if closest_point[1] > Tracker.SAME_DIST:
                    # If we are in scanning mode, just label this new point
                    if self._mode == Tracker.SCANNING_MODE:
                        new_working_set[self._curr_name] = point
                        self._labels[index] = self._curr_name

                        # get next name
                        self._curr_name = self.next_name()

                    # If we are in tracking mode, need to find which goddamn
                    # label this is
                    else:
                        label = self.label_point(point)
                        if label is not None:
                            new_working_set[label] = point
                            self._labels[index] = label

                else:
                    new_working_set[closest_point[0]] = point
                    self._labels[index] = closest_point[0]

            self._working_set = new_working_set


        # Otherwise OK, use optical flow
        else:
            new_points, self._st, err = cv.calcOpticalFlowPyrLK(
                self._old_frame,
                self._curr_frame,
                self._curr_points,
                None,
                **Tracker.lk_params
            )

            self._last_track = Tracker.OLD_POINTS

            # udpate points
            new_labels = []
            for index, point in enumerate(new_points):
                point_name = self._labels[index]

                # check if need to remove
                if self._st[index]:
                    self._working_set[point_name] = point
                    new_labels.append(point_name)
                else:
                    try:
                        del self._working_set[point_name]
                    except:
                        pass

            self._labels = new_labels

        return new_points

    '''
    def setup_track(self, screen_points):
        # read csv file
        content = util.read_file(self._start_file)
        start_points = {}

        for point in content.split('\n')[:-1]:
            name, x, y = tuple(point.split(','))
            point = np.array([float(coord) for coord in [x, y]])
            start_points[name] = np.array([point])

            # find closest point to this point
            min_index, min_dist = min([
                (index, np.linalg.norm(s_p - point))
                for index, s_p in enumerate(screen_points)
            ],
            key=lambda x: x[1]
            )
            self._labels[min_index] = name
        self._working_set = start_points
    '''



    def draw_points(self, viz_frame):
        if self._last_track == Tracker.OLD_POINTS:
            # Select good points
            good_new = self._curr_points[self._st==1]
            good_old = self._old_points[self._st==1]

            mask = np.zeros_like(self._viz_frame)

            # draw the tracks
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()
                mask = cv.line(mask, (a,b),(c,d), self._color[i].tolist(), 2)
                viz_frame = cv.circle(viz_frame,(a,b),5,self._color[i].tolist(),-1)

            viz_frame = cv.add(viz_frame, mask)

            self._curr_points = good_new.reshape(-1,1,2)
        else:
            for i, point in enumerate(self._curr_points):
                a,b = point.ravel()
                viz_frame = cv.circle(viz_frame, (a, b), 5, self._color[i].tolist(),-1)

        return viz_frame


    def img_process(self, frame):
        frame = cv.resize(frame, (constants.WIDTH, constants.HEIGHT))
        frame = frame[
            constants.Y_BORDER: constants.HEIGHT - constants.Y_BORDER,
            constants.X_BORDER: constants.WIDTH - constants.X_BORDER
        ]
        self._viz_frame = frame.copy()

        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self._curr_frame = frame.copy()

    def write_mode(self, frame):
        txt = Tracker.MODE_NAMES[self._mode]
        util.add_text(frame, txt, (20, 30), 100)
        return frame


    def vizualize(self, frame, debug=True):
        frame = self.write_mode(frame)

        # label points
        frame = self.label_points(frame)

        # Draw points
        frame = self.draw_points(frame)

        # Output image
        self._out.write(frame)

        if debug:
            cv.imshow(self._frame_name, frame)
            k = cv.waitKey(0) & 0xff
            if k == 27:
                return False
        return True

    def calc_sift_desc(self, point):
        y, x = map(int, point.tolist()[0])
        desc = self._desc_obj.create((x,y), self._curr_frame)
        return desc


    def document_points(self):
        with open(self._write_model_file, "a+") as f:
            # Add unvisited points to file
            for name, point in self._working_set.items():
                if name not in self._documented_points:
                    f.write("%s,0,0,0, %s\n" % (
                        name,
                        self.calc_sift_desc(point)
                    ))
                    self._documented_points.append(name)

    def read_model(self):
        content = util.read_file(self._read_model_file)
        model = {}
        for point in content.split('\n')[:-1]:
            name, x, y, z, desc = tuple(point.split(',', 4))
            model[name] = {
                "position": (float(x), float(y), float(z)),
                "descriptor" : eval(desc)
            }
        return model


    """
    Recieves a list of points on the screen that need to find a matching label
    Parms: list of numpy arrays

    """
    def label_point(self, to_match):
        # compute new descrptoy
        y, x = map(int, to_match.tolist()[0])
        new_desc = self._desc_obj.create((x, y), self._curr_frame)

        # find the best label for this
        relevant_descs = [
            (name, old_desc)
            for name, old_desc in self._old_descs
            if name not in self._labels
        ]
        best_fit = min([
            (name, np.linalg.norm(new_desc - old_desc))
            for name, old_desc in relevant_descs
        ],
        key=lambda x: x[1]
        )

        if best_fit[1] > Tracker.DESCRIPTOR_THRESH:
            return None
        return best_fit[0]



        """
        # computes the distances from the working set
        distances = []
        for name, model_point in self._model.items():
            x = model_point["position"][0]
            y = model_point["position"][1]
            pos2d = np.array([x, y])
            #print self._working_set.items()[0][1]

            sum_of_dist = sum([
                np.linalg.norm(w_s_point - pos2d)
                for w_s_name, w_s_point in self._working_set.items()
            ])

            distances.append((name, sum_of_dist))

        close_distances = sorted(distances, key=lambda x: x[1])


        return close_distances[0][0]
        """


    """
    One iteration of the tracking
    """
    def run_once(self, frame):
        # Process the image
        self.img_process(frame)

        # Track new points
        self._curr_points = self.find_features()

        # Document this point if we need
        if self._mode == Tracker.SCANNING_MODE:
            self.document_points()

        # Visualize tracking
        ret = self.vizualize(self._viz_frame)
        if not ret:
            return False

        # move on to next frame
        self._old_points = self._curr_points
        self._old_frame = self._curr_frame
        self._first_frame = False

        # Move on to th next frame
        return True


    def scan_points(self):
        self._mode = Tracker.SCANNING_MODE

        try:
            os.remove(self._write_model_file)
        except:
            pass

        self._cap = cv.VideoCapture(self._scan_file)
        size = (
            constants.WIDTH - 2 * constants.X_BORDER,
            constants.HEIGHT - 2 * constants.Y_BORDER,
        )
        fourcc = cv.VideoWriter_fourcc(*'MPEG')  # 'x264' doesn't work
        self._out = cv.VideoWriter(Tracker.SCAN_OUTPUT_FILE, fourcc, 20.0, size)

        ret, self._old_frame = self._cap.read()

        try:
            while True:
                ret, frame = self._cap.read()
                if not ret or self.run_once(frame):
                    break
        finally:
            cv.destroyAllWindows()
            self._cap.release()
            self._out.release()


    def track_points(self):
        self._mode = Tracker.TRACKING_MODE

        self._cap = cv.VideoCapture(self._track_file)
        size = (
            constants.WIDTH - 2 * constants.X_BORDER,
            constants.HEIGHT - 2 * constants.Y_BORDER,
        )
        fourcc = cv.VideoWriter_fourcc(*'MPEG')  # 'x264' doesn't work
        self._out = cv.VideoWriter(Tracker.TRACK_OUTPUT_FILE, fourcc, 20.0, size)

        # read model from file
        self._model = self.read_model()
        self._old_descs = [
            (name, np.array(point_dict["descriptor"]))
            for name, point_dict in self._model.items()
        ]

        # Main loop
        ret, self._old_frame = self._cap.read()
        try:
            while True:
                ret, frame = self._cap.read()
                if not ret or self.run_once(frame):
                    break
        finally:
            cv.destroyAllWindows()
            self._cap.release()
            self._out.release()
