import tracker
import argparse


def parse_args():
    parser = argparse.ArgumentParser(description='Tracking points')
    parser.add_argument('--mode', default=tracker.Tracker.TRACKING_MODE,
                       type=int, help='directory of all the training faces')
    parser.add_argument('--scan-vid', default=tracker.DEFAULT_SCAN,
                       type=str, help='file to scan')
    parser.add_argument('--track-vid', default=tracker.DEFAULT_TRACK,
                       type=str, help='file to scan')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    point_tracker = tracker.Tracker(
        mode=args.mode,
        scan_file=args.scan_vid,
        track_file=args.track_vid
    )

    # scanning phase
    #point_tracker.scan_points()

    #point_tracker = tracker.Tracker()

    # mapping phase
    point_tracker.run()


if __name__ == "__main__":
    main()
