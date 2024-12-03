#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np


class marker_finder():

    def __init__(self, marker_id, marker_dic):
        self.marker_id = marker_id
        self.marker_dic = marker_dic
        # Load the dictionary that was used to generate the markers.
        self.aruco_dict = aruco.getPredefinedDictionary(self.marker_dic)

        # Initialize the detector parameters using default values
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def finder(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the frame
        corners, ids, rejected = self.detector.detectMarkers(gray)

        # Get the location of each marker
        # for i, corner in enumerate(corners):
        #     # Each corner contains four points (the four corners of the marker)
        #     # Here we are just using the first corner
        #     if self.marker_id == ids[0]:
        #         return corner
        if not ids is None:
            corners = list(corners)
            for i in range(len(ids)):
                # Each corner contains four points (the four corners of the marker)
                # Here we are just using the first corner

                if self.marker_id == ids[i]:
                    return corners[i]
        return None

    def demo(self):
        # Start the video capture
        cap = cv2.VideoCapture(0)

        while True:
            # Capture a frame
            ret, frame = cap.read()
            if not ret:
                break

            corner = self.finder(frame=frame)

            if corner is not None:
                # Draw detected markers
                frame = aruco.drawDetectedMarkers(frame, np.array([corner]), np.array([self.marker_id]))

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # Break the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        cap.release()
        cv2.destroyAllWindows()


class marker_pos_locator():

    def __init__(self, target_id, marker_dic):

        self.marker_id = target_id
        self.marker_dic = marker_dic
        # Load the dictionary that was used to generate the markers.
        self.aruco_dict = aruco.getPredefinedDictionary(self.marker_dic)

        # Initialize the detector parameters using default values
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def finder(self, frame, return_target = False):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the frame
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if not ids is None:
            corners = list(corners)
            for i in range(len(ids)):
                # Each corner contains four points (the four corners of the marker)
                # Here we are just using the first corner

                if self.marker_id == ids[i]:
                    if return_target:
                        return corners[i], self.marker_id
                    else:
                        return corners[i]
        if return_target:
            return None, None
        else:
            return None