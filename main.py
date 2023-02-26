#!/usr/bin/env python
#author: Sourav R S
#Github: https://github.com/souravrs999

#General Imports
import threading
import cv2
import time
import socket
import numpy as np
from UnityGaze import GazeEstimation
from scipy.spatial import distance as dist
from UnityGaze.VideoCapture import WebcamVideoStream

# Get the screen resolution
def get_screen_res():

    import tkinter as tk

    root = tk.Tk()

    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()

    return screen_width, screen_height

def get_gaze_coords(cam_frame):
    p_x_2_bbox_w_r = None 
    p_y_2_bbox_h_r = None
    left_pupil = gaze.pupil_left_coords()
    right_pupil = gaze.pupil_right_coords()
    coord_coll = gaze.eye_roi_bb()

    if coord_coll is not None:

        left_coords = coord_coll[0]
        right_coords = coord_coll[1]

        left_x1 = left_coords[0]
        left_x2 = left_coords[1]
        left_y1 = left_coords[2]
        left_y2 = left_coords[3]

        right_x1 = right_coords[0]
        right_x2 = right_coords[1]
        right_y1 = right_coords[2]
        right_y2 = right_coords[3]

        ''' Average of the extreme cordinates of
        left pupil and right pupil '''

        avg_x1 = int((left_x1 + right_x1)/2 + 5)
        avg_x2 = int((left_x2 + right_x2)/2 - 5) 
        avg_y1 = int((left_y1 + right_y1)/2 - 2)
        avg_y2 = int((left_y2 + right_y2)/2 + 2)

        '''Find the distance between the centre
        points of the bounding box to get the width
        and height of the box'''

        avg_bbox_w = int(dist.euclidean((avg_x1,avg_y1), (avg_x2, avg_y1)))
        avg_bbox_h = int(dist.euclidean((avg_x1,avg_y1), (avg_x1, avg_y2)))

        if left_pupil or right_pupil is not None:

            try:

                ''' Average pupil center '''

                avg_pupil_x = int((right_pupil[0] + left_pupil[0])/2)
                avg_pupil_y = int((right_pupil[1] + left_pupil[1])/2)

                p_x_2_bbox = (avg_x1, avg_pupil_y)
                p_y_2_bbox = (avg_pupil_x, avg_y1)

                ''' Distance of the gaze coordinate from the left
                and top of the bounding box '''

                p_2_bbox_w_d = int(dist.euclidean((p_x_2_bbox), (avg_pupil_x, avg_pupil_y)))
                p_2_bbox_h_d = int(dist.euclidean((p_y_2_bbox), (avg_pupil_x, avg_pupil_y)))

                ''' Ratio of width and height of the gaze coordinate
                with respect to the bounding box '''

                p_x_2_bbox_w_r = avg_bbox_w/p_2_bbox_w_d
                p_y_2_bbox_h_r = avg_bbox_h/p_2_bbox_h_d

            except Exception:
                pass

            if p_x_2_bbox_w_r is not None and p_y_2_bbox_h_r is not None:
                return p_x_2_bbox_w_r, p_y_2_bbox_h_r
            else:
                return None
lock = threading.Lock()
class WorkerThread(threading.Thread):
    
    def __init__(self, threadID, name, cam, gaze, sock, sc_width, sc_height):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.cam = cam
        self.gaze = gaze
        self.sock = sock
        self.sc_width = sc_width
        self.sc_height = sc_height
        
    def run(self):
        
        while True:
            
            # Get a frame from the webcam
            frame = self.cam.read()
            self.gaze.refresh(frame)
            # Get the gaze coordinates
            p_xy = get_gaze_coords(frame)
            
            if p_xy is not None:

                # Estimate the screen coordinates
                if p_xy[0] != 0 and p_xy[1] != 0:
                    est_x = int((self.sc_width/p_xy[0]))
                    est_y = int((self.sc_height/p_xy[1]))
                else:
                    continue

                # Wait for half a second and check for blinking
                #time.sleep(0.5)
                if self.gaze.is_blinking:
                    blink = 1
                else:
                    blink = 0

                # Create the gaze coordinates packet
                gaze_coord = [est_x, est_y, blink]
                packet = ','.join(map(str, gaze_coord))

                # Use the lock to synchronize access to the shared variable
                with lock:
                    while True:
                        
                        print("trying")
                        self.sock.sendall(packet.encode("UTF-8"))
                        break


            frame = self.gaze.annotated_frame()

            cv2.namedWindow('Demo',cv2.WINDOW_NORMAL)
            cv2.imshow("Demo", frame)

            if cv2.waitKey(1) == 27:
                break

if __name__ == "__main__":

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host, port = "127.0.0.1", 25001

    # Get the screen resolution
    sc_width, sc_height = get_screen_res()

    # Create the webcam and gaze estimation objects
    cam = WebcamVideoStream(0).start()
    gaze = GazeEstimation()

    # Try to connect to the server and retry every second if connection is refused
    while True:
        try:
            print("trying")
            sock.connect((host, port))
            break
        except (ConnectionRefusedError,ConnectionResetError,ConnectionAbortedError):
            print("Connection refused. Retrying in 1 second...")
            time.sleep(1)

    # Create and start the worker thread
    worker_thread = WorkerThread(1, "WorkerThread", cam, gaze, sock, sc_width, sc_height)
    worker_thread.start()

    # Wait for the worker thread to finish
    worker_thread.join()

    # Stop the webcam and close all windows
    cam.stop()
    cv2.destroyAllWindows()