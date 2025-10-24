#!/usr/bin/env python3

# DUAL CAMERA LASER DETECTOR - WITH SOFTWARE FRAME SYNCHRONIZATION
# =================================================================
# This version attempts to synchronize frames from two standard USB cameras
# by timestamping them and matching pairs that are close in time.

import cv2
import numpy as np
import argparse
import serial
import time
import threading
from queue import Queue, Empty
import multiprocessing as mp
import math
from collections import deque




# phih = 0.2639
# phiv = 0.1985
# Nh = 320.0
# Nv = 240.0
# B = 40.0
# alpha2= 0.014
# cam1_coords= [342,225]
# cam2_coords= [271,210]

    
# print(f"[SYNC COORDS] CAM1: {cam1_coords} | CAM2: {cam2_coords}")

# n1h = (cam1_coords[0] - Nh)
# n1v = (-cam1_coords[1] + Nv)
# n2h = (cam2_coords[0] - Nh)
# n2v = (-cam2_coords[1] + Nv)

# if (n1h - n2h) != 0:

#     nv = (n1v + n2v) / 2
#     tgt = math.tan(phih) * n2h/Nh
#     if (1-math.tan(alpha2)*tgt != 0):
#         tgat = (math.tan(alpha2)+tgt)/(1-math.tan(alpha2)*tgt)
#         xz = math.tan(phih)*n1h/Nh

#         bz = xz - tgat
#         if (bz != 0):
#             laser_3d_z = B / bz
#             laser_3d_x = laser_3d_z * xz
#             laser_3d_y = laser_3d_z * nv * math.tan(phiv)/Nv
#             laser_3d_r = math.sqrt(laser_3d_x**2 + laser_3d_y**2 + laser_3d_z**2)
            
#             print(f"3D POSITION: X={laser_3d_x:.2f} Y={laser_3d_y:.2f} Z={laser_3d_z:.2f} R={laser_3d_r:.2f}")



# time.sleep(4)

# --- configure Arduino serial (adjust COM port!) ---
arduino = serial.Serial('COM4', 115200, timeout=1)  
time.sleep(2)  # wait for Arduino to reset

class SynchronizedDualCameraDetector:
    def __init__(self, camera1_id=0, camera2_id=1):
        self.camera1_id = camera1_id
        self.camera2_id = camera2_id

        self.last_sent_time = time.time()
        
        # Detection parameters
        self.brightness_threshold = 80
        
        # --- MODIFIED: Synchronization & Performance ---
        self.max_time_difference = 1/30  # Max timestamp difference for a pair (e.g., for 30 FPS)
        self.display_scale = 0.7
        self.max_queue_size = 5 # Allow a small buffer for synchronization
        
        # Thread control
        self.running = True
        
        # --- NEW: Re-architected queues for synchronization ---
        # Raw frames from each camera capture thread
        self.capture_queues = [Queue(maxsize=self.max_queue_size), Queue(maxsize=self.max_queue_size)]
        # Results ready for display
        self.display_queues = [Queue(maxsize=2), Queue(maxsize=2)]
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_timer = time.time()
        
        # 3D triangulation parameters
        self.phih = 0.2639
        self.phiv = 0.1985
        self.Nh = 320.0
        self.Nv = 240.0
        self.B = 0.0
        self.alpha1 = 0.0
        self.alpha2= 0.014

        
        # 3D position results
        self.laser_3d_x = 0.0
        self.laser_3d_y = 0.0
        self.laser_3d_z = 0.0
        self.laser_3d_r = 0.0

    # (detect_laser_fast remains the same as your original code)
    def detect_laser_fast(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([62, 70, 200])
        upper_green = np.array([75, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        v_channel = hsv[:, :, 2]
        brightness_mask = cv2.inRange(v_channel, 0, 255)
        combined_mask = cv2.bitwise_and(green_mask, brightness_mask)
        filtered = cv2.bitwise_and(frame, frame, mask=combined_mask)
        green_channel = filtered[:, :, 1] 
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(green_channel)
        if max_val > self.brightness_threshold:
            return max_loc, max_val, True, filtered
        else:
            return None, max_val, False, filtered

    # (calculate_3d_position remains the same, but is now called with synchronized coords)
    def calculate_3d_position(self, cam1_coords, cam2_coords):
        if self.B == 0.0:
            return
            
        print(f"[SYNC COORDS] CAM1: {cam1_coords} | CAM2: {cam2_coords}")
        
        n1h = (cam1_coords[0] - self.Nh)
        n1v = (-cam1_coords[1] + self.Nv)
        n2h = (cam2_coords[0] - self.Nh)
        n2v = (-cam2_coords[1] + self.Nv)
        
        if (n1h - n2h) != 0:

            nv = (n1v + n2v) / 2
            tgt = math.tan(self.phih) * n2h/self.Nh
            if (1-math.tan(self.alpha2)*tgt != 0):
                tgat = (math.tan(self.alpha2)+tgt)/(1-math.tan(self.alpha2)*tgt)
                xz = math.tan(self.phih)*n1h/self.Nh

                bz = xz - tgat
                if (bz != 0):
                    self.laser_3d_z = self.B / bz
                    self.laser_3d_x = self.laser_3d_z * xz
                    self.laser_3d_y = self.laser_3d_z * nv * math.tan(self.phiv)/self.Nv
                    self.laser_3d_r = math.sqrt(self.laser_3d_x**2 + self.laser_3d_y**2 + self.laser_3d_z**2)
                    
                    print(f"3D POSITION: X={self.laser_3d_x:.2f} Y={self.laser_3d_y:.2f} Z={self.laser_3d_z:.2f} R={self.laser_3d_r:.2f}")

    # --- MODIFIED: camera_capture_thread now timestamps frames ---
    def camera_capture_thread(self, camera_id, capture_queue):
        """Dedicated thread for camera capture. Timestamps every frame."""
        cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print(f"Error: Could not open camera {camera_id}")
            return
        
        # Set camera properties
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -11)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        print(f"Camera {camera_id} capture thread started")
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                # --- KEY CHANGE: Get timestamp and put tuple in queue ---
                timestamp = time.time()
                frame = cv2.flip(frame, 1)

                try:
                    # Put the frame and its timestamp into the queue
                    capture_queue.put((frame, timestamp), block=False)
                except:
                    # Drop frame if queue is full
                    pass
        finally:
            cap.release()
            print(f"Camera {camera_id} capture thread stopped")

    # --- NEW: The core synchronization logic ---
    def synchronizer_thread(self):
        """
        Pulls frames from both capture queues and finds time-matched pairs.
        This is the heart of the software synchronization.
        """
        # deque is efficient for adding to one end and removing from the other
        frame_buffers = [deque(), deque()]
        
        print("Synchronizer thread started")
        
        while self.running:
            # 1. Fill buffers from capture queues
            for i in range(2):
                try:
                    # Get all available frames to find the latest
                    while True:
                        frame, timestamp = self.capture_queues[i].get_nowait()
                        frame_buffers[i].append((frame, timestamp))
                except Empty:
                    pass # Queue is empty, which is normal

            # 2. Match frames from buffers
            # Work from the oldest frames in each buffer
            while frame_buffers[0] and frame_buffers[1]:
                # Peek at the oldest frames
                frame1, time1 = frame_buffers[0][0]
                frame2, time2 = frame_buffers[1][0]

                time_diff = abs(time1 - time2)

                if time_diff < self.max_time_difference:
                    # MATCH FOUND! Process this pair.
                    self.process_pair(frame1, frame2)
                    # Remove the matched frames from buffers
                    frame_buffers[0].popleft()
                    frame_buffers[1].popleft()
                elif time1 < time2:
                    # Frame 1 is too old, discard it and check again
                    frame_buffers[0].popleft()
                else:
                    # Frame 2 is too old, discard it and check again
                    frame_buffers[1].popleft()
            
            time.sleep(0.001) # Prevent busy-waiting




    # --- NEW: Centralized processing for a synchronized pair ---
    def process_pair(self, frame1, frame2):
        """Detects laser in a matched pair of frames and calculates 3D position."""
        start_time = time.time()
        
        # Detect in both frames
        pos1, bright1, detected1, mask1 = self.detect_laser_fast(frame1)
        pos2, bright2, detected2, mask2 = self.detect_laser_fast(frame2)

        # If detected in both, calculate 3D position
        if detected1 and detected2:
            self.calculate_3d_position(pos1, pos2)

        if detected1 and detected2:
            coords = f"{(pos1[0]+pos2[0])/2},{0}\n"
            arduino.write(coords.encode())
            print("Sent:", coords.strip())

        if detected1 and not detected2:
            coords = f"{(pos1[0])/2},{0}\n"
            arduino.write(coords.encode())
            print("Sent:", coords.strip())

        if detected2 and not detected1:
            coords = f"{(pos2[0]+640)/2},{0}\n"
            arduino.write(coords.encode())
            print("Sent:", coords.strip())



        
        
        detection_time = (time.time() - start_time) * 1000
        self.fps_counter += 1

        # Prepare results for display threads
        results = [
            {'frame': frame1, 'laser_pos': pos1, 'detected': detected1, 'green_mask': mask1, 'detection_time': detection_time},
            {'frame': frame2, 'laser_pos': pos2, 'detected': detected2, 'green_mask': mask2, 'detection_time': detection_time}
        ]

        for i in range(2):
            try:
                self.display_queues[i].put(results[i], block=False)
            except:
                pass

    def display_thread(self, camera_idx, result_queue):
        """Restored display thread with masks and crosshairs."""
        
        camera_id = self.camera1_id if camera_idx == 0 else self.camera2_id
        window_name = f'Synced CAM{camera_id} Detection + Mask'

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        display_width = int(640 * self.display_scale)
        display_height = int(480 * self.display_scale)
        cv2.resizeWindow(window_name, display_width * 2, display_height)

        while self.running:
            try:
                result = result_queue.get(timeout=0.1)

                frame = result['frame']
                laser_pos = result['laser_pos']
                detected = result['detected']
                detection_time = result['detection_time']
                green_mask = result['green_mask']

                # Draw detection circles and crosshairs
                if detected and laser_pos:
                    cv2.circle(frame, laser_pos, 8, (0, 255, 0), 2)
                    cv2.circle(frame, laser_pos, 2, (0, 0, 255), -1)
                    h, w = frame.shape[:2]
                    cv2.line(frame, (laser_pos[0], 0), (laser_pos[0], h), (0, 255, 0), 1)
                    cv2.line(frame, (0, laser_pos[1]), (w, laser_pos[1]), (0, 255, 0), 1)
                    # if camera_idx == 0:
                    #     # Send coordinates to ESP32
                    #     coords = f"{laser_pos[0]},{laser_pos[1]}\n"
                    #     arduino.write(coords.encode())
                    #     print("Sent:", coords.strip())

                    

                # Detection status
                status_color = (0, 255, 0) if detected else (0, 0, 255)
                cv2.putText(frame, f"CAM{camera_id} {'DETECTED' if detected else 'SEARCHING'}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                # FPS
                elapsed = time.time() - self.fps_timer
                if elapsed >= 1.0:
                    fps = self.fps_counter / elapsed
                    self.fps_counter = 0
                    self.fps_timer = time.time()
                    cv2.putText(frame, f"Synced FPS: {fps:.1f}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Resize if scaling is enabled
                if self.display_scale != 1.0:
                    frame = cv2.resize(frame, (display_width, display_height))
                    green_mask = cv2.resize(green_mask, (display_width, display_height))

                # Combine original + mask side by side
                combined = np.hstack((frame, green_mask))
                cv2.imshow(window_name, combined)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break



            except Empty:
                continue



        cv2.destroyWindow(window_name)



    # --- MODIFIED: Run method starts the new thread architecture ---
    def run(self, show_display=True):
        print("Starting detector with software frame synchronization...")
        
        threads = []

        # Start capture threads
        for i, camera_id in enumerate([self.camera1_id, self.camera2_id]):
            thread = threading.Thread(target=self.camera_capture_thread, args=(camera_id, self.capture_queues[i]))
            thread.daemon = True
            thread.start()
            threads.append(thread)
        
        # --- NEW: Start the single synchronizer thread ---
        sync_thread = threading.Thread(target=self.synchronizer_thread)
        sync_thread.daemon = True
        sync_thread.start()
        threads.append(sync_thread)

        # Start display threads (optional)
        if show_display:
            for i in range(2):
                thread = threading.Thread(target=self.display_thread, args=(i, self.display_queues[i]))
                thread.daemon = True
                thread.start()
                threads.append(thread)
        
        print("All threads started. Press 'q' in any window to quit.")
        
        # Keep main thread alive
        try:
            while self.running:
                time.sleep(0.1)
                # Check if any window was closed
                # if show_display and cv2.getWindowProperty(f'Synced CAM{self.camera1_id} Detection', cv2.WND_PROP_VISIBLE) < 1:
                #     self.running = False
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            # self.running = False
            print("Stopping all threads...")
            time.sleep(1) # Give threads time to exit cleanly
            cv2.destroyAllWindows()
            print("Session complete.")

def main():
    parser = argparse.ArgumentParser(description="Synchronized dual camera laser detector")
    # (Same arguments as before)
    parser.add_argument("-c1", "--camera1", type=int, default=1, help="First camera device id")
    parser.add_argument("-c2", "--camera2", type=int, default=2, help="Second camera device id")
    parser.add_argument("-t", "--threshold", type=int, default=80, help="Brightness threshold")
    parser.add_argument("--no-display", action="store_true", help="Disable display for maximum FPS")
    parser.add_argument("--scale", type=float, default=0.7, help="Display scale factor")
    parser.add_argument("--baseline", type=float, default=40, help="Distance between cameras in cm for 3D calculation")
    parser.add_argument("--fov-h", type=float, default=0.2639, help="Horizontal FOV angle in radians")
    parser.add_argument("--fov-v", type=float, default=0.1985, help="Vertical FOV angle in radians")
    parser.add_argument("--max-diff", type=float, default=1/30, help="Max time difference in seconds to consider frames synchronized")
    
    args = parser.parse_args()
    
    detector = SynchronizedDualCameraDetector(args.camera1, args.camera2)
    detector.brightness_threshold = args.threshold
    detector.display_scale = args.scale
    detector.max_time_difference = args.max_diff
    
    # Set 3D calculation parameters
    detector.B = args.baseline
    detector.phih = args.fov_h
    detector.phiv = args.fov_v
    
    show_display = not args.no_display
    detector.run(show_display)

if __name__ == "__main__":
    main()


