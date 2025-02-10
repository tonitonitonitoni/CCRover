import cv2  # state of the art computer vision algorithms library
import numpy as np  # fundamental package for scientific computing
import matplotlib.pyplot as plt  # 2D plotting library producing publication quality figures
import pyrealsense2 as rs

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Import colorizer for depth frames
#colorizer = rs.colorizer()
# Start streaming
profile=pipe.start(cfg)
tracker = cv2.TrackerCSRT_create()
for x in range(10):
    pipe.wait_for_frames()
frameset = pipe.wait_for_frames()
frame = frameset.get_color_frame()
frame = np.asanyarray(frame.get_data())
bbox = cv2.selectROI("Select object", frame, fromCenter=False, showCrosshair=True)
tracker.init(frame,bbox)

try:
    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frameset = pipe.wait_for_frames()
        # depth_frame = frameset.get_depth_frame()
        color_frame = frameset.get_color_frame()
        if not color_frame: continue
        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        color_frame = frameset.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())

        ret, bbox = tracker.update(frame)

        if ret:
            x,y,w,h=[int(v) for v in bbox]
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)

            aligned_depth_frame = frameset.get_depth_frame()
            depth = np.asanyarray(aligned_depth_frame.get_data())

            # Get data scale from the device and convert to meters
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth = depth * depth_scale
            dist, _, _, _ = cv2.mean(depth)
            print(f"C:Detected object {dist:0.4f} meters away.")

        else:
            cv2.putText(frame, "Tracking failed", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)


        # Display output image
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', frame)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    # Stop streaming
    pipe.stop()