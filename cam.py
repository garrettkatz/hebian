import matplotlib.pyplot as pt
import sys
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")

ret, frame = cap.read()  # Read a frame from the webcam
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

if not ret:
    print("Error: Failed to capture frame.")

else:
    # cv2.imshow('Webcam Feed', frame)  # Display the frame
    print(frame[:,:,::-1])
    pt.imshow(frame)
    pt.show()

