import cv2
import numpy as np

# Function to detect LED based only on BRIGHTNESS

def detect_led(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply a Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)
    
    # Apply thresholding to detect bright spots (LED)
    _, thresholded = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
    
    # Find contours of the bright regions
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours on the original frame
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Filter small contours (noise)
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Draw a rectangle around the detected LED
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame

# Open the video feed
cap = cv2.VideoCapture(-1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect LED in the current frame
    frame_with_led = detect_led(frame)
    
    # Display the frame with detected LEDs
    cv2.imshow("LED Detection", frame_with_led)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
