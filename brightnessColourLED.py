import cv2
import numpy as np

# Function to detect blue and red LEDs with color *and* light level thresholding
def detect_led(frame):
    # Convert the frame to HSV (Hue, Saturation, Value) color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Convert the frame to grayscale to check light level (brightness)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Define range for blue color in HSV space
    lower_blue = np.array([100, 150, 150])  # Lower bound of blue
    upper_blue = np.array([140, 255, 255])  # Upper bound of blue
    
    # Define range for red color in HSV space
    lower_red1 = np.array([0, 150, 150])    # Lower bound of red (lower half)
    upper_red1 = np.array([10, 255, 255])   # Upper bound of red (lower half)
    lower_red2 = np.array([170, 150, 150])  # Lower bound of red (upper half)
    upper_red2 = np.array([180, 255, 255])  # Upper bound of red (upper half)
    
    # Create masks for blue and red LEDs
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    # Combine the two red masks
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # Combine blue and red masks
    mask_color = cv2.bitwise_or(mask_blue, mask_red)
    
    # Apply brightness threshold on grayscale image to filter low-light areas
    _, mask_brightness = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    
    # Combine color mask and brightness mask to detect bright colored LEDs
    mask = cv2.bitwise_and(mask_color, mask_brightness)
    
    # Find contours of the detected LEDs
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
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
    
    # Detect blue and red LEDs in the current frame
    frame_with_led = detect_led(frame)
    
    # Display the frame with detected LEDs
    cv2.imshow("LED Detection", frame_with_led)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
