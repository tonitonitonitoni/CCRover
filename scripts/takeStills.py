import cv2

# Open the webcam (default camera is usually 0)
cap = cv2.VideoCapture(-1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not access the camera.")
    exit()
i=2
print(i)
while True:
    # Capture a frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to capture image.")
        break
    
    # Display the frame
    cv2.imshow('Webcam', frame)
    
    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF
    
    # If the spacebar (ASCII code 32) is pressed, take a picture
    if key == 32:
        cv2.imwrite(f'still_image_{i}.jpg', frame)  # Save the image
        print(f"Image captured and saved as 'captured_image_{i}.jpg'.")
        i+=1
    # If the 'q' key is pressed, exit the loop
    elif key == ord('q'):
        print("Exiting...")
        break

# Release the webcam
cap.release()

# Close any OpenCV windows
cv2.destroyAllWindows()
