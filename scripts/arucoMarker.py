import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Turn on the camera
cap=cv2.VideoCapture(-1)
while True:
  ret, frame=cap.read()
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  # Detect the markers
  corners, marker_ids, rejected = detector.detectMarkers(gray)

  if marker_ids is not None:
      cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
      
      # write the number on the frame
      for i, marker_id in enumerate(marker_ids):
        marker_textlocn = corners[i][0][1]
        m_loc = int(marker_textlocn[0]), int(marker_textlocn[1]-50)
        marker_msg=f'ID:{marker_id}'
        cv2.putText(frame, marker_msg, m_loc, cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 255), 2)
  cv2.imshow('frame', frame)
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()
