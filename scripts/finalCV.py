import cv2
import pytesseract
from googletrans import Translator
from PIL import Image
import numpy as np
from pytesseract import Output

translator = Translator()

# If needed, specify the tesseract binary path:
# pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

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
    ocr_frame = gray

 # Perform OCR with bounding box data
    data = pytesseract.image_to_data(ocr_frame, lang='rus', output_type=Output.DICT)

    # Loop through each detected element
    for i in range(len(data['text'])):
        word = data['text'][i].strip()
        conf = int(data['conf'][i])

        # Only consider words with a reasonable confidence
        if word and conf > 50:
            x = data['left'][i]
            y = data['top'][i]
            w = data['width'][i]
            h = data['height'][i]

            # Translate the recognized word (or you can accumulate words line-wise)
            try:
                result = translator.translate(word, src='ru', dest='en')
                translated_word = result.text
            except Exception as e:
                translated_word = "Error"

            # Draw translated (English) text just below the original text’s position
            # Since y indicates top of the text box, put translated text slightly below
            cv2.putText(frame, translated_word, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

            # Optionally, draw a bounding box around the text (for debugging)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
