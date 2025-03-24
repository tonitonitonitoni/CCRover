import cv2
import pytesseract
from googletrans import Translator
from PIL import Image
import numpy as np
from pytesseract import Output
import matplotlib.pyplot as plt

translator = Translator()

# If needed, specify the tesseract binary path:
# pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'


frame=cv2.imread('Chinese-traditional-simplified-1.png')


# Convert frame to grayscale for potentially better OCR
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Optional preprocessing (e.g., thresholding) for better OCR results
# _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
# ocr_frame = binary
ocr_frame = gray

# Perform OCR with bounding box data
data = pytesseract.image_to_data(ocr_frame, lang='chi_sim', output_type=Output.DICT)

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
            result = translator.translate(word, src='zh-cn', dest='en')
            translated_word = result.text
        except Exception as e:
            translated_word = "Error"

        # Draw translated (English) text just below the original text’s position
        # Since y indicates top of the text box, put translated text slightly below
        cv2.putText(frame, translated_word, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        # Optionally, draw a bounding box around the text (for debugging)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.imshow('chinese',frame)
