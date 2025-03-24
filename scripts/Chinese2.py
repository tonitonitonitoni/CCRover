import cv2
import pytesseract
from googletrans import Translator
from PIL import Image
import numpy as np
from pytesseract import Output

translator = Translator()

# If needed, specify the tesseract binary path:
# pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

cap = cv2.VideoCapture(-1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale for potentially better OCR
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Optional preprocessing (e.g., thresholding) for better OCR results
    # _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    # ocr_frame = binary
    ocr_frame = gray

    # Perform OCR with bounding box data
    data_hor = pytesseract.image_to_data(ocr_frame, lang='chi_sim', output_type=Output.DICT)
    data_ver = pytesseract.image_to_data(ocr_frame, lang='chi_sim_vert', output_type=Output.DICT)
    
    
    combined_text = []
    
    def process_ocr_data(data):
        for i in range(len(data['text'])):
            word = data['text'][i].strip()
            conf = int(data['conf'][i])
            if word and conf > 30:  # Only accept words with high confidence
                x, y, w, h = data['left'][i], data['top'][i], data['width'][i], data['height'][i]
                combined_text.append((word, x, y, w, h))

    # Process both OCR results
    process_ocr_data(data_hor)
    process_ocr_data(data_ver)

    # Merge detected words into full lines before translation
    lines = []
    line_text = ""

    # Sort words by vertical position (y) to group them into lines
    combined_text.sort(key=lambda item: item[2])

    for i, (word, x, y, w, h) in enumerate(combined_text):
        line_text += word + " "

        # If the next word is significantly lower, treat it as a new line
        if i == len(combined_text) - 1 or abs(combined_text[i + 1][2] - y) > 10:
            lines.append(line_text.strip())
            line_text = ""

    # Translate each full line instead of word-by-word
    for line in lines:
        try:
            result = translator.translate(line, src='zh-CN', dest='en')
            translated_text = result.text
            print(f"Original: {line} -> Translated: {translated_text}")

            # Draw translated text onto the frame
            cv2.putText(frame, translated_text, (50, 50 + 30 * lines.index(line)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        except Exception as e:
            print(f"Translation Error: {e}")

    # Show the frame with translated text
    cv2.imshow('CHINESE TRANSLATION - SIMPLIFIED & VERTICAL', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
