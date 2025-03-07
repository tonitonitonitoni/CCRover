from paddleocr import PaddleOCR
from googletrans import Translator
import cv2
translator = Translator()
ocr = PaddleOCR(use_angle_cls=True, lang='ch')
#to use the camera
#video_source=0
#cap = cv2.VideoCapture(video_source)

def translate_video_feed(frame):
# Step 1: OCR
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = ocr.ocr(rgb_frame, cls=True)
    extracted_text = ""
    for line in result:
        for word in line:
            if isinstance(word, list):
                extracted_text += word[1][0]

# Step 2: Translation
    if extracted_text:
        translated_text = translator.translate(extracted_text, src='zh-CN', dest='en').text
        print("Translated Text:", translated_text)


frame = cv2.imread("tradChi2.jpg")
translate_video_feed(frame)

#while True:
#    ret, frame = cap.read()
#    if not ret:
#        break
#    translate_video_feed(frame)
#    cv2.imshow("video feed", frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
