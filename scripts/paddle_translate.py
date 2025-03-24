from paddleocr import PaddleOCR
from googletrans import Translator
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def translate_video_feed(frame, ocr, translator):
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
        cv2.putText(frame, translated_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return translated_text


class ImageTranslator(Node):
    def __init__(self):
        super().__init__('image_translator')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        
        self.translator = Translator()
        self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        
        text = translate_video_feed(frame, self.ocr, self.translator)
        self.get_logger().info(f'Translated message {text}')
        
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageTranslator()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
