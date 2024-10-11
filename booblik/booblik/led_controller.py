import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge  # Библиотека для конвертации между ROS Image и OpenCV Image 
import logging 
import logging_config
from rpi_ws281x import *

# Конфигурация светодиодов для левого и правого модулей
LED_COUNT = 260  # Количество светодиодов в ленте
LED_PIN_LEFT = 18  # Пин для левого модуля
LED_PIN_RIGHT = 21  # Пин для правого модуля
LED_FREQ_HZ = 800000  # Частота сигнала
LED_DMA = 10  # DMA канал
LED_BRIGHTNESS = 125  # Яркость (0-255)
LED_INVERT = False  # Инвертировать сигнал при использовании NPN транзистора
LED_CHANNEL = 0  # GPIO канал

width = 26  # Ширина матрицы
height = 10  # Высота матрицы

# Инициализация светодиодных лент
strip_left = Adafruit_NeoPixel(LED_COUNT, LED_PIN_LEFT, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip_right = Adafruit_NeoPixel(LED_COUNT, LED_PIN_RIGHT, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip_left.begin()
strip_right.begin()
    
def resize(old_size, frame):
    proportion = max(old_size[0] / frame[0], old_size[1] / frame[1])
    return (int(old_size[0] / proportion), int(old_size[1] / proportion))

def modified_image(image, new_height = height, new_width=width ):
    old_height, old_width = image.shape[:2] 
    new_size = resize((old_height, old_width), (new_height, new_width) ) 
    resized_image = cv2.resize(image, (new_size[1], new_size[0]), interpolation = cv2.INTER_AREA)
    new_image = np.zeros((new_height, new_width, 3), dtype="uint8")
    start_y = (new_height - new_size[0]) // 2
    start_x = (new_width - new_size[1]) // 2 
    # Размещаем масштабированное изображение на новом холсте
    new_image[start_y:start_y + new_size[0], start_x:start_x + new_size[1]] = resized_image
    return new_image

class LedControllerNode(Node):
    def __init__(self, name='led_controller'):
        super().__init__(name)
        logging_config.setup_logging(log_filename='led_controller')  # Настройка логгера с использованием имени ноды
        self.logger = logging.getLogger('LED') # Создание логгера для данных 

        self.left_LED_sub_ = self.create_subscription(
            Image,
            '/booblik/sensors/LED/left',
            lambda msg: self.led_callback(msg, strip_left, is_right=False),
            10
        )
        self.right_LED_sub_ = self.create_subscription(
            Image,
            '/booblik/sensors/LED/right',
            lambda msg: self.led_callback(msg, strip_right, is_right=True),
            10
        )
 
        # Используем CvBridge для преобразования между ROS Image и OpenCV
        self.bridge = CvBridge()

    def led_callback(self, msg, strip, is_right):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Проверка размера изображения
            if cv_image.shape[1] != width or cv_image.shape[0] != height:
                # Масштабируем изображение под размер матрицы (26x10)
                resized_image = modified_image(cv_image)
                self.logger.info(f'Изображение масштабировано до {width}x{height} пикселей')
            else:
                resized_image = cv_image  # Размер соответствует, не требуется масштабирование

            # Отображаем изображение на ленте
            self.display_image(strip, resized_image, is_right)
        except Exception as e:
            self.logger.error(f"Ошибка отображения на модуле: {e}")

    def display_image(self, strip, image_matrix, is_right):
        for y in range(height):
            for x in range(width):
                led_index = self.get_led_index(x, y, is_right)
                color = image_matrix[y, x]  # Используем двумерный массив с RGB значениями
                strip.setPixelColor(led_index, Color(color[0], color[1], color[2]))
        strip.show()
                
    def get_led_index(self, x, y, width, height, is_right):
        '''Вспомогательная функция для вычисления индекса светодиода (с учётом направления "змеевидной" матрице )'''
        if is_right:
            # Для правой ленты (начинаем с правого нижнего угла)
            y = height - 1 - y
            if y % 2 != 0:
                return y * width + x
            else:
                return y * width + (width - 1 - x)
        else:
            # Для левой ленты (начинаем с верхнего левого угла)
            if y % 2 == 0:
                return y * width + x
            else:
                return y * width + (width - 1 - x)


def main(args=None):
    rclpy.init()
    task = LedControllerNode(args=args)
    rclpy.spin(task)
    rclpy.shutdown()
    

if __name__ ==  '__main__':
    try:
        main() 
    except KeyboardInterrupt:
        for i in range(width*height):
            strip_left.setPixelColor(i, Color(0, 0, 0))
            strip_right.setPixelColor(i, Color(0, 0, 0))
        strip_left.show()
        strip_right.show()