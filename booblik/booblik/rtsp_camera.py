import cv2
import numpy as np
from cv_bridge import CvBridge  # Библиотека для конвертации между ROS Image и OpenCV Image

from dataclasses import dataclass
from threading import Thread
import os
from datetime import datetime

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


@dataclass #декоратор
class CameraConfig:
    """Информация для подключения к камере через RTSP"""
    login: str
    password: str
    ip: str
    port: str

class CameraNode(Node):
    def __init__(self, node_name: str, config: CameraConfig) -> None:
        super().__init__(node_name)
        self.config = config
        self.homedir = os.path.join(os.getcwd(), 'images')   # папка для сохранения изображений
        
        if not os.path.exists(self.homedir):
            os.makedirs(self.homedir)

        # Создание издателя для публикации данных с камеры
        self.camera_ = self.create_publisher(
            Image,
            '/booblik/sensors/cameras/camera/image_raw',
            10)
        self.cv_bridge = CvBridge()  # Объект для конвертации между OpenCV и ROS Image


    def start(self):
        """Запуск потока для непрерывного чтения данных с камеры."""
        Thread(target=self._readLoop, daemon=False).start()

    def _readLoop(self):
        """Чтение и публикация данных с камеры"""
        link = f"rtspsrc location=rtsp://\
            {self.config.login}:{self.config.password}@{self.config.ip}:{self.config.port}\
            /ISAPI/Streaming/Channels/102 latency=50 ! decodebin ! videoconvert ! appsink"
        
        # Инициализация захвата видео с использованием GStreamer
        cap = cv2.VideoCapture(link, cv2.CAP_GSTREAMER)

        while True:
            ok, img = cap.read()  # Чтение кадра
            if ok:
                self._push_image(img)  # Публикация кадра, если чтение прошло успешно
                self._save_image(img)   # Сохранение кадра
            else:
                print('Ошибка чтения')
                continue

    def _push_image(self, img: np.ndarray):
        """Конвертация и публикация изображения в ROS топик."""
        ros2_img = self.cv_bridge.cv2_to_imgmsg(cvim=img, encoding='passthrough')
        self.camera_.publish(ros2_img)  # Публикация изображения
    
    def _save_image(self, img: np.ndarray):
        """Сохранение изображения с именем файла в формате временной метки."""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        cv2.imwrite(os.path.join(self.homedir, f"{timestamp}.jpg"), img)

def main(args=None):
    rclpy.init(args=args)
    cameraConfig = CameraConfig('admin', 'a123456789', '192.168.13.64', '554')
    task = CameraNode('camera', cameraConfig)
    task.start()
    rclpy.spin(task)
    rclpy.shutdown()