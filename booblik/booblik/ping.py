from dataclasses import dataclass
from brping import Ping1D  # Импорт библиотеки для работы с эхолотом Ping1D
from threading import Thread
import rclpy
from rclpy.node import Node
from booblik_msg.msg import Echo1d  # Импорт пользовательского типа сообщения для ROS2
import time
from datetime import datetime

@dataclass
class PingConfig:
    """Конфигурация для эхолота Ping1D."""
    port: str  # Серийный порт подключения
    baudrate: int  # Скорость передачи данных
    period: float  # Период между измерениями в миллисекундах
    speed_of_sound: float  # Скорость звука в воде в м/с


class Ping1dNode(Node):
    """Узел ROS2 для считывания данных с эхолота Ping1D."""
    config: PingConfig

    def __init__(self, name='echo_ping'):
        super().__init__(name)
        self.config = PingConfig(
            '/dev/ttyUSB0',  # рекомендуется проверять порт подключения
            115200,
            500,
            1490
        )
        # Создание издателя для публикации данных эхолота
        self.echo1d_ = self.create_publisher(
            Echo1d,
            '/booblik/sensors/echo1d',
            10)

        # Запуск потока для чтения и публикации данных эхолота
        Thread(target=self._readLoop, daemon=True).start()

    def _readLoop(self):
        """Чтение данных с эхолота и публикация в ROS."""
        ping = Ping1D()
        ping.connect_serial(self.config.port, self.config.baudrate)
        if ping.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)
        # Установка скорости звука для корректных измерений
        ping.set_speed_of_sound(self.config.speed_of_sound * 1000)

        while True:
            start = datetime.now()
            data = ping.get_distance()  # Получение данных от эхолота
            # Создание сообщения ROS с данными от эхолота
            msg = Echo1d()
            msg.distance = data["distance"]
            msg.confidence = data["confidence"]
            msg.scan_start = data["scan_start"]
            msg.scan_length = data["scan_length"]
            msg.transmit_duration = data["transmit_duration"]
            msg.gain_setting = data["gain_setting"]

            # Публикация сообщения
            self.echo1d_.publish(msg)
            # Логирование полученных данных 
            self.get_logger().info(
                f"Distance: {data['distance']}\t"
                f"Confidence: {data['confidence']}%"
            )
            # Пауза до следующего измерения
            time.sleep(self.config.period/1000)



def main(args=None):
    rclpy.init(args=args)
    task = Ping1dNode()  # Создание экземпляра узла
    rclpy.spin(task)  # Запуск узла
    rclpy.shutdown()  # Завершение работы узла


if __name__ == '__main__':
    main()
