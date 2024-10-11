from dataclasses import dataclass
from brping import Ping1D  # Импорт библиотеки для работы с эхолотом Ping1D
import rclpy
from rclpy.node import Node
from booblik_msg.msg import Echo1d  # Импорт пользовательского типа сообщения для ROS2
import time
import os
from booblik.utils import get_directory, load_config
from booblik.logging_config import setup_logging
import logging


@dataclass
class PingConfig:
    """Конфигурация для эхолота Ping1D."""
    port: str  # Серийный порт подключения
    baudrate: int  # Скорость передачи данных
    period: float = 500 # Период между измерениями в миллисекундах
    speed_of_sound: float = 1490  # Скорость звука в воде в м/с


class Ping1dNode(Node):
    def __init__(self, config, name='echo_ping'):
        super().__init__(name)
        self.config = PingConfig(
            config['port'],  # порт подключения в соответствии с конфигом
            config['baudrate']
        )

        setup_logging(log_filename='Echo_sounder', date=True)
        self.logger = logging.getLogger('Echo_sounder')

        self.echo1d_publisher = self.create_publisher(
            Echo1d,
            '/booblik/sensors/echo1d',
            10)
        
        self.initialize_ping()

        # Установка таймера для периодического вызова функции 
        self.timer = self.create_timer(self.config.period / 1000, self._read_loop)

    def initialize_ping(self):
        """Инициализация эхолота с учетом конфигурации"""
        self.ping = Ping1D()
        self.ping.connect_serial(self.config.port, self.config.baudrate)
        if not self.ping.initialize():
            self.logger.error(f'Failed to initialize Ping!')
            print(f'Failed to initialize Ping!')
            exit(1)
        # Установка скорости звука для корректных измерений
        self.ping.set_speed_of_sound(self.config.speed_of_sound * 1000)

    def _read_loop(self):
        """Чтение данных с эхолота и публикация в ROS."""
        data = self.ping.get_distance()  # Получение данных от эхолота
        
        if data:
            msg = Echo1d()
            msg.distance = data['distance']
            msg.confidence = data['confidence']
            msg.scan_start = data['scan_start']
            msg.scan_length = data['scan_length']
            msg.transmit_duration = data['transmit_duration']
            msg.gain_setting = data['gain_setting']

            self.echo1d_publisher.publish(msg)
            # Логирование полученных данных 
            self.logger.info(f"Distance: {data['distance']}\t Confidence:{data['confidence']}%")


def main(args=None):
    rclpy.init(args=args)
    booblik_dir = get_directory(target='booblik')
    config_file = os.path.join(booblik_dir, 'config.json')
    config = load_config(config_file)  # Загрузка конфигурации
    task = Ping1dNode(config['ping'])
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        pass
    finally:
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()