import struct
from dataclasses import dataclass
import serial
import numpy as np
from threading import Thread
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from booblik.utils import get_directory, load_config
from booblik.logging_config import setup_logging
import logging


START_SEQUENCE = bytearray([0xCE, 0xFA])

@dataclass
class LidarConfig:
    """Конфигурация для подключения к лидару."""
    port: str   # Порт подключения
    baudrate: int   # Скорость передачи данных
    frequency: int   # Частота вращения измерений лидара в Гц
    range_min: float  # Минимальное расстояние сканирования
    range_max: float  # Максимальное расстояние сканирования

class LidarNode(Node):
    def __init__(self, config, name="lidar"):
        super().__init__(name)
        self.config = LidarConfig(**config) 
        setup_logging(log_filename='Lidar', date=True)
        self.logger = logging.getLogger('Lidar')

        self.lidar_publisher = self.create_publisher(LaserScan, "/booblik/sensors/lidar", 10)
        
        self.initialize_lidar()

        self.running = True
        self.thread = Thread(target=self._read_loop, daemon=True)  # Запуск потока для чтения данных с лидара
        self.thread.start()

    def initialize_lidar(self):
        """Устанавливаем частоту вращения лидара (в RPM) и включаем его"""
        self.ser = serial.Serial(self.config.port, self.config.baudrate) 
        self.ser.write("LSRPM:{:d}H\n\r".format(self.config.frequency * 60).encode("ascii"))
        self.ser.write("LOCONH\n\r".format(self.config.frequency * 60).encode("ascii"))

    def _read_loop(self):
        """Поток для чтения данных с лидара и их публикации."""
        # Кольцевой буфер для чтения пакетов байт с последовательного порта
        buffer = bytearray()
        while self.running:
            buffer.extend(self.ser.read(256))  # Чтение данных из порта
            # Поиск начала пакета данных
            index = buffer.find(START_SEQUENCE, 1)
            while index != -1 and index:
                packet = buffer[0:index]
                self._parse_packet(packet)
                buffer = buffer[index:]
                index = buffer.find(START_SEQUENCE, 1)

    def _parse_packet(self, packet: bytearray):
        """Обработка пакета данных от лидара."""
        # Проверка на минимально допустимую длину пакета
        if len(packet) <= 10:
            self.logger.error(f'Package length is too low: {len(packet)}')
            print(f'Package length is too low: {len(packet)}')
            return

        # Распаковка начала пакета, его длины и угла в формате "unsigned short"
        [_, length, angle] = struct.unpack("=HHH", packet[0:6])
        # Проверка, что длина пакета соответствует ожидаемой
        if len(packet) == 8 + length * 2:
            # Распаковка данных из пакета. Формат строки определяется длиной данных.
            data = struct.unpack("={}H".format(length), packet[6:-2])
            # Извлечение контрольной суммы из последних двух байт пакета
            (crc_in,) = struct.unpack("=H".format(length), packet[-2:])
            distances = np.zeros((length))
            strengths = np.zeros((length))
            # Расчет углов для каждого измерения
            angles = np.radians(
                np.linspace(angle / 10, angle / 10 + 36, length, endpoint=False)
            )
            # Вычисление контрольной суммы для проверки целостности данных
            crc_calc = angle + length
            for i in range(length):
                crc_calc += data[i]
                crc_calc %= 65536
                # Расчет расстояния и интенсивности сигнала
                # Маскирование и масштабирование данных расстояния
                distances[i] = (data[i] & 0x1FFF) / 100
                # Извлечение интенсивности сигнала
                strengths[i] = (data[i] & 0xE000) >> 13
            if crc_in != crc_calc:  # Проверка контрольной суммы
                self.logger.error(f'CRC error in lidar packet')
                print(f'CRC error in lidar packet')
                return
            else:
                # Создание объекта LaserScan для публикации данных
                laser_scan = LaserScan()
                laser_scan.angle_min = angles[0]  # Минимальный угол сканирования
                laser_scan.angle_max = angles[-1]  # Максимальный угол сканирования
                laser_scan.angle_increment = angles[1] - angles[0]  # Шаг угла сканирования
                laser_scan.scan_time = 1.0 / self.config.frequency  # Время сканирования
                laser_scan.time_increment = 1.0 / self.config.frequency / 10 / \
                    length  # Временной интервал между измерениями
                laser_scan.range_min = self.config.range_min
                laser_scan.range_max = self.config.range_max
                laser_scan.ranges = distances.tolist()  # Список измеренных расстояний
                laser_scan.intensities = strengths.tolist()  # Список интенсивностей

                self.lidar_publisher.publish(laser_scan)
    
    def stop(self):
        self.running = False
        self.thread.join()


def main(args=None):
    rclpy.init(args=args)
    booblik_dir = get_directory(target="booblik")
    config_file = os.path.join(booblik_dir, "config.json")
    config = load_config(config_file)
    task = LidarNode(config["lidar"])
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        pass
    finally:
        task.stop()
        task.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()