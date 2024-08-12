from dataclasses import dataclass
import serial
from threading import Thread, Lock
import rclpy
import time
import argparse
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Тип сообщения ROS для данных о местоположении
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
import pynmea2  # Библиотека для разбора данных в формате NMEA, получаемых от GPS 

import logging 
DEBUG = True

compas_imported = False
try:
    from .QMC5883L import QMC5883LNode
    compas_imported = True
except ImportError as ie:
    print('Cannot import compas node: ', ie)

# Настройка логгера
logging.basicConfig(
            level=logging.DEBUG if DEBUG else logging.info,
            format='%(asctime)s [%(name)-8.8s] [%(levelname)-5.5s] %(message)s',
            filename='gps_log.log',
            filemode='a'
        )

def kmph_2_mps(kmph):
    return kmph / 3.6  # Конвертация км/ч в м/с

def knot_2_mps(knots):
    return knots * 0.514444  # Конвертация узлы в м/с

@dataclass
class GpsConfig:
    """Конфигурация подключения к GPS модулю."""
    port: str  # Порт, к которому подключен GPS
    baudrate: int  # Скорость передачи данных

class GpsImuNode(Node):
    config: GpsConfig

    def __init__(self, name='ws_m181'):
        super().__init__(name)
        self.config = GpsConfig('/dev/serial0', 115200)  # Настройки подключения к GPS
        self.latitude = None
        self.longitude = None
        self.satellites = None
        self.ground_speed = None
        # Доступные `sentence_type`: GLL, GSV, GGA, GSA, VTG
        self.parse_sentence = {
            'GGA': self.parseGGA,  
            'VTG': self.parseVTG,
            'RMC': self.parseRMC,
        }
        # Создание издателя для публикации данных о местоположении
        self.nav_ = self.create_publisher(NavSatFix, '/booblik/sensors/gps/navsat/fix', 10)
        self.odometry_ = self.create_publisher(Odometry, '/booblik/sensors/position/ground_truth_odometry', 10)
        self.satellites_ = self.create_publisher(UInt32, '/booblik/sensors/gps/satellites', 10)
        
        # 
        self.serial_lock = Lock()
        self.serial = None
        self.lock = Lock()
        
        self.logger = logging.getLogger('GPS') # Создание логгера для GPS данных
        
        Thread(target=self._updateLoop, daemon=True).start()  # Запуск потока для обновления соединения
        Thread(target=self._readLoop, daemon=True).start()  # Запуск потока для чтения данных с GPS
        Thread(target=self._publishLoop, daemon=True).start()  # Запуск потока для публикации данных

    def _publishLoop(self):
        """Цикл для публикации данных."""
        while True:
            self.publishData()
            time.sleep(0.05)

    def _updateLoop(self):
        """Цикл для обновления соединения с GPS модулем."""
        while True:
            with self.serial_lock:
                if self.serial:
                    self.serial.close()  # Закрываем предыдущий порт для предотвращения утечек ресурсов и конфликтов
                self.serial = serial.Serial(self.config.port, self.config.baudrate, timeout=3)
            time.sleep(2)
            print('updated connection')

    def _readLoop(self):
        """Цикл для чтения данных с GPS и их обработки."""
        while True:
            time.sleep(0.05)
            if self.serial is None:  # Проверяем, что объект последовательного порта существует и открыт
                continue
            with self.serial_lock:
                try:
                    raw_data = self.serial.readline() 
                    self.logger.debug(f'Raw data: {raw_data}\n') # Запись сырых данных в лог
                    raw_data = raw_data.decode('utf-8')  # Чтение строки данных
                    if not raw_data:  # Проверяет, что данные были успешно прочитаны из последовательного порта
                        continue  # Пропуск пустых строк
                    data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                    self.logger.info(f'Parsed data: {data}\n') # Запись разобранных данных в файл
                    if data.sentence_type in self.parse_sentence:
                        self.parse_sentence[data.sentence_type](data)
                    self.serial.reset_input_buffer()  # Очистка буфера ввода
                except UnicodeDecodeError as e: 
                    self.logger.error(f'UnicodeDecodeError: {e}\n')
                    continue  # Пропуск строки при ошибке декодирования
                except pynmea2.nmea.ParseError as e:
                    self.logger.error(f'NMEA ParseError: {e}\n') 
                    continue  # Пропуск строки при ошибке парсинга
                except Exception as e:
                    self.logger.error(f'Exception: {e}\n') 

    def publishData(self):
        """Публикация данных."""
        print({attr: getattr(self, attr) for attr in ['latitude', 'longitude', 'satellites', 'ground_speed']})
        with self.lock:
            if self.satellites is not None:
                self.publishOdometry()
                self.publishNavSatFix()
                self.publishSallelites()

    def publishOdometry(self):
        """Публикация данных одометрии."""
        msg = Odometry()
        if self.ground_speed is not None:
            msg.twist.twist.linear.x = self.ground_speed
            self.odometry_.publish(msg)

    def publishNavSatFix(self):
        """Публикация данных GPS."""
        nav = NavSatFix()
        if self.latitude is not None and self.longitude is not None:
            nav.latitude = self.latitude
            nav.longitude = self.longitude
            self.nav_.publish(nav)

    def publishSallelites(self):
        """Публикация количества спутников."""
        msg = UInt32()
        if self.satellites is not None:
            msg.data = self.satellites
            self.satellites_.publish(msg)

    def parseGGA(self, data):
        """Обработка данных GGA."""
        with self.lock:
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.satellites = int(data.num_sats)

    def parseVTG(self, data):
        """Обработка данных VTG."""
        with self.lock:
            self.ground_speed = kmph_2_mps(data.spd_over_grnd_kmph)

    def parseRMC(self, data):
        """Обработка данных RMC."""
        with self.lock:
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.ground_speed = knot_2_mps(data.spd_over_grnd_kmph)   #Скорость в узлах

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-compas', action='store_true')
    return parser.parse_args()

def main(args=None):
    rclpy.init(args=None)
    args = parse_args()
    print(args)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(GpsImuNode())
    if args.use_compas:
        executor.add_node(QMC5883LNode())
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main(parse_args())
