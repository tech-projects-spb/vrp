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
from datetime import datetime

compas_imported = False
try:
    from .QMC5883L import QMC5883LNode
    compas_imported = True
except ImportError as ie:
    print('Cannot import compas node: ', ie)

def kmph_2_mps(kmph):
    return kmph / 3.6  # Конвертация км/ч в м/с

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
        self.data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'satellites': 0,
            'ground_speed': 0.0,
        }
        # Доступные `sentence_type`: GLL, GSV, GGA, GSA, VTG
        self.parse_sentence = {
            "GGA": self.parseGGA,  
            'VTG': self.parseVTG,
        }
        # Создание издателя для публикации данных о местоположении
        self.nav_ = self.create_publisher(NavSatFix, '/booblik/sensors/gps/navsat/fix', 10)
        self.odometry_ = self.create_publisher(Odometry, '/booblik/sensors/position/ground_truth_odometry', 10)
        self.satellites_ = self.create_publisher(UInt32, '/booblik/sensors/gps/satellites', 10)
        self.serial_lock = Lock()
        self.serial = None
        self.lock = Lock()
        log_filename = f'/tmp/gps_data{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        self.log_file = open(log_filename, 'a')  # Открытие файла для записи данных
        
        Thread(target=self._updateLoop, daemon=True).start()  # Запуск потока для обновления соединения
        Thread(target=self._readLoop, daemon=True).start()  # Запуск потока для чтения данных с GPS
        Thread(target=self._publishLoop, daemon=True).start()  # Запуск потока для публикации данных

    def _publishLoop(self):
        """Цикл для публикации данных."""
        while True:
            self.publishData()
            time.sleep(0.1)

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
            time.sleep(0.1)
            if self.serial is None:  # Проверяем, что объект последовательного порта существует и открыт
                continue
            with self.serial_lock:
                try:
                    raw_data = self.serial.readline()
                    self.log_file.write(f"Raw data: {raw_data}\n")  # Запись сырых данных в файл
                    try:
                        raw_data = raw_data.decode('utf-8')  # Чтение строки данных
                    except UnicodeDecodeError as e:
                        self.log_file.write(f"UnicodeDecodeError: {e}\n")
                        continue  # Пропуск строки при ошибке декодирования
                    if not raw_data:  # Проверяет, что данные были успешно прочитаны из последовательного порта
                        continue  # Пропуск пустых строк
                    try:
                        data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                        self.log_file.write(f"Parsed data: {data}\n")  # Запись разобранных данных в файл
                        if data.sentence_type in self.parse_sentence:
                            self.parse_sentence[data.sentence_type](data)
                    except pynmea2.nmea.ParseError as e:
                        self.log_file.write(f"NMEA ParseError: {e}\n")
                        continue  # Пропуск строки при ошибке парсинга
                    self.serial.reset_input_buffer()  # Очистка буфера ввода
                except Exception as e:
                    self.log_file.write(f"Exception: {e}\n")

    def publishData(self):
        """Публикация данных."""
        print({attr: getattr(self, attr) for attr in ['latitude', 'longitude', 'satellites', 'ground_speed']})
        with self.lock:
            if self.satellites is not None:
                self.publishOdometry()
                self.publishNavSatFix()
                self.publishSallelites()

    def update_data(self, new_data: dict):
        """Обновление данных."""
        with self.lock:
            self.data.update(new_data)

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
