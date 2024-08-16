from dataclasses import dataclass
import serial
from threading import Thread, Lock
import rclpy
import time 
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Тип сообщения ROS для данных о местоположении
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32, Float64
import pynmea2  # Библиотека для разбора данных в формате NMEA, получаемых от GPS 
from geomag.geomag import GeoMag

import logging 
DEBUG = True 

# Настройка логгера
logging.basicConfig(
            level=logging.DEBUG if DEBUG else logging.info,
            format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
            filename='gps_log.log',
            filemode='a'
        )

def kmph_2_mps(kmph):
    """Конвертация км/ч в м/с"""
    return kmph / 3.6

def knot_2_mps(knots):
    """Конвертация узлы в м/с"""
    return knots * 0.514444

def declin_dir(mag_var, direction):
    """Обработка направления магнитного склонения"""
    return mag_var if direction == 'E' else -mag_var

@dataclass
class GpsConfig:
    """Конфигурация подключения к GPS модулю"""
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
        self.declination = None
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
        self.declination_ = self.create_publisher(Float64, '/booblik/sensors/gps/navsat/declination', 10)
        
        self.serial_lock = Lock()
        self.serial = None
        self.lock = Lock()
        
        self.logger = logging.getLogger('GPS') # Создание логгера для GPS данных
        
        Thread(target=self._updateLoop, daemon=True).start()  # Запуск потока для обновления соединения
        Thread(target=self._readLoop, daemon=True).start()  # Запуск потока для чтения данных с GPS
        Thread(target=self._publishLoop, daemon=True).start()  # Запуск потока для публикации данных

    def _publishLoop(self):
        """Цикл для публикации данных"""
        while True:
            self.publishData()
            time.sleep(0.05)

    def _updateLoop(self):
        """Цикл для обновления соединения с GPS модулем"""
        while True:
            with self.serial_lock:
                if self.serial:
                    self.serial.close()  # Закрываем предыдущий порт для предотвращения утечек ресурсов и конфликтов
                self.serial = serial.Serial(self.config.port, self.config.baudrate, timeout=3)
            time.sleep(2)
            print('updated connection')

    def _readLoop(self):
        """Цикл для чтения данных с GPS и их обработки"""
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
        """Публикация данных"""
        print({attr: getattr(self, attr) for attr in ['latitude', 'longitude', 'satellites', 'ground_speed', 'declination'] if getattr(self, attr) is not None})
        with self.lock: # Захват блокировки для синхронизации доступа к данным перед публикацией
            if self.satellites is not None:
                self.publishOdometry()
                self.publishNavSatFix()
                self.publishSatellites()
                self.publishDeclination()

    def publishOdometry(self):
        """Публикация данных одометрии"""
        msg = Odometry()
        if self.ground_speed is not None:
            msg.twist.twist.linear.x = self.ground_speed
            self.odometry_.publish(msg)

    def publishNavSatFix(self):
        """Публикация данных GPS"""
        nav = NavSatFix()
        if self.latitude is not None and self.longitude is not None:
            nav.latitude = self.latitude
            nav.longitude = self.longitude
            self.nav_.publish(nav)

    def publishSatellites(self):
        """Публикация количества спутников"""
        msg = UInt32()
        if self.satellites is not None:
            msg.data = self.satellites
            self.satellites_.publish(msg)
    
    def publishDeclination(self):
        """Публикация данных магнитного склонения"""
        msg = Float64()
        if self.declination is not None:
            msg.data = self.declination
        elif self.latitude is not None and self.longitude is not None: # Если нет данных, то посчитаем, используя координаты 
            geomag_instance = GeoMag()
            msg.data = geomag_instance.GeoMag(self.latitude, self.longitude, 0).dec # Возвращает только значения склонения
        self.declination_.publish(msg)

    def parseGGA(self, data):
        """Обработка данных GGA"""
        with self.lock: # Захват блокировки для безопасного доступа к разделяемым данным
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.satellites = int(data.num_sats)

    def parseVTG(self, data):
        """Обработка данных VTG"""
        with self.lock: # Захват блокировки для безопасного доступа к разделяемым данным
            self.ground_speed = kmph_2_mps(data.spd_over_grnd_kmph)

    def parseRMC(self, data):
        """Обработка данных RMC"""
        with self.lock: # Захват блокировки для безопасного доступа к разделяемым данным
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.ground_speed = knot_2_mps(data.spd_over_grnd_kmph)   #Скорость в узлах
            self.declination = declin_dir(data.mag_variation, data.mag_var_dir)
                

def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)
    rclpy.shutdown()

if __name__ == '__main__':
    main()