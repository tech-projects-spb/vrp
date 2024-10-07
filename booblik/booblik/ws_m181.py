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
import os
from booblik.utils import kmph_2_mps, knot_2_mps, declin_dir, load_config, get_directory
from booblik.logging_config import setup_logging
import logging 

@dataclass
class GpsConfig:
    """Конфигурация подключения к GPS модулю"""
    port: str  # Порт, к которому подключен GPS
    baudrate: int  # Скорость передачи данных

class GpsImuNode(Node):
    def __init__(self, config, name='ws_m181'):
        super().__init__(name)
        # Инициализация конфигурации GPS на основе данных из конфигурационного файла
        self.config = GpsConfig(config['port'], config['baudrate'])

        setup_logging(log_filename='GPS')  # Настройка логгера с использованием имени ноды
        self.logger = logging.getLogger('GPS') # Создание логгера для данных 

        # Переменные для хранения данных
        self.latitude = None
        self.longitude = None
        self.satellites = None
        self.ground_speed = None
        self.declination = None
        # Доступные `sentence_type`: GLL, GSV, GGA, GSA, VTG
        self.parse_sentence = {
            'GGA': self.parse_gga,
            'VTG': self.parse_vtg,
            'RMC': self.parse_rmc,
        }
        # Создание издателя для публикации данных о местоположении
        self.nav_publisher = self.create_publisher(NavSatFix, '/booblik/sensors/gps/navsat/fix', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/booblik/sensors/position/ground_truth_odometry', 10)
        self.satellites_publisher = self.create_publisher(UInt32, '/booblik/sensors/gps/satellites', 10)
        self.declination_publisher = self.create_publisher(Float64, '/booblik/sensors/gps/navsat/declination', 10)
        
        # Блокировки для работы с данными
        self.gps_serial_lock = Lock()
        self.data_lock = Lock()

        self.serial = None

        self.threads = [
            Thread(target=self._update_connection, daemon=True),  # Запуск потока для обновления соединения
            Thread(target=self._read_loop, daemon=True)  # Запуск потока для чтения данных с GPS
        ]

        for thread in self.threads:
            thread.start()

    def _update_connection(self):
        """Цикл для обновления соединения с GPS модулем"""
        while rclpy.ok():
            with self.gps_serial_lock:
                if self.serial:
                    self.serial.close()  # Закрываем предыдущий порт для предотвращения утечек ресурсов и конфликтов
                self.serial = serial.Serial(self.config.port, self.config.baudrate, timeout=3)
            time.sleep(2)
            print('Сonnection updated')

    def _read_loop(self):
        """Цикл для чтения данных с GPS и их обработки"""
        while rclpy.ok():
            time.sleep(0.05)
            if not self.serial:  # Проверяем, что объект последовательного порта существует и открыт
                continue
            with self.gps_serial_lock:
                try:
                    raw_data = self.serial.readline()
                    self.logger.debug(f'Raw data: {raw_data}') # Запись сырых данных в лог
                    raw_data = raw_data.decode('utf-8')  # Чтение строки данных
                    if not raw_data:  # Проверяет, что данные были успешно прочитаны из последовательного порта
                        continue  # Пропуск пустых строк
                    data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                    self.logger.info(f'Parsed data: {data} ') # Запись разобранных данных в файл
                    if data.sentence_type in self.parse_sentence:
                        self.parse_sentence[data.sentence_type](data)
                    self.publish_data()  # Публикация данных после обработки
                    self.serial.reset_input_buffer()  # Очистка буфера ввода
                except UnicodeDecodeError as e: 
                    self.logger.error(f'UnicodeDecodeError: {e}') 
                except pynmea2.nmea.ParseError as e:
                    self.logger.error(f'NMEA ParseError: {e}')  
                except Exception as e:
                    self.logger.error(f'Exception: {e}') 

    def publish_data(self):
        """Публикация данных"""
        print({attr: getattr(self, attr) for attr in ['latitude', 'longitude', 'satellites', 'ground_speed', 'declination'] if getattr(self, attr) is not None})
        with self.data_lock: # Захват блокировки для синхронизации доступа к данным перед публикацией
            if self.satellites < 3:
                self.logger.warning(f"Too few satellites: {self.satellites}")
                print(f"Warning: Only {self.satellites} satellites available") 
            else:
                self.publish_odometry()
                self.publish_nav_sat_fix() 
                self.publish_declination()
            self.publish_satellites()  # Всегда публикуем количество спутников

    def publish_odometry(self):
        """Публикация данных одометрии"""
        msg = Odometry()
        if self.ground_speed is not None:
            msg.twist.twist.linear.x = self.ground_speed
            self.odometry_publisher.publish(msg)

    def publish_nav_sat_fix(self):
        """Публикация данных GPS"""
        nav = NavSatFix()
        if self.latitude is not None and self.longitude is not None:
            nav.latitude = self.latitude
            nav.longitude = self.longitude
            self.nav_publisher.publish(nav)

    def publish_satellites(self):
        """Публикация количества спутников"""
        msg = UInt32()
        if self.satellites is not None:
            msg.data = self.satellites
            self.satellites_publisher.publish(msg)
    
    def publish_declination(self):
        """Публикация данных магнитного склонения"""
        msg = Float64()
        if self.declination is not None:
            msg.data = self.declination
        elif self.latitude is not None and self.longitude is not None: # Если нет данных, то посчитаем, используя координаты 
            geomag_instance = GeoMag()
            msg.data = geomag_instance.GeoMag(self.latitude, self.longitude, 0).dec # Возвращает только значения склонения
        self.declination_publisher.publish(msg)

    def parse_gga(self, data):
        """Обработка данных GGA"""
        with self.data_lock: # Захват блокировки для безопасного доступа к разделяемым данным
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.satellites = int(data.num_sats)

    def parse_vtg(self, data):
        """Обработка данных VTG"""
        with self.data_lock:
            self.ground_speed = kmph_2_mps(data.spd_over_grnd_kmph)

    def parse_rmc(self, data):
        """Обработка данных RMC"""
        with self.data_lock:
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.ground_speed = knot_2_mps(data.spd_over_grnd_kmph)  #Скорость в узлах
            self.declination = declin_dir(data.mag_variation, data.mag_var_dir)
    
    def stop(self):
        """Остановка всех потоков"""
        for thread in self.threads:
            thread.join()
                

def main(args=None):
    rclpy.init(args=args)
    booblik_dir = get_directory(target='booblik')
    config_file = os.path.join(booblik_dir, 'config.json')
    config = load_config(config_file)  # Загрузка конфигурации

    task = GpsImuNode(config['GPS'])  # Передаем конфигурацию в конструктор ноды
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        pass
    finally:
        task.stop()  # Останавливаем потоки
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()