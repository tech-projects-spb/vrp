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

compas_imported = False
try:
    from .QMC5883L import QMC5883LNode
    compas_imported = True
except ImportError as ie:
    print('Cannot import compas node: ', ie)

def kmph_2_mps(kmph):
    return kmph * 3.6

def parseGGA(data):
    return {
        'latitude': data.latitude, 
        'longitude': data.longitude,
        'satellites': int(data.num_sats)
    }

def parseVTG(data):
    """Строка с идентификатором VTG содержит скорость и курс относительно земли"""
    return {
        "cog": data.mag_track,
        "ground_speed": kmph_2_mps(data.spd_over_grnd_kmph),
        "speed_over_ground_kts": float(data.spd_over_grnd_kts),
    }

# Доступные `sentence_type`: GLL, GSV, GGA, GSA, VTG
parse_sentence = {
    "GGA": parseGGA,  
    'VTG': parseVTG,
}

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
        self.data : dict = {
            'latitude': 0.0, 'longitude': 0.0,
            'satellites': 0,
            'ground_speed': 0.0,
        }
        # Создание издателя для публикации данных о местоположении
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        self.odometry_ = self.create_publisher(
            Odometry,
            '/booblik/sensors/position/ground_truth_odometry',
            10)
        self.satellites_ = self.create_publisher(
            UInt32,
            '/booblik/sensors/gps/satellites',
            10)
        self.serial_lock = Lock()
        self.serial = None
        Thread(target=self._updateLoop, daemon=True).start()
        Thread(target=self._readLoop, daemon=True).start()  # ЗЗапуск отдельного потока для чтения данных с GPS
        

    def _updateLoop(self):
        while True:
            with self.serial_lock:
                del(self.serial)
                self.serial = serial.Serial(
                self.config.port,
                self.config.baudrate,
                timeout=3
                )  # open serial port
            time.sleep(2)

    def _readLoop(self):
        """Цикл для чтения данных с GPS и их публикации."""
        # Инициализация подключения к GPS через последовательный порт
        # ser = serial.Serial(
        #     self.config.port,
        #     self.config.baudrate,
        #     timeout=3
        # )  # open serial port
        while True:
            time.sleep(0.1)
            if self.serial == None: continue
            with self.serial_lock:
                try:
                    raw_data = self.serial.readline().decode()  # Чтение строки данных
                    data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                    if data.sentence_type in parse_sentence:
                        results = parse_sentence[data.sentence_type](data)
                        self.data.update(results)
                except Exception as e:
                    pass
                    print('Exception: ', e)
                self.publishData()
    
    def publishData(self):
        print(self.data)
        self.publishOdometry()
        self.publishNavSatFix()
        self.publishSallelites()

    def publishOdometry(self):
        msg = Odometry()
        
        # msg.twist.twist.angular.z = self.angular_velocity
        msg.twist.twist.linear.x = self.data['ground_speed']
        self.odometry_.publish(msg)
    
    def publishNavSatFix(self):
        # Публикация данных GPS
        nav = NavSatFix()  # Создание сообщения NavSatFix
        # Заполнение координат
        nav.latitude = self.data['latitude']
        nav.longitude = self.data['longitude']
        self.nav_.publish(nav)  # Публикация сообщения
    
    def publishSallelites(self):
        msg = UInt32()
        msg.data = int(self.data['satellites'])
        self.satellites_.publish(msg)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-compas', action='store_true')
    return parser.parse_args()

def main(args=None):
    rclpy.init(args=None)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(GpsImuNode())
    if args.use_compas:
        executor.add_node(QMC5883LNode())
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()



if __name__ == '__main__':
    main(parse_args())
