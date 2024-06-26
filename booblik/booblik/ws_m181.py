from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Тип сообщения ROS для данных о местоположении
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
import pynmea2  # Библиотека для разбора данных в формате NMEA, получаемых от GPS

from .QMC5883L import QMC5883LNode

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
        Thread(target=self._readLoop, daemon=True).start()  # ЗЗапуск отдельного потока для чтения данных с GPS


    def _readLoop(self):
        """Цикл для чтения данных с GPS и их публикации."""
        # Инициализация подключения к GPS через последовательный порт
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=3
        )  # open serial port
        while True:
            try:
                raw_data = ser.readline().decode()  # Чтение строки данных
                print(raw_data)
                data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                if data.sentence_type in parse_sentence:
                    results = parse_sentence[data.sentence_type](data)
                    self.data.update(results)
            except NotImplementedError:
                pass
                # print('Парсинг `{0}` не реализован'.format(data.sentence_type))
            except Exception as e:
                pass
                # print('Exception: ', e)
            self.publishData()
    
    def publishData(self):
        #print(self.data)
        self.publishOdometry()
        self.publishNavSatFix()
        self.publishSallelites()

    def publishOdometry(self):
        msg = Odometry()
        
        # msg.twist.twist.angular.z = self.angular_velocity
        msg.twist.twist.linear.x = self.data['ground_speed']
        
        # msg.pose.pose.orientation.x = self.qx
        # msg.pose.pose.orientation.y = self.qy
        # msg.pose.pose.orientation.z = self.qz
        # msg.pose.pose.orientation.w = self.qw
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


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(GpsImuNode())
    #executor.add_node(QMC5883LNode())
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
