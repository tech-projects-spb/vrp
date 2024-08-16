from threading import Thread
import rclpy
from rclpy.node import Node
import time
import raspy_qmc5883l  # Библиотека для работы с магнитометром QMC5883L
from sensor_msgs.msg import Imu  # Стандартный тип сообщения ROS для данных IMU
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math 
from . import logging_config

import logging 
DEBUG = True

def euler_to_quaternion(yaw, pitch, roll):
    """Преобразование углов Эйлера в кватернион для описания ориентации в пространстве"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qw, qx, qy, qz

# Калибровочные данные для магнитометра
CALIBRATION_MATRIX = [[1.0817261189833043, -0.06705906178799911, -485.7272567957916],
                      [0.06705906178799906, -1.0550242422352802, 2953.8769005789645],
                      [0.0, 0.0, 1.0]]

DECLINATIONS = { # Данные по магнитному склонению на 16.08.2024
    'Obninsk' : 11.57,
    'Saint-Petersburg' : 12.0873,
    'Vladivostok' : -10.9382
}

class QMC5883LNode(Node):
    def __init__(self, name='QMC5883L', location='Saint-Petersburg'):
        super().__init__(name) 
        logging_config.setup_logging(log_filename='Compas')  # Настройка логгера с использованием имени ноды
        self.logger = logging.getLogger('Compas') # Создание логгера для данных 

        # self.declare_parameter('location', 'default_location')
        # location = self.get_parameter('location').get_parameter_value().string_value

        self.sensor = self.initialize_sensor(location)
        
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/quaternions',
            10)
        self.euler_ = self.create_publisher(
            Vector3,
            '/booblik/sensors/imu/imu/euler',
            10)
        
        self.declination_ = self.create_subscription(
            Float64,
            '/booblik/sensors/gps/navsat/declination',
            self.declination_callback,
            10
        )

        self.update_rate = 0.1  # Частота обновления данных в секундах
        # Запуск потока для чтения данных с магнитометра
        Thread(target=self._readLoop, daemon=True).start()

    def initialize_sensor(self, location):
        """Инициализация магнитометра с помощью калибовочной матрицы"""
        while True:
            try:
                sensor = raspy_qmc5883l.QMC5883L()
                sensor.calibration = CALIBRATION_MATRIX
                self.logger.info('Magnetometer initialized successfully.') 
                # Выбор магнитного склонения для инициализации в зависимости от местоположения
                sensor.declination = DECLINATIONS.get(location,'Saint-Petersburg')
                self.logger.info('Declination set for {location}.')
                return sensor
            except Exception as e:
                self.logger.error(f'Init Error: {e}\nRetrying initialization...')
                time.sleep(0.5) 

    def declination_callback(self, msg):
        self.sensor.declination = msg.data  

    def _readLoop(self):
        """Поток для непрерывного чтения и публикации данных с магнитометра"""
        while True:
            time.sleep(self.update_rate)  # Ограничение частоты чтения
            try:
                # Получение азимутального угла от магнитометра 
                bearing = self.sensor.get_bearing()                                
                self.logger.info(f'Bearing: {bearing},  when declination is {self.sensor.declination}\n')
                print(f'Bearing with declination: {bearing:.2f}, when declination is {self.sensor.declination:.2f}')
 
                # Преобразование азимута в кватернион
                qw, qx, qy, qz = euler_to_quaternion(math.radians(bearing), 0, 0)
                self.publish_quaternion((qw, qx, qy, qz))
                self.publish_euler(math.radians(bearing), 0.0, 0.0)
                
            except Exception as e:
                self.logger.error(f'Except: Reques error: {e}\n')

    def publish_quaternion(self, quats):
        qw, qx, qy, qz = quats
        # Формирование и публикация сообщения IMU
        imu = Imu() 
        # Заполнение данных ориентации
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_.publish(imu)
    
    def publish_euler(self, yaw, pitch, roll):
        euler = Vector3()
        # заполнение данных углов
        euler.x = pitch
        euler.y = yaw
        euler.z = roll
        self.euler_.publish(euler)

def main(args=None):
    rclpy.init(args=args)
    task = QMC5883LNode(location='Vladivostok')
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
