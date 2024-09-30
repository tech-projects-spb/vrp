from threading import Thread
import rclpy
from rclpy.node import Node
import time
import raspy_qmc5883l  # Библиотека для работы с магнитометром QMC5883L
from sensor_msgs.msg import Imu  # Стандартный тип сообщения ROS для данных IMU
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math 
import logging_config
from .utils import euler_to_quaternion, get_directory, load_config
import os
import json

import logging 


class QMC5883LNode(Node):
    def __init__(self, config_file, name='QMC5883L', default_location='Saint-Petersburg' ):
        super().__init__(name) 

        # Настройка логгера с использованием имени ноды
        logging_config.setup_logging(log_filename='Compas', date=True)
        self.logger = logging.getLogger('Compas') 

        self.config = self.load_config(config_file) 
        self.construction_angle_fix = self.config['compas']['construction_angle_fix']

        # Установка местоположения
        self.location = self.config.get('location', default_location)
        self.logger.info(f"Using location: {self.location}")

        # Получение значения склонения для указанного местоположения
        declinations = self.config['compas']['declinations']
        self.declination = declinations.get(self.location, declinations[default_location])
        self.logger.debug(f"Magnetic declination for {self.location}: {self.declination}")
        

        # Инициализация сенсора с использованием местоположения
        self.sensor = self.initialize_sensor() 
        
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
        Thread(target=self._readLoop, daemon=True).start()  # Запуск потока для чтения данных с магнитометра

    def initialize_sensor(self):
        """Инициализация магнитометра с помощью калибовочной матрицы"""
        while True:
            try:
                sensor = raspy_qmc5883l.QMC5883L()
                sensor.calibration = self.config['compas']['calibration_matrix']
                self.logger.info(f'Magnetometer initialized successfully.') 
                # Выбор магнитного склонения для инициализации в зависимости от местоположения
                sensor.declination = self.declination + self.construction_angle_fix
                self.logger.info(f'Declination set for {self.location}.')
                return sensor
            except Exception as e:
                self.logger.error(f'Init Error: {e}\nRetrying initialization...')
                time.sleep(0.5) 

    def declination_callback(self, msg):
        self.declination = msg.data
        self.sensor.declination = msg.data + self.construction_angle_fix

    def _readLoop(self):
        """Поток для непрерывного чтения и публикации данных с магнитометра"""
        while True:
            time.sleep(self.update_rate)  # Ограничение частоты чтения
            try:
                # Получение азимутального угла от магнитометра 
                bearing = self.sensor.get_bearing()                                
                self.logger.info(f'Bearing: {bearing}, when declination is {self.declination}\n')
                print(f'Bearing with declination: {bearing:.2f}, when declination is {self.declination:.2f}')
 
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
    # Получаем путь к директории booblik
    booblik_dir = get_directory(target='booblik') 
    config_file = os.path.join(booblik_dir, 'config.json')
    task = QMC5883LNode(config_file=config_file)
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
