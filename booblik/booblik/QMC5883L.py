import rclpy
from rclpy.node import Node
import raspy_qmc5883l  # Библиотека для работы с магнитометром QMC5883L
from sensor_msgs.msg import Imu  # Стандартный тип сообщения ROS для данных IMU
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math
from booblik.logging_config import setup_logging
from booblik.utils import euler_to_quaternion, load_config, get_directory
import os
import logging


class QMC5883LNode(Node):
    def __init__(self, config, name='QMC5883L'):
        super().__init__(name)

        # Настройка логгера с использованием имени ноды
        setup_logging(log_filename='Compass', date=True)
        self.logger = logging.getLogger('Compass')

        self.config = config 

        # Установка местоположения
        location = self.config.get('location', 'Saint-Petersburg')
        self.logger.info(f"Using location: {location}")

        # Получение значения склонения для указанного местоположения
        declinations = self.config['declinations']
        self.declination = declinations.get(location, declinations.get('Saint-Petersburg'))
        self.logger.debug(f"Magnetic declination for {location}: {self.declination}")
        

        # Инициализация сенсора с использованием местоположения
        self.sensor = self.initialize_sensor() 
        
        self.imu_publisher = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/quaternions',
            10)
        self.euler_publisher = self.create_publisher(
            Vector3,
            '/booblik/sensors/imu/imu/euler',
            10)
        
        self.declination_subscription = self.create_subscription(
            Float64,
            '/booblik/sensors/gps/navsat/declination',
            self.declination_callback,
            10
        )

        # Запуск непрерывного чтения данных
        self.read_data_continuously()

    def initialize_sensor(self):
        """Инициализация магнитометра с помощью калибовочной матрицы и склонения"""
        while rclpy.ok():
            try:
                sensor = raspy_qmc5883l.QMC5883L()
                sensor.calibration = self.config['calibration_matrix']
                self.logger.info(f'Magnetometer initialized successfully.') 
                
                # Выбор магнитного склонения для инициализации в зависимости от местоположения
                sensor.declination = self.declination + self.config['construction_angle_fix']
                self.logger.info(f'Declination set: {self.declination} with construction redused {self.config["construction_angle_fix"]}')
                return sensor
            except Exception as e:
                self.logger.error(f'Init Error: {type(e).__name__} - {e}\nRetrying initialization...') 

    def declination_callback(self, msg):
        """Обновление магнитного склонения при получении данных от GPS"""
        self.declination = msg.data
        self.sensor.declination = self.declination + self.config['construction_angle_fix']
        self.logger.info(f'Updated declination using GPS data: {self.declination}')

    def read_data_continuously(self):
        """Непрерывное чтение данных с магнитометра."""
        while rclpy.ok():
            try:
                # Получение азимутального угла от магнитометра 
                bearing = self.sensor.get_bearing()  # Здесь библиотека непрерывно возвращает новые данные
                self.logger.info(f'Bearing: {bearing}, when declination is {self.declination}')
                print(f'Bearing with declination: {bearing:.2f}, when declination is {self.declination:.2f}')
 
                # Преобразование азимута в кватернион
                qw, qx, qy, qz = euler_to_quaternion(math.radians(bearing), 0, 0)
                self.publish_quaternion((qw, qx, qy, qz))
                self.publish_euler(math.radians(bearing), 0.0, 0.0)
                
            except Exception as e:
                self.logger.error(f'Except: Request error: {e}')

    def stop(self):
        self.running = False  # Остановка потока
        self.thread.join()  # Ждем завершения потока

    def publish_quaternion(self, quats):
        """Публикация кватернионов."""
        qw, qx, qy, qz = quats
        imu = Imu()
        # Заполнение данных ориентации
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_publisher.publish(imu)
    
    def publish_euler(self, yaw, pitch, roll):
        """Публикация углов Эйлера."""
        euler = Vector3()
        euler.x = pitch
        euler.y = yaw
        euler.z = roll
        self.euler_publisher.publish(euler)

def main(args=None):
    rclpy.init(args=args)
    booblik_dir = get_directory(target='booblik')
    config_file = os.path.join(booblik_dir, 'config.json')
    config = load_config(config_file)  # Загрузка конфигурации

    task = QMC5883LNode(config['compass'])  # Передаем конфигурацию в конструктор ноды
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        pass
    finally:
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()