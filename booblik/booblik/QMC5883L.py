from threading import Thread
import rclpy
from rclpy.node import Node
import time
import raspy_qmc5883l  # Библиотека для работы с магнитометром QMC5883L
from sensor_msgs.msg import Imu  # Стандартный тип сообщения ROS для данных IMU
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math 

import logging 
DEBUG = True

def degrees_to_radians(degrees):
    """Конвертация углов из градусов в радианы"""
    return degrees * math.pi / 180

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
                      [-0.06705906178799906, 1.0550242422352802, -2953.8769005789645],
                      [0.0, 0.0, 1.0]]

ZERO_THRESHOLD = 1e-2 # Определяем порог "близости к нулю" для избежания ошибок

# Настройка логгера
logging.basicConfig(
            level=logging.DEBUG if DEBUG else logging.info,
            format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
            filename='compas_log.log',
            filemode='a'
        )

class QMC5883LNode(Node):
    def __init__(self, name='QMC5883L'):
        super().__init__(name)
        self.last_bearing = None
        self.declination = 12.0873 # Для Петербурга ~ 12.0873° E 
        self.logger = logging.getLogger('Compas') # Создание логгера для данных 

        while True:
            try:
                # Инициализация магнитометра
                self.sensor = raspy_qmc5883l.QMC5883L()
                # Загрузка калибровочных данных для магнитометра
                self.sensor.calibration = CALIBRATION_MATRIX
                self.logger.info('Magnetometer initialized successfully.')
                break
            except Exception as e:
                self.logger.error('Init Error: {e}\nRetrying initialization...')
                print("Init Error. Retrying initialization...")
                time.sleep(0.1)

        # Создание издателя для публикации данных IMU        
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/data',
            10)
        self.odometry_ = self.create_publisher(
            Odometry,
            '/booblik/sensors/position/ground_truth_odometry',
            10)
        
        self.declination_ = self.create_subscription(
            Float64,
            '/booblik/sensors/gps/navsat/declination',
            self.declination_callback,
            10
        )

        # Запуск потока для чтения данных с магнитометра
        Thread(target=self._readLoop, daemon=True).start()

    def declination_callback(self, msg):
        self.declination = msg.data

                    
    def _readLoop(self):
        """Поток для непрерывного чтения и публикации данных с магнитометра"""
        while True:
            time.sleep(0.1)  # Ограничение частоты чтения
            try:
                # Получение азимутального угла от магнитометра 
                bearing = self.sensor.get_bearing()
                if bearing == 0:
                    if self.last_bearing is not None and abs(self.last_bearing) > ZERO_THRESHOLD:
                        self.logger.warning('Unexpected 0 bearing detected, possibly a sensor error.')
                        continue # Игнорируем данное значение, скорее всего ошибка
                    else:
                        self.logger.info('Bearing is 0, which seems consistent with previous readings')
                

                # Учет магнитного склонения
                corrected_bearing = bearing + self.declination
                if corrected_bearing > 360: 
                    corrected_bearing -= 360
                elif corrected_bearing < 0:
                    corrected_bearing += 360
                                
                self.logger.info(f'Bearing: {bearing}, Declination: {self.declination}, Corrected bearing: {corrected_bearing}\n')
                print(f'Bearing with declination: {corrected_bearing:.2f}, when declination is {self.declination:.2f}')
 
                # # NOTE это нужно, чтобы в pypilot отображалось правильно
                # bearing = math.degrees(math.pi/ 2) - bearing

                # Преобразование азимута в кватернион
                qw, qx, qy, qz = euler_to_quaternion(math.radians(corrected_bearing), 0, 0)
                self.publishQuats((qw, qx, qy, qz))
                self.last_bearing = bearing
                
            except Exception as e:
                self.logger.error(f'Except: Reques error: {e}\n')
                print("Except: Reques error: ", e)
    
    def publishQuats(self, quats):
        qw, qx, qy, qz = quats

        # Формирование и публикация сообщения Odometry
        odometry = Odometry()
        # Заполнение данных ориентации
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        self.odometry_.publish(odometry)
    
        # Формирование и публикация сообщения IMU
        imu = Imu() 
        # Заполнение данных ориентации
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    task = QMC5883LNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
