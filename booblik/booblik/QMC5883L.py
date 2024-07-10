from threading import Thread
import rclpy
from rclpy.node import Node
import time
import raspy_qmc5883l  # Библиотека для работы с магнитометром QMC5883L
from sensor_msgs.msg import Imu  # Стандартный тип сообщения ROS для данных IMU
from nav_msgs.msg import Odometry
import math

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

class QMC5883LNode(Node):
    def __init__(self, name='QMC5883L'):
        super().__init__(name)
        self.last_bearing = None
        # Запуск потока для чтения данных с магнитометра
        Thread(target=self._readLoop, daemon=True).start()
        while 1:
            try:
                # Инициализация магнитометра
                self.sensor = raspy_qmc5883l.QMC5883L()
                # Загрузка калибровочных данных для магнитометра
                #TODO read from config? 
                self.sensor.calibration = [[1.0817261189833043, -0.06705906178799911, -485.7272567957916], 
                      [-0.06705906178799906, 1.0550242422352802, -2953.8769005789645], 
                      [0.0, 0.0, 1.0]]
                break
            except:
                print("Init Error. Try init again...")
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
            

    def _readLoop(self):
        """Поток для непрерывного чтения и публикации данных с магнитометра."""
        while True:
            time.sleep(0.1)  # Ограничение частоты чтения
            try:
                # Получение азимутального угла от магнитометра 
                bearing = self.sensor.get_bearing()                
                print('Bearing', bearing)

                # NOTE пытаемся исключить момент, когда у нас не приходит беринг
                # Наблюдается получение bearing=0
                if self.last_bearing != None:
                    value = ((bearing - self.last_bearing) / (self.last_bearing + 1e-5))
                    if abs(value - 1) < 1e-4:
                        continue 

                # NOTE это нужно, чтобы в pypilot отображалось правильно
                bearing = math.degrees(math.pi/ 2) - bearing

                # Преобразование азимута в кватернион
                qw, qx, qy, qz = euler_to_quaternion(math.radians(bearing), 0, 0)
                self.publishQuats((qw, qx, qy, qz))
                self.last_bearing = bearing
                
            except Exception as e:
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
