from enum import Enum
import math
import struct
from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt32, Float32

import logging
from . import logging_config
DEBUG = True

class PacketID(Enum):
    RTC = 0x50 #< Real-Time-Clock: Year from 2000, Month, Day, Hour, Minute, Second (8-bit unsigned integers) + Millisecond (16-bit unsigned integer), representing time passed since last time set up in the \ref ridTimeYearMonth, \ref ridTimeDayHour, \ref ridTimeMinuteSecond and \ref ridTimeMilliseconds registers
    Acceleration = 0x51 #< Linear accelerations + temperature/reserved field [X-Y-Z] (16-bit binary normalized quasi-floats)
    AngularVelocity = 0x52 #< Angular velocities + temperature/reserved field [Roll-Pitch-Yaw] (16-bit binary normalized quasi-floats)
    Angles = 0x53 #< Euler angles + temperature/reserved field [Roll-Pitch-Yaw] (16-bit binary normalized quasi-floats)
    Magnetometer = 0x54 # Magnetic field tensity + temperature/reserved field [world X-Y-Z] (16-bit binary normalized quasi-floats)
    DataPortStatus = 0x55 # Data port status packet, vendor-defined value
    Altimeter = 0x56 # Altimeter + Barometer output (32-bit binary normalized quasi-floats)
    GPSCoordinates = 0x57 # GPS: longitude + latitude, if supported by hardware (32-bit binary normalized quasi-floats)
    GPSGroundSpeed = 0x58 # GPS: ground speed (32-bit binary normalized quasi-float) + altitude + angular velocity around vertical axis (16-bit binary normalized quasi-floats), if supported by hardware
    Orientation = 0x59 # Orientation defined as quaternion [X-Y-Z-W], when available from the sensor firmware (16-bit binary normalized quasi-floats)
    GPSAccuracy = 0x5A # GPS: visible satellites + variance vector [East-North-Up] (16-bit binary normalized quasi-floats)


# Форматы структур данных для разбора пакетов данных
format_time = 'BBBB'
format_latlon = "=BBiic"
format_default = "=BBhhhhc"
format_angle = format_default
format_quat = format_default
format_angle_velocity = format_default
format_acceleration = format_default
format_gps_accuracy = format_default
format_gps_ground_speed = "=BBhhhic"


def parseLanLon(packet: bytearray):
    """Разбор пакета данных с широтой и долготой."""
    # Распаковка данных согласно заданному формату
    s = struct.unpack(format_latlon, packet)
    # Преобразование координат в десятичный формат
    lon = s[2]//1e7 + (s[2] % 1e7)/1e5/60 
    lat = s[3]//1e7 + (s[3] % 1e7)/1e5/60
    return lat, lon


def parseAngle(packet: bytearray):
    """Разбор пакета данных с углами ориентации."""
    s = struct.unpack(format_angle, packet)
    # Преобразование значений в градусы
    roll = s[2] / 32768 * 180
    pitch = s[3] / 32768 * 180
    yaw = s[4] / 32768 * 180
    return roll, pitch, yaw


def parseQuat(packet: bytearray):
    """Разбор пакета данных с кватернионами ориентации."""
    s = struct.unpack(format_angle, packet)
    # Преобразование значений кватернионов
    qx = s[3] / 32768
    qy = s[4] / 32768
    qz = s[5] / 32768
    qw = s[2] / 32768
    return qx, qy, qz, qw

def parseAngleVelocities(packet: bytearray):
    """Разбор пакета данных с угловыми скоростями."""
    s = struct.unpack(format_angle_velocity, packet)
    # Преобразование значений кватернионов
    wx = s[2] / 32768 * 2000
    wy = s[3] / 32768 * 2000
    wz = s[4] / 32768 * 2000
    t = s[5] / 100
    return wx, wy, wz, t

def parseAcceleration(packet: bytearray):
    """Разбор пакета данных с ускорениями."""
    s = struct.unpack(format_acceleration, packet)
    # Преобразование значений кватернионов
    ax = s[2] / 32768 * 16 * 9.81
    ay = s[3] / 32768 * 16 * 9.81
    az = s[4] / 32768 * 16 * 9.81
    t = s[5] / 100
    return ax, ay, az, t

def parseGpsAccuracy(packet: bytearray):
    """Разбор пакета данных с ускорениями."""
    s = struct.unpack(format_acceleration, packet)
    # Преобразование значений кватернионов
    satellites = s[2] 
    local_accuracy = s[3] / 32768 
    horizontal_accuracy = s[4] / 32768
    vertical_accuracy = s[5] / 32768
    return satellites, local_accuracy, horizontal_accuracy, vertical_accuracy

def parseGpsGroundSpeed(packet: bytearray):
    """Разбор пакета данных с ускорениями."""
    s = struct.unpack(format_acceleration, packet)
    # Преобразование значений кватернионов
    altitude = s[2] / 10
    angular_velocity = s[4] / 10
    ground_speed = s[5] / 1000
    return altitude, angular_velocity, ground_speed


@dataclass
class GpsConfig:
    """Конфигурация для GPS-модуля."""
    port: str  # Порт подключения
    baudrate: int  # Скорость передачи данных


class GpsImuNode(Node):
    """Узел ROS2 для приема и публикации данных с GPS и IMU."""
    config: GpsConfig

    def __init__(self, name='gpsimu'):
        super().__init__(name)
        self.config = GpsConfig('/dev/serial0', 9600)  # Конфигурация порта

        logging_config.setup_logging(log_filename='Compas')  # Настройка логгера с использованием имени ноды
        self.logger = logging.getLogger('Compas') # Создание логгера для данных 

        # Создание издателей для GPS и IMU
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        self.nav_
        self.accuracy_ = self.create_publisher(
            Vector3,
            '/booblik/sensors/gps/accuracy',
            10)
        self.accuracy_
        self.satellites_ = self.create_publisher(
            UInt32,
            '/booblik/sensors/gps/satellites',
            10)
        self.satellites_
        
        self.odometry_ = self.create_publisher(
            Odometry,
            '/booblik/sensors/position/ground_truth_odometry',
            10)
        self.odometry_
        
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/data',
            10)
        self.imu_
        
        self.lat, self.lon = 0,0
        
        self.wx, self.wy, self.wz = 0.,0.,0.
        self.ax, self.ay, self.az = 0.,0.,0.
        self.qx, self.qy, self.qz, self.qw = 0.,0.,0.,0.
        self.satellites, self.local_acc, self.horizontal__acc, self.vertical_acc = 0.,0.,0.,0.
        self.altitude, self.angular_velocity, self.ground_speed = 0.,0.,0.

        Thread(target=self._readLoop, daemon=True).start()  # Запуск чтения данных в отдельном потоке

    def _readLoop(self):
        """Чтение и обработка данных из последовательного порта."""
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate
        )  # open serial port
        buffer = bytearray()  # Буфер для хранения прочитанных данных

        while True:
            buffer.extend(ser.read(22))  # Чтение данных из порта
            index = buffer.find(0x55, 1)  # Поиск начала следующего пакета
            while index != -1 and index:
                packet = buffer[0:index]  # Извлечение пакета данных
                if (len(packet) == 11):
                    self.parsePacket(packet)  # Обработка пакета
                buffer = buffer[index:]  # Удаление обработанного пакета из буфера
                index = buffer.find(0x55, 1)  # Поиск следующего пакета

    def parsePacket(self, packet: bytearray):
        """Разбор пакета данных и публикация сообщений."""
        print
        if packet[1] == PacketID.Orientation.value:  # Если пакет содержит данные кватернионов
            self.qx, self.qy, self.qz, self.qw = parseQuat(packet)  # Разбор данных кватернионов
            self.logger.debug(f'Parsing Quaternions: {self.qx=}\t{self.qy=}\t{self.qz=}\t{self.qw=}')           
            self.imu_process()
            self.odometry_process()
        elif packet[1] == PacketID.AngularVelocity.value:  
            self.wx, self.wy, self.wz, _ = parseAngleVelocities(packet) 
            self.logger.debug(f'Parsing AngularVelocities: {self.wx=}\t{self.wy=}\t{self.wz=}')
            self.imu_process()
        elif packet[1] == PacketID.Acceleration.value:  
            self.ax, self.ay, self.az, _ = parseAcceleration(packet) 
            self.logger.debug(f'Parsing Acceleration: {self.ax=}\t{self.ay=}\t{self.az=}')
            self.imu_process()
        elif packet[1] == PacketID.GPSCoordinates.value:  # Если пакет содержит данные о широте и долготе
            self.lat, self.lon = parseLanLon(packet)  # Разбор данных о широте и долготе
            self.logger.debug(f'Parsing GPSCoordinates: {self.lat=}\t{self.lon=}')
            self.navsatfix_process()
        elif packet[1] == PacketID.GPSAccuracy.value: 
            self.satellites, self.local_acc, self.horizontal__acc, self.vertical_acc = parseGpsAccuracy(packet)
            self.logger.debug(f'Parsing GPSAccuracy: {self.satellites=}\t{self.local_acc=}\t{self.horizontal__acc=}\t{self.vertical_acc=}')
            self.accuracy_process()
            self.satellites_process()
        elif packet[1] == PacketID.GPSGroundSpeed.value:
            self.altitude, self.angular_velocity, self.ground_speed = parseGpsGroundSpeed(packet) 
            self.logger.debug(f'Parsing Acceleration: {self.altitude=}\t{self.angular_velocity=}\t{self.ground_speed=}\n')
            self.odometry_process()
            

    def navsatfix_process(self):
        # Публикация данных GPS
        nav = NavSatFix()  # Создание сообщения NavSatFix
        # Заполнение координат
        nav.latitude = self.lat
        nav.longitude = self.lon
        self.nav_.publish(nav)  # Публикация сообщения
    
    def odometry_process(self):
        msg = Odometry()
        
        msg.twist.twist.angular.z = self.angular_velocity
        msg.twist.twist.linear.x = self.ground_speed
        
        msg.pose.pose.orientation.x = self.qx
        msg.pose.pose.orientation.y = self.qy
        msg.pose.pose.orientation.z = self.qz
        msg.pose.pose.orientation.w = self.qw
        self.odometry_.publish(msg)
        
    def satellites_process(self):
        msg = UInt32()
        msg.data = self.satellites
        self.satellites_.publish(msg)  # Публикация сообщения
    
    def accuracy_process(self):
        msg = Vector3()
        msg.x = self.local_acc
        msg.y = self.horizontal__acc
        msg.z = self.vertical_acc
        self.accuracy_.publish(msg)  # Публикация сообщения
    
    # Публикация данных ориентации
    def imu_process(self):
        imu = Imu()  # Создание сообщения IMU
        # Заполнение данных ориентации
        imu.angular_velocity.x = math.radians(self.wx)
        imu.angular_velocity.y = math.radians(self.wy)
        imu.angular_velocity.z = math.radians(self.wz)
        
        imu.linear_acceleration.x = self.ax
        imu.linear_acceleration.y = self.ay
        imu.linear_acceleration.z = self.az
        
        imu.orientation.x = self.qx
        imu.orientation.y = self.qy
        imu.orientation.z = self.qz
        imu.orientation.w = self.qw
        self.imu_.publish(imu)  # Публикация сообщения
    
    
    
def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)  # Запуск узла
    rclpy.shutdown()  # Завершение работы узла


if __name__ == '__main__':
    main()
