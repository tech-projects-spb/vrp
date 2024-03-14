# import troykahat
import struct
from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu

# Форматы структур данных для разбора пакетов данных
format_time = 'BBBB'
format_latlon = "=BBiic"
format_angle = "=BBhhhhc"
format_quat = '=BBhhhhc'


def parseLanLon(packet: bytearray):
    """Разбор пакета данных с широтой и долготой."""
    # Распаковка данных согласно заданному формату
    s = struct.unpack(format_latlon, packet)
    # Преобразование координат в десятичный формат
    lon = s[2]//1e7 + (s[2] % 1e7)/1e5/60 
    lat = s[3]//1e7 + (s[3] % 1e7)/1e5/60 
    print(lat, lon)
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
    q1 = s[2] / 32768
    q2 = s[3] / 32768
    q3 = s[4] / 32768
    q0 = s[5] / 32768
    return q0, q1, q2, q3


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
        # Создание издателей для GPS и IMU
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        self.nav_
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/data',
            10)
        self.imu_

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
        if packet[1] == 0x59:  # Если пакет содержит данные кватернионов
            q0, q1, q2, q3 = parseQuat(packet)  # Разбор данных кватернионов
            # Публикация данных ориентации
            imu = Imu()  # Создание сообщения IMU
            # Заполнение данных ориентации
            imu.orientation.x = q0
            imu.orientation.y = q1
            imu.orientation.z = q2
            imu.orientation.w = q3
            self.imu_.publish(imu)  # Публикация сообщения
        elif packet[1] == 0x57:  # Если пакет содержит данные о широте и долготе
            lat, lon = parseLanLon(packet)  # Разбор данных о широте и долготе
            # Публикация данных GPS
            nav = NavSatFix()  # Создание сообщения NavSatFix
            # Заполнение координат
            nav.latitude = lat
            nav.longitude = lon
            self.nav_.publish(nav)  # Публикация сообщения


def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)  # Запуск узла
    rclpy.shutdown()  # Завершение работы узла


if __name__ == '__main__':
    main()
