from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Тип сообщения ROS для данных о местоположении
import pynmea2  # Библиотека для разбора данных в формате NMEA, получаемых от GPS


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
        # Создание издателя для публикации данных о местоположении
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
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
                data = pynmea2.parse(raw_data)  # Разбор строки в формате NMEA
                
                # Обработка только GGA сообщений, содержащих информацию о местоположении
                if data.sentence_type == "GGA":
                    nav = NavSatFix()
                    # Заполнение сообщения данными широты и долготы
                    nav.latitude = data.latitude
                    nav.longitude = data.longitude
                    self.nav_.publish(nav)
                    print(data.latitude, " ", data.longitude)
            except Exception as e:
                pass


def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
