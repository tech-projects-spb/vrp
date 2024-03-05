import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from threading import Thread
import time
import libscrc  # Библиотека для расчета CRC
import struct  # Модуль для преобразования байтов в значения Python


class ECTDSSensor(Node):
    sensor_idx = 0x4    # Идентификатор датчика EC/TDS
    # Шаблон запроса данных с датчика, включает в себя команду чтения и адрес регистра
    rqst_msg = [sensor_idx, 0x03, 0x0, 0x0, 0x0, 0x2, 0xFF, 0xFF]


    def __init__(self):
        super().__init__('ec_tds_sensor')
        # Создание издателя и подписчика для обмена сообщениями через RS485
        self.tx_ = self.create_publisher(
            UInt8MultiArray,
            '/booblik/rs485Rx',
            10
        )
        self.rx_ = self.create_subscription(
            UInt8MultiArray,
            '/booblik/rs485Tx',
            self.recieve_callback,
            10
        )
        # Запуск потока для периодической отправки запросов к датчику
        self.sendThread = Thread(
            target=self.request_thread, daemon=True).start()


    def check_crc (self, data):
        """Проверка контрольной суммы полученных данных для подтверждения их целостности."""
        size = len(data)
        if (size < 3):
            return False
        # Расчет CRC для проверенной части сообщения                
        crc = libscrc.modbus(bytes(data[0:(size - 2)]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        # Сравнение расчетного CRC с полученным в сообщении
        if (crcLow == data[size - 2]) and (crcHigh == data[size - 1]):
            return True
        else:
            return False


    def check_idx (self, data):
        """Проверка, что сообщение пришло от ожидаемого датчика."""
        if data[0] == self.sensor_idx:
            return True
        else:
            return False


    def parce_msg (self, data):
        """Разбор полученного сообщения и извлечение из него данных."""
        try:
            # calib = struct.unpack(">H", data[3:5])[0] / 10.0  # Калибровочное значение
            tds = struct.unpack(">H", data[5:7])[0] / 10.0  # Значение TDS 
            return (tds)
        except Exception as e:
            print("Parce error:", e)
            return (0.0, 0.0)


    def recieve_callback(self, msg):
        """Обработка полученных данных."""
        data = msg.data

        if self.check_crc(data) == False:
            print("crc parse error")
            return

        if self.check_idx(data) == False:
            return
        
        print(self.parce_msg(data))


    def get_rqst_msg (self):
        """Подготовка сообщения запроса к датчику с корректным CRC."""
        l = self.rqst_msg
        crc = libscrc.modbus(bytes(l[0:(len(l) - 2)]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        l[6] = crcLow
        l[7] = crcHigh
        return l


    def request_thread (self):
        """Отправка запросов к датчику с заданным интервалом."""
        while True:
            request_message = self.get_rqst_msg()
            # print(request_message)
            self.send_message(request_message)
            time.sleep(1)

    
    def send_message(self, message):
        """Отправка сообщения датчику через интерфейс RS485."""
        try:
            msg = UInt8MultiArray()

            for item in message:
                msg.data.append(item)
            
            self.tx_.publish(msg)
        except Exception as e:
            print("Error send message: ", e)


def main(args=None):
    rclpy.init(args=args)
    ec_tds_sensor = ECTDSSensor()
    rclpy.spin(ec_tds_sensor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
