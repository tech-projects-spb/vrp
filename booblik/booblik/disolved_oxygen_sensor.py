import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from threading import Thread
import time
import libscrc
import struct


class Sensor(Node):
    sensor_idx = 0x6  # Уникальный идентификатор датчика
    rqst_msg = [sensor_idx, 0x03, 0x00, 0x00, 0x00, 0x06, 0xFF, 0xFF]  # Шаблон запроса данных с датчика


    def __init__(self):
        super().__init__('HT_sensor')
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
        """Проверка CRC для подтверждения целостности данных."""
        size = len(data)
        if (size < 3):
            return False
        
        crc = libscrc.modbus(bytes(data[0:(size - 2)]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        if (crcLow == data[size - 2]) and (crcHigh == data[size - 1]):
            return True
        else:
            return False


    def check_idx (self, data):
        """Проверка, что данные от ожидаемого датчика."""
        if data[0] == self.sensor_idx:
            return True
        else:
            return False


    def parce_msg (self, data):
        """Разбор полученного сообщения и извлечение из него данных."""
        try:
            oxygen_saturation = struct.unpack(">f", data[3:7])[0]  # Насыщение кислородом
            oxygen_concentration = struct.unpack(">f", data[7:11])[0]  # Концентрация кислорода
            #temperature = struct.unpack(">f", data[11:15])[0]  # Температура (внутренний параметр датчика, не температура воды)
            return (oxygen_saturation, oxygen_concentration)
        except:
            print("parce error")
            return (0.0, 0.0, 0.0)


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
        """Генерация сообщения запроса с корректным CRC."""
        l = self.rqst_msg
        crc = libscrc.modbus(bytes(l[0:6]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        l[6] = crcLow
        l[7] = crcHigh
        return l


    def request_thread (self):
        """Отправка запросов к датчику с регулярным интервалом."""
        while True:
            request_message = self.get_rqst_msg()
            self.send_message(request_message)
            time.sleep(1)

    
    def send_message(self, message):
        """Отправка сообщения датчику через RS485."""
        try:
            msg = UInt8MultiArray()

            for item in message:
                msg.data.append(item)
            
            self.tx_.publish(msg)
        except Exception as e:
            print("Error send message: ", e)


def main(args=None):
    rclpy.init(args=args)
    sensor = Sensor()
    rclpy.spin(sensor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
