import troykahat # Используем библиотеку TroykaHat для работы с аппаратным обеспечением
import time
from dataclasses import dataclass
from threading import Thread
import rclpy  # Импортируем библиотеку ROS2 для Python
from rclpy.node import Node
from std_msgs.msg import Float64


@dataclass #декоратор
class MotorConfig:
    """Конфигурация отдельного мотора."""
    pin_ap: int  # Номер пина, к которому подключен мотор
    direction: int  # Направление вращения мотора (1 или -1)


@dataclass
class MotorsConfig:
    """Конфигурация всех моторов робота."""
    left: MotorConfig  # Левый мотор
    right: MotorConfig  # Правый мотор
    back: MotorConfig  # Задний мотор


class TroykaMotorDriver:
    """Драйвер для управления моторами через TroykaHat."""
    config: MotorsConfig
    delta: float = 0.5
    stop: float = 1.5
    freq: float = 500
    block_time: float = 1

    def __init__(self, config: MotorsConfig, freq: float = 500, block_time: float = 1) -> None:
        self.config = config  # Сохранение конфигурации моторов
        self.freq = freq # Частота PWM сигнала
        # Настройка пинов для управления моторами
        self.ap = troykahat.analog_io()
        self.ap._setPwmFreq(self.freq)  # Устанавливаем частоту PWM
        # Установка режима работы пинов
        self.ap.pinMode(self.config.left.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.right.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.back.pin_ap, self.ap.OUTPUT)

        """Инициализация моторов в нейтральное положение."""
        self.coeff = self.freq / 1000
        p = self.stop * self.coeff
        self.ap.analogWrite(self.config.left.pin_ap, p)
        self.ap.analogWrite(self.config.right.pin_ap, p)
        self.ap.analogWrite(self.config.back.pin_ap, p)

        time.sleep(2)

        self.coeff = self.freq / 1000
        self.last_time = time.time()
        self.block_time = block_time  # Время блокировки управления после последней команды
        self.alarm = False
        Thread(target=self.checkLoop, daemon=True).start()

    def setThrust(self, left: float, right: float, back: float) -> None:
        """Установка тяги для каждого из моторов."""
        self.last_time = time.time()  # Обновляем время последней команды

        # Расчёт и установка тяги с учётом направления вращения и коэффициента масштабирования
        p = (self.stop + left * self.delta * self.config.left.direction) * self.coeff 
        self.ap.analogWrite(self.config.left.pin_ap, p)
        p = (self.stop + right * self.delta * self.config.right.direction) * self.coeff
        self.ap.analogWrite(self.config.right.pin_ap, p)
        p = (self.stop + back * self.delta * self.config.back.direction) * self.coeff
        self.ap.analogWrite(self.config.back.pin_ap, p)

    def checkLoop(self):
        """Проверка на превышение времени блокировки управления."""
        while True:
            now = time.time()
            if now - self.last_time > self.block_time:
                if not self.alarm:
                    self.alarm = True
                     # Возвращаем моторы в нейтральное положение при блокировке
                    p = self.stop * self.coeff
                    self.ap.analogWrite(self.config.left.pin_ap, p)
                    self.ap.analogWrite(self.config.right.pin_ap, p)
                    self.ap.analogWrite(self.config.back.pin_ap, p)
            else:
                self.alarm = False
            time.sleep(0.1)


class MotorsNode(Node):
    """Узел ROS2 для управления моторами"""
    def __init__(self, name='motors'):
        super().__init__(name)
        self.left = 0
        self.right = 0
        self.back = 0
        
        # Инициализация драйвера моторов с конфигурацией
        self.driver = TroykaMotorDriver(MotorsConfig(
            MotorConfig(0, 1),  # Левый мотор
            MotorConfig(1, -1),  # Правый мотор
            MotorConfig(2, 1),   # Задний мотор 
        ), freq=500)

        # Подписка на топики для управления тягой моторов
        self.left_ = self.create_subscription(
            Float64,
            '/booblik/thrusters/left/thrust',
            self.left_callback,
            10)
        self.left_
        
        self.right_ = self.create_subscription(
            Float64,
            '/booblik/thrusters/right/thrust',
            self.right_callback,
            10)
        self.right_
        
        self.back_ = self.create_subscription(
            Float64,
            '/booblik/thrusters/back/thrust',
            self.back_callback,
            10)
        self.back_
    
    def left_callback(self, data):
        self.left = data.data / 20.0
        self.send_thrust()
    
    def right_callback(self, data):
        self.right = data.data / 20.0
        self.send_thrust()
    
    def back_callback(self, data):
        self.back = data.data / 20.0
        self.send_thrust()
    
    def send_thrust(self):
        self.driver.setThrust(self.left, self.right, self.back)


def main(args=None):
    rclpy.init(args=args)
    task = MotorsNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
