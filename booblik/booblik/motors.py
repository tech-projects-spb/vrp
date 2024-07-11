import troykahat # Используем библиотеку TroykaHat для работы с аппаратным обеспечением
import time
from dataclasses import dataclass
from threading import Thread
import rclpy  # Импортируем библиотеку ROS2 для Python
from rclpy.node import Node
from std_msgs.msg import Float64

MAX_THRUST = 10

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

 
    config: MotorsConfig  # Хранит конфигурацию моторов
    delta: float = 0.5  # Разница в скорости между стоповым значением PWM и max/min значением
    stop: float = 1.5  # Значение PWM, при котором моторы находятся в состоянии покоя (не вращаются)
    freq: float = 500  # Частота PWM сигнала в Гц
    block_time: float = 1  # Время в секундах 

    def __init__(self, config: MotorsConfig, freq: float = 500, block_time: float = 1) -> None:
        self.config = config  # Сохранение конфигурации моторов
        self.freq = freq  # Частота PWM сигнала
        self.coeff = self.freq / 1000  # Определение коэффициента масштабирования
        # Настройка пинов для управления моторами
        self.ap = troykahat.analog_io()
        self.ap._setPwmFreq(self.freq)  # Устанавливаем частоту PWM
        # Установка режима работы пинов
        self.ap.pinMode(self.config.left.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.right.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.back.pin_ap, self.ap.OUTPUT)    
        self.initializeMotors()  # Инициализация моторов в нейтральное положение

        self.last_time = time.time()  # Время последней команды управления
        self.block_time = block_time  # Время блокировки управления после последней команды
        self.alarm = False
        Thread(target=self.checkLoop, daemon=True).start() # Запускаем поток для контроля блокировки

    def initializeMotors(self):
        """Инициализация моторов в нейтральное положение."""
        p = self.stop * self.coeff
        self.ap.analogWrite(self.config.left.pin_ap, p)
        self.ap.analogWrite(self.config.right.pin_ap, p)
        self.ap.analogWrite(self.config.back.pin_ap, p)
        time.sleep(2)  # Задержка для стабилизации системы

    def setThrust(self, left: float, right: float, back: float) -> None:
        """Установка тяги для каждого из моторов."""
        self.last_time = time.time()  # Обновляем время последней команды

        # Расчёт и установка тяги с учётом направления вращения и коэффициента масштабирования
        self.applyThrust(left, self.config.left.pin_ap, self.config.left.direction)
        self.applyThrust(right, self.config.right.pin_ap, self.config.right.direction)
        self.applyThrust(back, self.config.back.pin_ap, self.config.back.direction)
        self.last_time = time.time()  # Обновляем время последней команды

    def applyThrust(self, value: float, pin: int, direction: int):
        """Применение тяги к мотору."""
        p = (self.stop + value * self.delta * direction) * self.coeff
        self.ap.analogWrite(pin, p)

    def checkLoop(self):
        """Проверка на превышение времени блокировки управления."""
        while True: 
            if time.time() - self.last_time > self.block_time and not self.alarm:
                self.alarm = True
                self.initializeMotors()  # Возвращаем моторы в нейтральное положение при блокировке
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
        self.setupSubscribers()

    def setupSubscribers(self):
        """Настройка подписок на топики управления тягой."""
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
    
    # Обработкиа сообщений от топиков
    def left_callback(self, data): self.updateThrust('left', data.data)
    def right_callback(self, data): self.updateThrust('right', data.data)
    def back_callback(self, data): self.updateThrust('back', data.data)

    def updateThrust(self, motor: str, value: float):
        """Обновление тяги и отправка значений в драйвер."""
        value = min(MAX_THRUST, max(-MAX_THRUST, value))
        setattr(self, motor, value / 20.0)  # Преобразование значения тяги
        self.send_thrust()  # Отправка обновлённых значений тяги в драйвер

    def send_thrust(self):
        """Отправка значений тяги в драйвер моторов."""
        self.driver.setThrust(self.left, self.right, self.back)


def main(args=None):
    rclpy.init(args=args)
    task = MotorsNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
