import troykahat # Используем библиотеку TroykaHat для работы с аппаратным обеспечением
import time
from dataclasses import dataclass
from threading import Thread
import rclpy  # Импортируем библиотеку ROS2 для Python
from rclpy.node import Node
from std_msgs.msg import Float64
from utils import get_directory
import os
import json 
 
SCALING = 1000  # коэффициент масштабирования PWM сигнала 

@dataclass #декоратор
class MotorConfig:
    """Конфигурация отдельного мотора."""
    pin_ap: int  # Номер пина, к которому подключен мотор
    direction: int  # Направление вращения мотора (1 или -1)
    thrust_coeff: float # Коэффициент тяги для мотора


@dataclass
class MotorsConfig:
    """Конфигурация всех моторов робота."""
    left: MotorConfig  # Левый мотор
    right: MotorConfig  # Правый мотор
    back: MotorConfig  # Задний мотор

class TroykaMotorDriver:
    """Драйвер для управления моторами через TroykaHat.""" 
    delta: float = 0.5  # Разница в скорости между стоповым значением PWM и max/min значением
    stop: float = 1.5  # Значение PWM, при котором моторы находятся в состоянии покоя (не вращаются) 

    def __init__(self, config: MotorsConfig, freq: float = 500, block_time: float = 2) -> None:
        self.config = config  # Конфигурация моторов
        self.freq = freq  # Частота PWM в Гц
        self.coeff = self.freq / SCALING  # Коэффициент масштабирования для PWM
        self.block_time = block_time  # Время блокировки управления

        # Настройка пинов для управления моторами
        self.ap = troykahat.analog_io()
        self.ap._setPwmFreq(self.freq)  # Устанавливаем частоту PWM
        self.ap.pinMode(self.config.left.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.right.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.back.pin_ap, self.ap.OUTPUT)

        self.initializeMotors()  # Инициализация моторов в нейтральное положение

        self.last_time = time.time()  # Время последней команды управления
        self.alarm = False
        Thread(target=self.checkLoop, daemon=True).start()  # Запуск потока для контроля времени блокировки

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

        # Расчёт и установка тяги с учётом направления вращения, коэффициента масштабирования и коэффициентов тяги
        self.applyThrust(left, self.config.left.pin_ap, self.config.left.direction, self.config.left.thrust_coeff)
        self.applyThrust(right, self.config.right.pin_ap, self.config.right.direction, self.config.right.thrust_coeff)
        self.applyThrust(back, self.config.back.pin_ap, self.config.back.direction, self.config.back.thrust_coeff)
        
        self.last_time = time.time()  # Обновляем время последней команды

    def applyThrust(self, value: float, pin: int, direction: int, thrust_coeff: float):
        """Применение тяги к мотору."""
        p = (self.stop + value * self.delta * direction * thrust_coeff) * self.coeff
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
    def __init__(self, config_file, name='motors'):
        super().__init__(name)

        # загрузка конфигурации
        self.config = self.load_config(config_file)

        self.left = 0
        self.right = 0
        self.back = 0
        
        # Инициализация драйвера моторов с конфигурацией
        self.driver = TroykaMotorDriver(MotorsConfig(
            MotorConfig(self.config['motors']['left_pin'], self.config['motors']['left_direction'], self.config['motors']['left_thrust_coeff']),  # Левый мотор
            MotorConfig(self.config['motors']['right_pin'], self.config['motors']['right_direction'], self.config['motors']['right_thrust_coeff']),  # Правый мотор
            MotorConfig(self.config['motors']['back_pin'], self.config['motors']['back_direction'], self.config['motors']['back_thrust_coeff']),   # Задний мотор 
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

    def load_config(self, file_path):
        """Загрузка конфигурации из JSON файла."""
        with open(file_path, 'r') as f:
            config = json.load(f)
        return config     
    

    def updateThrust(self, motor: str, value: float):
        """Обновление тяги и отправка значений в драйвер."""
        max_thrust = self.config['motors']['max_thrust']
        value = min(max_thrust, max(-max_thrust, value))
        setattr(self, motor, value / 20.0)  # Преобразование значения тяги
        self.send_thrust()  # Отправка обновлённых значений тяги в драйвер

    def send_thrust(self):
        """Отправка значений тяги в драйвер моторов."""
        self.driver.setThrust(self.left, self.right, self.back)


def main(args=None):
    rclpy.init(args=args)
    # Получаем путь к директории booblik
    booblik_dir = get_directory(target='booblik') 
    config_file = os.path.join(booblik_dir, 'config.json')
    task = MotorsNode(config_file=config_file)
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
