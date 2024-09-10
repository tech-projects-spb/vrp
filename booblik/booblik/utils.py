import rclpy
import math


def euler_to_quaternion(yaw, pitch, roll):
    """Преобразование углов Эйлера в кватернионы для описания ориентации в пространстве"""
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


def kmph_2_mps(kmph):
    """Конвертация км/ч в м/с"""
    return kmph / 3.6

def knot_2_mps(knots):
    """Конвертация узлы в м/с"""
    return knots * 0.514444

def declin_dir(mag_var, direction):
    """Обработка направления магнитного склонения"""
    return mag_var if direction == 'E' else -mag_var