#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import threading
import math
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from random import randrange
from std_msgs.msg import String

class auto_teleop_lidar(Node):
    def __init__(self):
        super().__init__('auto_teleop_lidar')

        self.pub = self.create_publisher(TwistStamped, '/controller/reference', 1)
        self.sub_lidar = self.create_subscription(LaserScan, '/my_robot/lslidar/scan', self.lidar_callback, 10)

        # Расстояния по секторам
        self.distances_front = []
        self.distances_back = []
        self.distances_right = []
        self.distances_left = []
        self.distances_front_left = []
        self.distances_front_right = []

        # Максимальные скорости
        self.MYROBOT_STOP_VEL = 0.0
        self.MYROBOT_MAX_LIN_VEL_X = 0.2
        self.MYROBOT_MAX_LIN_VEL_Y = 0.12
        self.MYROBOT_MAX_ANG_VEL = 0.6

        # Безопасные расстояния
        self.SENSOR_RANGE_DISTANCE = 0.5
        self.SAFE_SIDE_DISTANCE = 0.25
        self.EMERGENCY_DISTANCE = 0.15

        # Расстояние, при котором поворот считается завершенным
        self.CLEAR_DISTANCE = 0.65

        # Состояние алгоритма
        self.last_turn = 0
        self.counter_stay = 0
        self.double_forward = 0

        # Направление текущего поворота:
        #  1 - налево
        # -1 - направо
        #  0 - робот не поворачивает
        self.turn_direction = 0

        # Количество циклов поворота
        self.turn_ticks = 0

        # Время последнего сканирования
        self.last_scan_time = None

        # Целевые скорости
        self.target_linear_vel_x = 0.0
        self.target_linear_vel_y = 0.0
        self.target_angular_vel = 0.0

        self.twist = TwistStamped()
        self.twist.twist.linear.x = 0.0
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0

        # Количество оставшихся циклов поворота
        self.turn_steps_left = 0

        # Направление поворота
        # 1 = налево, -1 = направо
        self.turn_direction = 0

        # Параметры пошагового поворота
        self.TURN_STEPS = 4
        self.TURN_SPEED = 0.5

        # Скорость движения вперед
        self.FORWARD_SPEED = 0.15

        # Минимальное свободное расстояние впереди
        self.CLEAR_DISTANCE = 0.5
        # Данные последнего сканирования
        self.scan = None

        # Запомненное направление поворота
        # 0 — не выбрано
        # 1 — налево
        # -1 — направо
        self.turn_direction = 0

        # Параметры движения
        self.FORWARD_SPEED = 0.15
        self.TURN_SPEED = 0.45

        # Геометрия робота
        self.ROBOT_RADIUS = 0.20
        self.SAFETY_MARGIN = 0.05

        # Расстояние, на котором проверяем возможность проезда
        self.LOOKAHEAD = 0.9

    def lidar_callback(self, msg):
        #Функция которая вызываетcя, если пришло сообщение в топик
        #В данном случае, в массив msg.ranges[] размером в 400 значений типа float, будут записаны данные 400 точек измерения лидара, которые вы можете использовать как хотите
        #Хотите - разбивайте на сегменты и записывать значения в ваши переменные, хотите, детектируйте стены и так далее

        if len(msg.ranges) != 400:
            self.get_logger().warning(f'debug {len(msg.ranges)}')
            self.last_scan_time = None
            return

        points = []
        invalid_count = 0

        for i, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                if math.isnan(distance) or distance < 0:
                    invalid_count += 1
                continue

            if distance < msg.range_min or distance <= 0:
                invalid_count += 1
                continue

            if distance > msg.range_max:
                continue

            # Угол измерения
            angle = msg.angle_min + i * msg.angle_increment

            # Переводим полярные координаты в декартовы
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            points.append((x, y))

        # Слишком много некорректных измерений
        if (
            not msg.ranges
            or invalid_count > len(msg.ranges) * 0.25
        ):
            self.scan = None
            return

        # Сохраняем готовый снимок и время получения
        self.scan = (
            points,
            self.get_clock().now().nanoseconds
        )

    def path_clearance(self, points, angle, max_distance):

        # Направление предполагаемого движения
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # Необходимое расстояние от центра до препятствия
        radius = self.ROBOT_RADIUS + self.SAFETY_MARGIN

        # Изначально предполагаем, что путь свободен
        clearance = max_distance

        for x, y in points:

            # Расстояние вдоль выбранного направления
            forward = x * cos_a + y * sin_a

            # Поперечное расстояние до препятствия
            lateral = -x * sin_a + y * cos_a

            # Проверяем препятствия внутри коридора движения
            if (
                0 < forward < clearance
                and abs(lateral) < radius
            ):
                clearance = forward

        return clearance

    def choose_turn_direction(self, points):

        best_score = float('-inf')
        best_direction = 1

        # Проверяем направления слева и справа
        for direction in (1, -1):

            # Углы от 10 до 90 градусов
            for degrees in range(10, 91, 10):

                angle = math.radians(degrees) * direction

                # Насколько далеко можно проехать
                distance = self.path_clearance(
                    points,
                    angle,
                    1.5
                )

                # Небольшое предпочтение меньшим поворотам
                score = distance - 0.1 * (degrees / 90)

                if score > best_score:

                    best_score = score
                    best_direction = direction

        return best_direction

    def publish(self):
        self.twist.twist.linear.x = self.target_linear_vel_x
        self.twist.twist.linear.y = self.target_linear_vel_y
        self.twist.twist.angular.z = self.target_angular_vel
        self.pub.publish(self.twist)
        self.last_turn = self.target_angular_vel

    def update_velocity(self):

        # По умолчанию останавливаем робота
        self.target_linear_vel_x = 0.0
        self.target_linear_vel_y = 0.0
        self.target_angular_vel = 0.0

        # Нет данных лидара
        scan = self.scan

        if scan is None:
            self.publish()
            return

        points, scan_time = scan

        # Проверяем актуальность данных
        now = self.get_clock().now().nanoseconds
        age = now - scan_time

        if age < 0 or age > 1_000_000_000:
            self.publish()
            return

        # Проверяем свободное пространство впереди
        front = self.path_clearance(
            points,
            0.0,
            self.LOOKAHEAD
        )

        # 1. Впереди свободно
        if front >= self.LOOKAHEAD:

            self.target_linear_vel_x = self.FORWARD_SPEED

            # Направление поворота сохраняем!

        # 2. Впереди препятствие
        else:

            # Если направление еще не выбрано,
            # ищем свободный проход
            if self.turn_direction == 0:

                self.turn_direction = self.choose_turn_direction(
                    points
                )

                direction_name = (
                    "LEFT" if self.turn_direction == 1 else "RIGHT"
                )

                self.get_logger().info(
                    f"Selected direction: {direction_name}"
                )

            # Продолжаем поворачивать
            # в ранее выбранную сторону
            self.target_angular_vel = (
                self.turn_direction * self.TURN_SPEED
            )

        # Отправляем команду
        self.publish()
                
def main(args=None):
    rclpy.init(args=args)
    teleop = auto_teleop_lidar()
    thread = threading.Thread(target=rclpy.spin, args=(teleop,), daemon=True)
    rate = teleop.create_rate(10)
    thread.start()
    try:
        while rclpy.ok():
            teleop.update_velocity()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally: # обнуляем значение скорости
        teleop.target_linear_vel_x = teleop.MYROBOT_STOP_VEL
        teleop.target_linear_vel_y = teleop.MYROBOT_STOP_VEL
        teleop.target_angular_vel = teleop.MYROBOT_STOP_VEL
        teleop.publish()
        rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()
