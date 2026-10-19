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
import time

class auto_teleop_es(Node):
    def __init__(self):
        super().__init__('auto_teleop_es')

        self.pub = self.create_publisher(TwistStamped, '/controller/reference', 1)
        self.sub_back_center = self.create_subscription(LaserScan, '/my_robot/sensor/back_center', self.backCenter, 10)
        self.sub_back_left = self.create_subscription(LaserScan, '/my_robot/sensor/back_left', self.backLeft, 10)
        self.sub_back_right = self.create_subscription(LaserScan, '/my_robot/sensor/back_right', self.backRight, 10)
        self.sub_front_center = self.create_subscription(LaserScan, '/my_robot/sensor/front_center', self.frontCenter, 10)
        self.sub_front_left = self.create_subscription(LaserScan, '/my_robot/sensor/front_left', self.frontLeft, 10)
        self.sub_front_right = self.create_subscription(LaserScan, '/my_robot/sensor/front_right', self.frontRight, 10)
        self.sub_left_back = self.create_subscription(LaserScan, '/my_robot/sensor/left_back', self.leftBack, 10)
        self.sub_left_front = self.create_subscription(LaserScan, '/my_robot/sensor/left_front', self.leftFront, 10)
        self.sub_left_mid = self.create_subscription(LaserScan, '/my_robot/sensor/left_mid', self.leftMid, 10)
        self.sub_right_back = self.create_subscription(LaserScan, '/my_robot/sensor/right_back', self.rightBack, 10)
        self.sub_right_front = self.create_subscription(LaserScan, '/my_robot/sensor/right_front', self.rightFront, 10)
        self.sub_right_mid = self.create_subscription(LaserScan, '/my_robot/sensor/right_mid', self.rightMid, 10)
        
        self.dist_back_center = 0.0
        self.dist_back_left = 0.0
        self.dist_back_right = 0.0
        self.dist_front_center = 0.0
        self.dist_front_left = 0.0
        self.dist_front_right = 0.0
        self.dist_left_back = 0.0
        self.dist_left_front = 0.0
        self.dist_left_mid = 0.0
        self.dist_right_back = 0.0
        self.dist_right_front = 0.0
        self.dist_right_mid = 0.0
        
        #место для важных констант/переменных
        
        self.twist = TwistStamped()
        self.twist.twist.linear.x = 0.0
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0
        
        self.MYROBOT_STOP_VEL = 0.0
        self.SENSOR_RANGE_DISTANCE = 0.4 # дистанция реакции на обект можете указать свою!

        self.MYROBOT_MAX_LIN_VEL_X = 0.2 #линейная скоровть по x можете указать свою!
        self.MYROBOT_MAX_LIN_VEL_Y = 0.2 #линейная скорость по y можете указать свою!
        self.MYROBOT_MAX_ANG_VEL = 0.5 #скорость поворота можете указать свою!

        self.target_linear_vel_x = self.MYROBOT_STOP_VEL
        self.target_linear_vel_y = self.MYROBOT_STOP_VEL
        self.target_angular_vel = self.MYROBOT_STOP_VEL

        self.SAFE_DISTANCE = 0.5
        self.CRITICAL_DISTANCE = 0.25
        self.SIDE_DISTANCE = 0.35

        self.FORWARD_SPEED = 0.2
        self.BACKWARD_SPEED = 0.15
        self.STRAFE_SPEED = 0.15
        self.TURN_SPEED = 0.5

        self.turn_direction = 1
        self.EMERGENCY_DISTANCE = 0.15

        self.last_log_time = 0.0

        self.TEST_MODE = False
        self.TEST_CASE = "DEAD_END"

    def backCenter(self, msg):
        self.dist_back_center = msg.ranges[0]
    def backLeft(self, msg):
        self.dist_back_left = msg.ranges[0]
    def backRight(self, msg):
        self.dist_back_right = msg.ranges[0]
    def frontCenter(self, msg):
        self.dist_front_center = msg.ranges[0]
    def frontLeft(self, msg):
        self.dist_front_left = msg.ranges[0]
    def frontRight(self, msg):
        self.dist_front_right = msg.ranges[0]
    def leftBack(self, msg):
        self.dist_left_back = msg.ranges[0]
    def leftFront(self, msg):
        self.dist_left_front = msg.ranges[0]
    def leftMid(self, msg):
        self.dist_left_mid = msg.ranges[0]
    def rightBack(self, msg):
        self.dist_right_back = msg.ranges[0]
    def rightFront(self, msg):
        self.dist_right_front = msg.ranges[0]
    def rightMid(self, msg):
        self.dist_right_mid = msg.ranges[0]
        

    def publish(self):
        self.twist.twist.linear.x = self.target_linear_vel_x
        self.twist.twist.linear.y = self.target_linear_vel_y
        self.twist.twist.angular.z = self.target_angular_vel
        self.pub.publish(self.twist)
        self.last_turn = self.target_angular_vel


    def update_velocity(self):

        # 1. Получаем показания всех 12 датчиков
        sensors = [
            self.dist_front_center,
            self.dist_front_left,
            self.dist_front_right,

            self.dist_back_center,
            self.dist_back_left,
            self.dist_back_right,

            self.dist_left_front,
            self.dist_left_mid,
            self.dist_left_back,

            self.dist_right_front,
            self.dist_right_mid,
            self.dist_right_back
        ]

        if self.TEST_MODE:

            if self.TEST_CASE == "EMERGENCY":
                # Препятствие в 10 см спереди
                sensors = [
                    0.10, 1.0, 1.0,
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0
                ]

            elif self.TEST_CASE == "DEAD_END":
                # Препятствия спереди и по бокам
                # Сзади свободно
                sensors = [
                    0.30, 0.30, 0.30,
                    1.00, 1.00, 1.00,
                    0.30, 0.30, 0.30,
                    0.30, 0.30, 0.30
                ]

        # 2. Вычисляем минимальные расстояния
        front = min(sensors[0:3])
        back = min(sensors[3:6])
        left = min(sensors[6:9])
        right = min(sensors[9:12])

        # Сбрасываем скорости
        self.target_linear_vel_x = 0.0
        self.target_linear_vel_y = 0.0
        self.target_angular_vel = 0.0

        state = "STOP"

        # 3. Проверяем корректность данных
        if any(math.isnan(d) or d <= 0 for d in sensors):

            state = "WAITING FOR SENSORS"

        # 4. Аварийная остановка
        elif any(d < self.EMERGENCY_DISTANCE for d in sensors):

            state = "EMERGENCY STOP"

        else:

            # 5. Проверяем свободные направления
            front_free = front > self.SAFE_DISTANCE
            back_free = back > self.SAFE_DISTANCE
            left_free = left > self.SIDE_DISTANCE
            right_free = right > self.SIDE_DISTANCE

            # 6. Определяем направление поворота
            if left > right + 0.1:
                self.turn_direction = 1

            elif right > left + 0.1:
                self.turn_direction = -1

            # 7. Тупик
            if not front_free and not left_free and not right_free:

                if back_free:

                    self.target_linear_vel_x = -self.BACKWARD_SPEED

                    state = "MOVING BACKWARD"

                else:

                    state = "BLOCKED"

            # 8. Препятствие спереди
            elif not front_free:

                self.target_angular_vel = (
                    self.TURN_SPEED * self.turn_direction
                )

                state = "TURNING"

            # 9. Препятствие слева
            elif not left_free and right_free:

                self.target_linear_vel_x = self.FORWARD_SPEED * 0.5
                self.target_linear_vel_y = -self.STRAFE_SPEED * 0.5

                state = "AVOIDING LEFT WALL"

            # 10. Препятствие справа
            elif not right_free and left_free:

                self.target_linear_vel_x = self.FORWARD_SPEED * 0.5
                self.target_linear_vel_y = self.STRAFE_SPEED * 0.5

                state = "AVOIDING RIGHT WALL"

            # 11. Свободный путь
            elif front_free:

                self.target_linear_vel_x = self.FORWARD_SPEED

                state = "MOVING FORWARD"

        current_time = time.monotonic()

        if current_time - self.last_log_time >= 1.0:

            self.get_logger().info(
                f"State: {state} | "
                f"Front: {front:.2f} m | "
                f"Back: {back:.2f} m | "
                f"Left: {left:.2f} m | "
                f"Right: {right:.2f} m | "
                f"Vx: {self.target_linear_vel_x:.2f} | "
                f"Vy: {self.target_linear_vel_y:.2f} | "
                f"W: {self.target_angular_vel:.2f}"
            )

            self.last_log_time = current_time

        # 13. Отправляем скорости
        if self.TEST_MODE:
            self.target_linear_vel_x = 0.0
            self.target_linear_vel_y = 0.0
            self.target_angular_vel = 0.0

        self.publish()
            
def main(args=None):
    rclpy.init(args=args)
    teleop = auto_teleop_es()
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
