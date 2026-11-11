#!/usr/bin/env python3
"""Faz de VESC + servo para o carro no Gazebo.

  /commands/motor/speed    (Float64, RPM)  --> velocidade das rodas de tras [rad/s]
  /commands/servo/position (Float64, 0..1) --> angulo das rodas da frente [rad] (Ackermann)

Publica em 4 topicos Float64 (prefixo gz_cmd_prefix) que o ros_gz_bridge passa para os
plugins JointController / JointPositionController do modelo (ver ca_gazebo/urdf).
A velocidade tem um atraso de 1a ordem (tau_speed), como no kinematic_sim; o atraso
do servo e feito pelo JointPositionController no Gazebo (ganho e velocidade maxima).
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from ca_sim.vesc import ackermann_angles, declare_calibration, rear_wheel_speeds


class VescToGz(Node):
    def __init__(self):
        super().__init__('vesc_to_gz')
        self.calib, self.L = declare_calibration(self)
        p = self.declare_parameter
        p('track', 0.25)             # distancia entre rodas (esquerda-direita) [m]
        p('wheel_radius', 0.05)
        p('rate_hz', 100.0)
        p('tau_speed', 0.2)
        p('cmd_timeout', 0.5)
        p('gz_cmd_prefix', '/ca_car/cmd')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.track = float(g('track'))
        self.r = float(g('wheel_radius'))
        self.dt = 1.0 / float(g('rate_hz'))
        self.tau_v = max(1e-3, float(g('tau_speed')))
        self.cmd_timeout = float(g('cmd_timeout'))
        pre = g('gz_cmd_prefix').rstrip('/')

        self.v_cmd = 0.0
        self.v = 0.0
        self.delta_cmd = 0.0
        self.last_cmd_time = None

        mk = lambda n: self.create_publisher(Float64, f'{pre}/{n}', 10)  # noqa: E731
        self.pub_steer_l = mk('steer_left')
        self.pub_steer_r = mk('steer_right')
        self.pub_wheel_l = mk('wheel_rear_left')
        self.pub_wheel_r = mk('wheel_rear_right')
        self.create_subscription(Float64, '/commands/motor/speed', self.speed_cb, 10)
        self.create_subscription(Float64, '/commands/servo/position', self.steer_cb, 10)
        self.create_timer(self.dt, self.step)
        c = self.calib
        self.get_logger().info(
            f'vesc_to_gz: L={self.L} track={self.track} r={self.r} k_speed={c.k_speed} '
            f'servo_c={c.servo_center} steer_max={math.degrees(c.steer_max):.1f} deg')

    def speed_cb(self, msg):
        self.v_cmd = self.calib.speed(msg.data)
        self.last_cmd_time = self.get_clock().now()

    def steer_cb(self, msg):
        self.delta_cmd = self.calib.steer(msg.data)
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.v_cmd = 0.0
        a = self.dt / (self.tau_v + self.dt)
        self.v += a * (self.v_cmd - self.v)

        left, right = ackermann_angles(self.delta_cmd, self.L, self.track)
        vl, vr = rear_wheel_speeds(self.v, self.delta_cmd, self.L, self.track)
        for pub, val in ((self.pub_steer_l, left), (self.pub_steer_r, right),
                         (self.pub_wheel_l, vl / self.r), (self.pub_wheel_r, vr / self.r)):
            pub.publish(Float64(data=float(val)))


def main(args=None):
    rclpy.init(args=args)
    node = VescToGz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
