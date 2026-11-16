#!/usr/bin/env python3
"""Faz de VESC + servo para o carro no Gazebo.

  /commands/motor/speed    (Float64, RPM)  --> velocidade das rodas de tras [rad/s]
  /commands/servo/position (Float64, 0..1) --> angulo das rodas da frente [rad] (Ackermann)

Publica em 4 topicos Float64 (prefixo gz_cmd_prefix) que o ros_gz_bridge passa para os
plugins JointController / JointPositionController do modelo (ver ca_gazebo/urdf).
A velocidade tem um atraso de 1a ordem (tau_speed), como no kinematic_sim; o atraso
do servo e feito pelo JointPositionController no Gazebo (ganho e velocidade maxima).

Tambem publica /joint_states (joint_states_hz) para o robot_state_publisher / RViz: direcao
com a mesma velocidade maxima do servo do Gazebo (servo_rate) e rodas a girar com a velocidade.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from ca_sim.vesc import (CAR, PROFILES, VescCalibration, ackermann_angles, declare_calibration,
                         rear_wheel_speeds)

JOINTS = ['front_left_steering_joint', 'front_right_steering_joint', 'front_left_wheel_joint',
          'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']


class VescToGz(Node):
    def __init__(self):
        super().__init__('vesc_to_gz')
        # 'wheelbase' e a geometria do carro (CAR, igual ao URDF), nao a do controlador
        self.calib, self.L = declare_calibration(self)
        p = self.declare_parameter
        # '' = usa os parametros de calibracao acima; 'mpc' | 'dwa' = calibracao desse controlador.
        # Pode mudar com o carro a andar: ros2 param set /vesc_to_gz profile dwa
        # (o ca_sim/drive faz isto sozinho quando um controlador comeca a conduzir)
        p('profile', '')
        p('track', CAR['track'])     # distancia entre rodas (esquerda-direita) [m]
        p('wheel_radius', CAR['wheel_radius'])
        p('servo_rate', 6.0)         # [rad/s], igual ao URDF (so para o /joint_states)
        p('joint_states_hz', 50.0)
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
        self.servo_rate = float(g('servo_rate'))
        self.js_every = max(1, round(float(g('rate_hz')) / float(g('joint_states_hz'))))
        self.js_count = 0
        self.steer_pos = [0.0, 0.0]          # esquerda, direita
        self.wheel_pos = [0.0] * 4           # fl, fr, rl, rr
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        mk = lambda n: self.create_publisher(Float64, f'{pre}/{n}', 10)  # noqa: E731
        self.pub_steer_l = mk('steer_left')
        self.pub_steer_r = mk('steer_right')
        self.pub_wheel_l = mk('wheel_rear_left')
        self.pub_wheel_r = mk('wheel_rear_right')
        self.create_subscription(Float64, '/commands/motor/speed', self.speed_cb, 10)
        self.create_subscription(Float64, '/commands/servo/position', self.steer_cb, 10)
        self.create_timer(self.dt, self.step)
        if g('profile'):
            self._apply_profile(g('profile'))
        else:
            self._log_calib('parametros')
        self.add_on_set_parameters_callback(self._on_params)

    def _log_calib(self, origem):
        c = self.calib
        self.get_logger().info(
            f'calibracao ({origem}): L={self.L} k_speed={c.k_speed:.0f} servo_c={c.servo_center} '
            f'steer_max={math.degrees(c.steer_max):.1f} deg | track={self.track} r={self.r}')

    def _apply_profile(self, name):
        """So muda a calibracao dos comandos; a geometria (wheelbase/track) e a do carro."""
        if name not in PROFILES:
            raise ValueError(f'perfil desconhecido: {name} (ha: {list(PROFILES)})')
        self.calib = VescCalibration.from_profile(name)
        self._log_calib(f'perfil {name}')

    def _on_params(self, params):
        for prm in params:
            if prm.name == 'profile' and prm.value:
                if prm.value not in PROFILES:
                    return SetParametersResult(successful=False,
                                               reason=f'perfil desconhecido: {prm.value}')
                self._apply_profile(prm.value)
        return SetParametersResult(successful=True)

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
        self._joint_states(left, right, vl, vr)

    def _joint_states(self, left, right, vl, vr):
        step = self.servo_rate * self.dt
        for i, target in enumerate((left, right)):
            self.steer_pos[i] += max(-step, min(step, target - self.steer_pos[i]))
        for i, v in enumerate((vl, vr, vl, vr)):
            self.wheel_pos[i] = (self.wheel_pos[i] + v / self.r * self.dt) % (2 * math.pi)
        self.js_count += 1
        if self.js_count % self.js_every:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [float(v) for v in self.steer_pos + self.wheel_pos]
        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = VescToGz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # [ca_ws] Ctrl+C no launch
        pass
    except Exception:
        if rclpy.ok():  # erro a serio; com o ROS ja a fechar e so o Ctrl+C a meio do spin
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
