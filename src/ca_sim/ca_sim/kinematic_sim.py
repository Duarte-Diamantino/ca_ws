#!/usr/bin/env python3
"""Simulador cinematico (modelo bicicleta) do carro, sem Gazebo.

Fecha o circuito dos controladores (MPC / DWA) em software:
  /commands/motor/speed    (Float64, RPM ou unidades "raw" do VESC)
  /commands/servo/position (Float64, posicao do servo 0..1)
        |
        v
  modelo bicicleta + atraso de 1a ordem
        |
        v
  /odometry/filtered (nav_msgs/Odometry, frame "map")  + TF map -> base_link

A calibracao (RPM por m/s, centro/amplitude do servo, angulo maximo) e
parametrizavel, porque o MPC e o DWA da TESE assumem calibracoes diferentes.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from ca_sim.vesc import declare_calibration, start_pose_from_path


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class KinematicSim(Node):
    def __init__(self):
        super().__init__('kinematic_sim')
        # calibracao (ver ca_sim/vesc.py): wheelbase, speed_cmd_per_mps, servo_center,
        # servo_span, steer_max_rad, steer_sign. Default = calibracao do MPC.
        self.calib, self.L = declare_calibration(self)
        p = self.declare_parameter
        p('rate_hz', 50.0)
        p('tau_speed', 0.2)                  # atraso de 1a ordem [s]
        p('tau_steer', 0.08)
        p('cmd_timeout', 0.5)                # sem comandos -> trava
        p('x0', 0.0)
        p('y0', 0.0)
        p('yaw0', 0.0)
        p('init_from_path', True)            # pose inicial = inicio do /reference_path
        p('path_topic', '/reference_path')
        p('odom_topic', '/odometry/filtered')
        p('frame_id', 'map')
        p('child_frame_id', 'base_link')
        p('publish_tf', True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dt = 1.0 / float(g('rate_hz'))
        self.tau_v = max(1e-3, float(g('tau_speed')))
        self.tau_d = max(1e-3, float(g('tau_steer')))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.frame_id = g('frame_id')
        self.child_frame_id = g('child_frame_id')
        self.publish_tf = bool(g('publish_tf'))

        self.x = float(g('x0'))
        self.y = float(g('y0'))
        self.yaw = float(g('yaw0'))
        self.v = 0.0
        self.delta = 0.0
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        self.last_cmd_time = None
        self.pose_locked = not bool(g('init_from_path'))

        self.odom_pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Float64, '/commands/motor/speed', self.speed_cb, 10)
        self.create_subscription(Float64, '/commands/servo/position', self.steer_cb, 10)
        self.create_subscription(Path, g('path_topic'), self.path_cb, 10)
        self.create_timer(self.dt, self.step)
        c = self.calib
        self.get_logger().info(
            f'kinematic_sim: L={self.L} k_speed={c.k_speed} servo_c={c.servo_center} '
            f'span={c.servo_span} steer_max={math.degrees(c.steer_max):.1f} deg')

    # --- entradas -------------------------------------------------------
    def speed_cb(self, msg):
        self.v_cmd = self.calib.speed(msg.data)
        self.last_cmd_time = self.get_clock().now()
        self.pose_locked = True

    def steer_cb(self, msg):
        self.delta_cmd = self.calib.steer(msg.data)
        self.last_cmd_time = self.get_clock().now()
        self.pose_locked = True

    def path_cb(self, msg):
        if self.pose_locked or len(msg.poses) < 2:
            return
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.x, self.y, self.yaw = start_pose_from_path(pts)
        self.pose_locked = True
        self.get_logger().info(
            f'pose inicial do path: x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f} deg')

    # --- dinamica -------------------------------------------------------
    def step(self):
        now = self.get_clock().now()
        if self.last_cmd_time is not None:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.v_cmd = 0.0
        a_v = self.dt / (self.tau_v + self.dt)
        a_d = self.dt / (self.tau_d + self.dt)
        self.v += a_v * (self.v_cmd - self.v)
        self.delta += a_d * (self.delta_cmd - self.delta)

        yaw_rate = self.v * math.tan(self.delta) / self.L
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + yaw_rate * self.dt),
                              math.cos(self.yaw + yaw_rate * self.dt))

        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.frame_id
        od.child_frame_id = self.child_frame_id
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(od)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()
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
