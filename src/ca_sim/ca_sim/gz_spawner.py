#!/usr/bin/env python3
"""Poe o carro no Gazebo no inicio do /reference_path (como o kinematic_sim faz).

Espera pelo primeiro /reference_path (ou usa x0/y0/yaw0 se init_from_path:=false) e
chama o "create" do ros_gz_sim com o URDF do topico /robot_description.
Termina depois de criar o carro (codigo 0) ou ao fim de max_attempts falhadas (codigo 1).
"""
import math
import os
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Path
from rclpy.node import Node

from ca_sim.vesc import start_pose_from_path


class GzSpawner(Node):
    def __init__(self):
        super().__init__('gz_spawner')
        p = self.declare_parameter
        p('world', 'pista')
        p('entity_name', 'ca_car')
        p('description_topic', 'robot_description')
        p('z', 0.06)
        p('x0', 0.0)
        p('y0', 0.0)
        p('yaw0', 0.0)
        p('init_from_path', True)
        p('path_topic', '/reference_path')
        p('max_attempts', 15)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.pose = None
        self.attempts = 0
        self.done = None
        self.create_exe = os.path.join(get_package_prefix('ros_gz_sim'), 'lib', 'ros_gz_sim', 'create')
        if g('init_from_path'):
            self.create_subscription(Path, g('path_topic'), self.path_cb, 10)
            self.get_logger().info(f'a espera de {g("path_topic")} para criar o carro')
        else:
            self.pose = (float(g('x0')), float(g('y0')), float(g('yaw0')))
        self.create_timer(1.0, self.try_spawn)

    def path_cb(self, msg):
        if self.pose is None and len(msg.poses) >= 2:
            pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            self.pose = start_pose_from_path(pts)

    def try_spawn(self):
        if self.done is not None or self.pose is None:
            return
        x, y, yaw = self.pose
        self.attempts += 1
        cmd = [self.create_exe, '-world', self.g('world'), '-name', self.g('entity_name'),
               '-topic', self.g('description_topic'),
               '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', f'{float(self.g("z")):.4f}',
               '-Y', f'{yaw:.5f}']
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
            out = (res.stdout + res.stderr).strip()
            ok = res.returncode == 0  # o create devolve != 0 se o servico falhar
        except subprocess.TimeoutExpired:
            ok, out = False, 'timeout'
        if ok:
            self.get_logger().info(
                f'carro criado em x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg')
            self.done = 0
        else:
            self.get_logger().warn(f'tentativa {self.attempts} falhou: {out[-300:]}')
            if self.attempts >= int(self.g('max_attempts')):
                self.get_logger().error('nao consegui criar o carro no Gazebo')
                self.done = 1


def main(args=None):
    rclpy.init(args=args)
    node = GzSpawner()
    try:
        while rclpy.ok() and node.done is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    code = node.done if node.done is not None else 1
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
