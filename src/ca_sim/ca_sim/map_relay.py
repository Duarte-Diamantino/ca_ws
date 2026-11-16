#!/usr/bin/env python3
"""Volta a publicar o mapa de x em x segundos para o RViz:  /map  ->  /map_rviz.

O map_server publica /map uma unica vez (transient local). No RViz 2 do Humble, o botao Reset (e o
reset automatico quando o tempo da simulacao salta para tras) apaga o mapa do ecra e NAO volta a
subscrever o topico (MapDisplay::reset so faz clear()), por isso o mapa nunca mais aparecia.
Este no guarda o ultimo /map e republica-o em /map_rviz a cada period_s (o sim.rviz usa /map_rviz):
depois de um Reset o mapa volta em <= period_s.

  ros2 run ca_sim map_relay --ros-args -p period_s:=2.0
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        period = float(self.declare_parameter('period_s', 2.0).value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map = None
        self.pub = self.create_publisher(OccupancyGrid, '/map_rviz', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_timer(period, self.republish)

    def on_map(self, msg):
        first = self.map is None
        self.map = msg
        self.pub.publish(msg)
        if first:
            self.get_logger().info(
                f'mapa {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.5f} m/px '
                'recebido: a republicar em /map_rviz')

    def republish(self):
        if self.map is not None:
            self.pub.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    node = MapRelay()
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
