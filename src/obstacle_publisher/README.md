# obstacle_publisher

Publica obstáculos fixos:
- `/obstacles` (`PoseArray`), para o MPC;
- `/obstacles_marker` (cilindros de 0,3 m), para o DWA e o RViz;
- e ainda `/goal` e `/odom_for_dwa`.

```bash
ros2 run obstacle_publisher obstacle_publisher_node --ros-args \
  -p positions:="[8.1, 2.16, 0.54, 1.94]" -p goal:="[0.5, 2.15]"
```

Na simulação com a GUI (`path:=gui`) os obstáculos são os da GUI, e este nó fica só com o `/goal` e o
`/odom_for_dwa`.
