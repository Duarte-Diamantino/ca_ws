# ca_gui — GUI da simulação

GUI em Qt (PyQt5) para a simulação:
- o mapa da pista à escala real (a textura do chão do Gazebo) e o carro em tempo real;
- caminhos que seguem a estrada;
- obstáculos, que aparecem também no Gazebo;
- conduzir com o MPC, o DWA ou o teclado;
- afinar o MPC.

```bash
~/ca_ws/tools/sim.sh path:=gui          # abre-a com a simulação (Gazebo + RViz)
ros2 run ca_gui ca_gui [--map map.yaml] [--texture pista.png] [--obstacles "x1,y1,x2,y2"]
```

Como se usa: [docs/simulacao.md](../../docs/simulacao.md#com-a-gui-ca_gui).

| Ficheiro | |
|---|---|
| `app.py` | janela principal: caminho, obstáculos, conduzir, separador do MPC, log |
| `view.py` | o mapa (`QGraphicsView` em metros): caminho, pontos arrastáveis, obstáculos, carro, rasto, plano do MPC |
| `track.py` | a estrada a partir do mapa e caminhos pela estrada entre pontos (Dijkstra numa grelha de 5 cm) |
| `ros_io.py` | ROS e Gazebo: publica o caminho, os obstáculos e os comandos; arranca o MPC/DWA; põe os obstáculos no Gazebo |

- **Publica:** `/reference_path` (latched), `/obstacles` (`PoseArray`, `z` = raio), `/obstacles_marker`,
  `/commands/motor/speed` e `/commands/servo/position` (teclado) e `/ca_sim/driver`.
- **Subscreve:** `/odometry/filtered` e `/predicted_trajectories` (plano do MPC).
- **Obstáculos no Gazebo:** modelos `ca_obs_<x>_<y>_<r>` (em mm). A GUI acerta o Gazebo com a lista dela
  no arranque, a cada mudança e de 15 em 15 s, e tira-os ao fechar.

Testes: `colcon test --packages-select ca_gui`. Precisam de display: o WSLg serve, ou
`QT_QPA_PLATFORM=offscreen`.
