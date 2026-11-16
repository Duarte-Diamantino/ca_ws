# ca_gazebo — carro e pista no Gazebo Fortress

Normalmente é aberto pelo `ca_sim` (`tools/sim.sh`). Só o Gazebo com o carro:

```bash
ros2 launch ca_gazebo gazebo.launch.py gui:=true x:=-0.4 y:=0.0 yaw:=0.0
```

| Ficheiro | |
|---|---|
| `urdf/ca_car.urdf.xacro` | Traxxas 1/10 com as meshes do CAD (`meshes/`, de `ca_sim_ros2`) e as medidas do carro montado (entre eixos 0,324 m). Servo da direção, colisões simples e detalhes só visuais |
| `worlds/pista.sdf` + `worlds/media/pista.png` | armazém com a pista FNR no chão, à escala real (lado menor 7 m), cones e estantes |
| `scripts/gerar_mundo.py` | gera, a partir do desenho `map_200.pgm`, o mundo, a textura e o `pickup_description/maps/map.yaml` + `fnr_ca.png` |
| `config/bridge.yaml` | `ros_gz_bridge`: `/clock`, odometria, TF |
| `launch/gazebo.launch.py` | Gazebo + `robot_state_publisher` + bridge (+ o carro, com `spawn:=true`) |

Detalhes: [docs/simulacao.md](../../docs/simulacao.md#o-carro-e-o-mundo-no-gazebo-ca_gazebo).
