# ca_ws — carro autónomo FNR (ROS 2 Humble)

Workspace do carro autónomo 1/10 da prova de Condução Autónoma do FNR. Junta num só os repositórios
`TESE`, `GUI_pista`, `CA_SIM`, `ca_sim_ros2` e `semaforo_CA_FNR` (Duarte Diamantino) e o `fnr_thesis`
(Afonso Oliveira: mapa e localização), com uma simulação no **Gazebo Fortress** e testes em circuito
fechado **sem carro**.

```
GUI (ca_gui) ──/reference_path──► mpc_controller  ou  dwa_global_follower ──/commands/motor/speed────► VESC
          └───/obstacles, /obstacles_marker──►      ▲                       /commands/servo/position
                                                    │                                 │
  carro real: point_cloud_processor (mapa + AMCL) + EKF ── /odometry/filtered          │
  sim:=kinematic: ca_sim/kinematic_sim ◄───────────┼───────────────────────────────────┤
  sim:=gazebo:    Gazebo (ca_gazebo) ◄── ca_sim/vesc_to_gz ◄───────────────────────────┘
                  └─ /odometry/filtered, /tf, /joint_states (ros_gz_bridge)
```

## Começar

```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install     # 1.a vez ~10 min (o point_cloud_processor demora ~7)
source install/setup.bash
~/ca_ws/tools/sim.sh path:=gui     # Gazebo + RViz + GUI
```

Na GUI: clica pontos na pista, **Publicar caminho**, e depois **MPC** ou **DWA** (ou **Teclado**). Tudo
sobre a simulação em [docs/simulacao.md](docs/simulacao.md).

Para a simulação não é preciso o pacote do Afonso:
`colcon build --symlink-install --packages-skip point_cloud_processor`.

## Estrutura

```
ca_ws/
├── src/      pacotes ROS 2 (tabela abaixo; cada um tem um README)
├── tools/    sim.sh (simulação e terminais de condução), run_sim_tests.sh, mpc_offline.py, manual_control.py
├── docs/     documentação (índice em docs/README.md)
└── extras/   não é compilado: código antigo do CA_SIM e o semáforo da prova (Raspberry Pi)
```

| Pacote (`src/`) | O que é | Origem |
|---|---|---|
| [`ca_gui`](src/ca_gui) | GUI da simulação (Qt): mapa à escala real, caminhos pela estrada, obstáculos no Gazebo, conduzir e afinar MPC/DWA, teclado | novo |
| [`ca_sim`](src/ca_sim) | Carro simulado (cinemático ou Gazebo), ponte VESC→Gazebo, terminais de condução, monitor PASS/FAIL, launches | novo |
| [`ca_gazebo`](src/ca_gazebo) | Carro 1/10 para o Gazebo Fortress (modelo URDF/Xacro único com CAD, rodas e servo), mundo com a pista, bridge | novo |
| [`mpc_python`](src/mpc_python) | MPC (scipy L-BFGS-B, modelo bicicleta): segue `/reference_path` e contorna `/obstacles` | TESE |
| [`dwa_follower`](src/dwa_follower) | DWA (C++): `dwa_global_follower` segue `/reference_path` e contorna os obstáculos | TESE |
| [`obstacle_publisher`](src/obstacle_publisher) | Obstáculos fixos (`positions:=[x1,y1,...]`) | TESE |
| [`pickup_description`](src/pickup_description) | Mapas da pista (incl. o do Afonso), `map_server`, caminhos de teste (Q, spline, retângulo), URDF simples | TESE |
| [`spline_visualizer`](src/spline_visualizer) | Desenha uma parábola no RViz | TESE |
| [`gui_pista`](src/gui_pista) | GUI antiga (Tkinter): imagem da pista → Hybrid A\* + DWA + MPC + linha de corrida | GUI_pista |
| [`point_cloud_processor`](src/point_cloud_processor) | **Afonso Oliveira.** Carro real: mapa da pista + AMCL (localização global), EKF, localização pelas linhas (ZED), mapas a partir da nuvem de pontos, deteção de sinais | fnr_thesis |
| [`led_controller`](src/led_controller) | LEDs do carro conforme o sinal de trânsito detetado (`/signal`) | ca_sim_ros2 |

## Documentação

| | |
|---|---|
| [docs/simulacao.md](docs/simulacao.md) | conduzir no Gazebo (GUI e terminais), afinar o MPC e o DWA, a pista à escala real, o carro e o mundo |
| [docs/testes.md](docs/testes.md) | testes unitários, cenários em circuito fechado e resultados, MPC sem ROS |
| [docs/alteracoes.md](docs/alteracoes.md) | o que mudou no código original (marcado `[ca_ws]`) e as calibrações MPC vs DWA |
| [docs/limitacoes.md](docs/limitacoes.md) | o que ainda não está resolvido |
| [docs/ambiente_wsl.md](docs/ambiente_wsl.md) | WSL: Gazebo, rede, `~/.bashrc`, Python |
| [docs/Global.md](docs/Global.md) | carro real (Jetson): sensores, mapa e localização, controladores |
| [docs/README.md](docs/README.md) | índice completo (inclui as notas antigas do CA_SIM) |

## Origem

| Repositório | Commit | Para onde foi |
|---|---|---|
| [Duarte-Diamantino/TESE](https://github.com/Duarte-Diamantino/TESE) | `653131a` | `src/mpc_python`, `dwa_follower`, `obstacle_publisher`, `pickup_description`, `spline_visualizer`; `tools/manual_control.py` |
| [Duarte-Diamantino/GUI_pista](https://github.com/Duarte-Diamantino/GUI_pista) | `d61a0cd` | `src/gui_pista` |
| [Duarte-Diamantino/CA_SIM](https://github.com/Duarte-Diamantino/CA_SIM) | `ecbc295` | notas em `docs/`, código antigo em `extras/CA_SIM`, medidas do carro para o `ca_gazebo` |
| [Duarte-Diamantino/ca_sim_ros2](https://github.com/Duarte-Diamantino/ca_sim_ros2) | `c29202d` | `src/led_controller`; meshes do CAD integradas no modelo único do `ca_gazebo` |
| [Duarte-Diamantino/semaforo_CA_FNR](https://github.com/Duarte-Diamantino/semaforo_CA_FNR) | `95a5bab` | `extras/semaforo_CA_FNR` |
| [olivas-afonso/fnr_thesis](https://github.com/olivas-afonso/fnr_thesis) (Afonso Oliveira) | `e0570de` | `src/point_cloud_processor` (sem os pesos `.onnx`, 231 MB) |

Os repositórios originais continuam no GitHub, com o histórico. O mapa da pista do Afonso
(`fnr_thesis/maps/map.png`) já vinha no TESE como `pickup_description/maps/map.pgm` (igual pixel a
pixel); agora tem o `map_afonso.yaml` e o resto (AMCL, criação de mapas) está no `point_cloud_processor`.
Não vieram os submódulos vazios do TESE (`f1tenth_system`, `OsqpEigen`, `LBFGSpp`): o driver do VESC vem
do `f1tenth_system` no Jetson.

Todas as alterações ao código original estão marcadas com `[ca_ws]` e explicadas em
[docs/alteracoes.md](docs/alteracoes.md).
