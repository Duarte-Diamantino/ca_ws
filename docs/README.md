# Documentação

| Documento | O que tem |
|---|---|
| [simulacao.md](simulacao.md) | Conduzir no Gazebo (GUI `ca_gui` e terminais), afinar o MPC e o DWA, a pista à escala real, o carro e o mundo do Gazebo |
| [testes.md](testes.md) | `colcon test`, cenários em circuito fechado (`tools/run_sim_tests.sh`) e resultados, MPC sem ROS (`tools/mpc_offline.py`) |
| [alteracoes.md](alteracoes.md) | Tudo o que mudou no código original (marcado `[ca_ws]` no código) e as calibrações diferentes do MPC e do DWA |
| [limitacoes.md](limitacoes.md) | O que ainda não está resolvido |
| [ambiente_wsl.md](ambiente_wsl.md) | WSL Ubuntu 22.04: Gazebo Fortress, rede, `~/.bashrc`, Python |

Notas antigas do `CA_SIM` (escritas para o carro real e para o workspace antigo, com caminhos como
`~/test_ws` e `~/f1tenth` do Jetson):

| Documento | O que tem |
|---|---|
| [Global.md](Global.md) | Carro real (Jetson): câmara e EKF, mapa e localização global (`point_cloud_processor`), localização relativa, MPC/DWA |
| [terminal.md](terminal.md) | Comandos soltos: obstáculos, comando Bluetooth, RustDesk, LEDs, VESC Tool, tópicos, rosbag |
| [rviz_readme.md](rviz_readme.md) | Mapa e URDF no RViz com o `pickup_description`, spline e trajetória quadrada |
| [gazebo_wsl_setup.md](gazebo_wsl_setup.md) | O README do `CA_SIM`: a primeira simulação no Gazebo no WSL2 (substituída pelo `ca_gazebo`) |

Cada pacote em `src/` tem também um `README.md`.
