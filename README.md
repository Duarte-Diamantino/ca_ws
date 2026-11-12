# ca_ws — carro autónomo (ROS 2 Humble)

Workspace único que junta o que estava espalhado pelos repositórios `TESE`, `GUI_pista` e `CA_SIM`,
com testes em circuito fechado **sem carro**: num simulador cinemático (rápido) e no **Gazebo Fortress**.

```
GUI_pista ──/reference_path──► mpc_controller  ou  dwa_global_follower ──/commands/motor/speed────► VESC
obstacle_publisher ──/obstacles, /obstacles_marker──►       ▲              /commands/servo/position
                                                            │                        │
     carro real: EKF /odometry/filtered ────────────────────┤                        │
     sim:=kinematic: ca_sim/kinematic_sim ◄─────────────────┼────────────────────────┤
     sim:=gazebo:    Gazebo (ca_gazebo) ◄── ca_sim/vesc_to_gz ◄──────────────────────┘
                     └─ /odometry/filtered, /tf, /joint_states (ros_gz_bridge)
```

## Pacotes (`src/`)

| Pacote | O que é | Origem |
|---|---|---|
| `gui_pista` | GUI Tkinter: imagem da pista → Hybrid A\* + DWA + MPC + linha de corrida → publica no ROS | `GUI_pista` @ `d61a0cd` |
| `mpc_python` | MPC (scipy L-BFGS-B, modelo bicicleta) que segue `/reference_path` e evita `/obstacles` | `TESE` @ `653131a` |
| `dwa_follower` | `dwa_follower_node`, `dwa_global_follower`, `dwa_spline_follower` (C++) | `TESE` @ `653131a` |
| `obstacle_publisher` | Publica obstáculos fixos (`positions:=[x1,y1,...]`) | `TESE` @ `653131a` |
| `pickup_description` | URDF (só visual), mapa, launch do RViz/map_server, geradores de caminhos de teste (Q, spline, retângulo) | `TESE` @ `653131a` |
| `spline_visualizer` | Desenha uma parábola no RViz | `TESE` @ `653131a` |
| `ca_sim` | **Novo.** Carro simulado (cinemático ou Gazebo), ponte VESC→Gazebo, monitor PASS/FAIL, launch de testes | este repo |
| `ca_gazebo` | **Novo.** Carro 1/10 para o Gazebo Fortress (URDF com rodas, atrito e servo), mundo com o mapa da pista, bridge | este repo |

Em `docs/` estão as notas do `CA_SIM` (Jetson, RViz, terminal, Gazebo no WSL).
Em `tools/` está o `manual_control.py` da TESE e o script de testes.

## Compilar

```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Testar

```bash
# testes unitários / lint (inclui o planeador do GUI, precisa de display: WSLg serve)
PYTHONNOUSERSITE=1 colcon test && colcon test-result --verbose

# cenários em circuito fechado (MPC/DWA + carro simulado), compara com o esperado
./tools/run_sim_tests.sh            # todos (os gz_* são saltados se não houver Gazebo)
./tools/run_sim_tests.sh gz_        # só os do Gazebo

# ver um cenário
ros2 launch ca_sim sim_closed_loop.launch.py controller:=mpc rviz:=true
ros2 launch ca_sim sim_closed_loop.launch.py sim:=gazebo gui:=true rviz:=true

# só o Gazebo com o carro (sem controlador)
ros2 launch ca_gazebo gazebo.launch.py gui:=true x:=-0.4 y:=0.0 yaw:=0.0
```

Argumentos do `sim_closed_loop.launch.py`: `sim:=kinematic|gazebo`, `controller:=mpc|dwa`, `gui:=true`
(janela do Gazebo), `obstacles:="[x1,y1,x2,y2]"`, `dwa_goal_tol:=0.3`, `max_cte:=0.5`,
`min_obstacle_clearance:=0.3`, `trace:=1.0` (escreve a pose a cada 1 s), `rviz:=true`.

### Resultados atuais (12 nov 2026)

| Cenário | Cinemático | Gazebo | Notas |
|---|---|---|---|
| MPC, caminho Q, sem obstáculos | **PASS** | **PASS** | ~16,7 m em ~24 s, erro lateral máx. 0,17–0,25 m, para ~0,7 m antes do fim (de propósito: 0,8 m). No Gazebo falhou antes da correção do `obstacles_cb` (ver abaixo) |
| DWA global, caminho Q | instável | instável (FAIL 3/3) | o Q acaba em (1,1; 0), em cima da reta inicial; o DWA só olha à distância ao último ponto (sem progresso) → às vezes para logo na reta, outras dá mais uma volta com erro até ~3,5 m. Numa pista fechada acontece sempre |
| MPC com os obstáculos do `terminal.md` | **PASS** | **PASS** | só o (7,5; 2,0) interessa (0,4 m por fora da curva grande; os outros 3 estão a ~1,6 m do caminho). Contorna-o por dentro a 0,67–0,76 m, erro lateral máx. ≤ 0,42 m (6/6 corridas). Antes das alterações ao MPC (ver abaixo) parava 2 m antes |
| MPC com um obstáculo **em cima** da reta (2,5; 0) | FAIL (conhecido) | – | para ~1,5 m antes: `STOP: sem trajetória livre (curto e longo)` — ver "Limitações do MPC" |
| GUI: planear sem obstáculos | **PASS** | – | |
| GUI: contornar uma parede | xfail (conhecido) | – | Hybrid A\* encontra caminho, DWA fica preso, o segmento é saltado e o caminho final fica vazio |

O carro do Gazebo foi validado contra o modelo bicicleta: 3 s a 1 m/s → 2,78 m (esperado ~2,8 m com o
atraso de 0,2 s); 3 s a 1 m/s com 11,2° de direção → rodou 100,4° (bicicleta: 101,7°).

## O carro no Gazebo (`ca_gazebo`)

- `base_link` = centro do eixo traseiro (a referência do modelo bicicleta), distância entre eixos 0,335 m.
  **Bitola (0,25 m), raio das rodas (0,05 m) e massa (~3,7 kg) são estimativas de um 1/10 — confirmar no carro.**
- Atuação "tipo VESC", sem ros2_control: o `ca_sim/vesc_to_gz` converte `/commands/motor/speed` e
  `/commands/servo/position` (com a calibração do controlador que está a ser testado) em
  velocidade das rodas de trás (diferencial ideal) e ângulos de Ackermann das rodas da frente.
  O servo é um `JointPositionController` com velocidade máxima de 6 rad/s.
  (Não usei o plugin `AckermannSteering` do Fortress: tem o ganho da direção fixo em 1 → ~1 s de atraso.)
- A odometria é a pose "verdadeira" do modelo (`OdometryPublisher`, frame `map`) — faz de EKF perfeito.
  Ainda não há lidar nem ruído.
- O mundo `pista` tem a imagem do `map_200.pgm` no chão, na mesma posição que o `map_server` publica.
- Com `sim:=gazebo` todos os nós usam o relógio da simulação (`use_sim_time`).

## Alterações em relação ao código original

Todas as alterações no código da TESE estão marcadas com `[ca_ws]` nos comentários.

- `mpc_controller`, **contornar obstáculos** (antes parava sempre ~2 m antes):
  - **Custo de obstáculo suave**: `W_OBSTACLE_SOFT * (R_SOFT - d)²` quando `d < R_SOFT = OBSTACLE_RADIUS + 0,20`,
    com `W = 1e5`. Antes, um plano com algum estado a menos de `OBSTACLE_RADIUS` devolvia um custo fixo
    (1e9) — um patamar sem gradiente, de onde o L-BFGS-B não sai. A validação dura (raio 0,5 m) mantém-se.
  - **Modo "contornar" (rejoin sequencial) desligado** (`USE_REJOIN = False`, o código continua lá).
    Começava a referência *depois* do bloco inseguro com o carro ainda ~2 m antes; os 25 planos
    candidatos não colidiam, mas cortavam a curva 1,4–2,0 m e eram todos rejeitados pelo tubo
    (`PATH_TUBE_LIMIT = 0,55`) → `STOP: sem trajetória livre após obstáculo`.
  - **`ftol` do L-BFGS-B 0,1 → 1e-3** (`MPC_FTOL`). Com 0,1 o otimizador parava cedo e o plano ficava às
    vezes 1 cm dentro do raio duro → paragem aleatória (7/10 corridas chegavam ao fim; agora 10/10).
    Custo: `update()` ~6 → ~18 ms em média (máx. ~55 ms) num PC. **Medir na Jetson** (o ciclo é de 300 ms).
  - Diagnóstico e escolha dos valores com `tools/mpc_offline.py` (ver abaixo).
- `mpc_controller`: o `obstacles_cb` chamava `rebuild_path_structures()` **a cada mensagem de
  `/obstacles` (10 Hz)**. Essa função não usa os obstáculos, mas repõe o progresso
  (`max_index_reached`, `post_obs_bias_count`) a 0 e relocaliza o carro no ponto mais próximo de
  **todo** o caminho. No Q (o fim passa em cima da reta inicial) o índice saltava para o fim e o MPC
  parava com "fim de trajetória" a 9 % do percurso. No Gazebo aconteceu na primeira corrida; no
  simulador cinemático não apareceu nas corridas que fiz (depende de por onde o carro passa e de
  quando chega a mensagem). Agora só reconstrói na primeira mensagem (arranque).
- `dwa_global_follower`: `std::sort` → `std::stable_sort` ao ordenar o caminho por timestamp.
  Com todos os pontos com o mesmo stamp (é o que fazem `q_path_publisher`, `marker_publisher`,
  `rounded_rectangle_marker_publisher`) o `std::sort` **baralhava o caminho** a partir de 17 pontos.
- `dwa_global_follower`: `GOAL_TOL` (0,05 m fixo) passou a parâmetro `goal_tol` (default igual, 0,05 m).
  A 3 m/s e 10 Hz o carro anda 0,3 m por ciclo e nunca entrava nos 5 cm → ficava às voltas.
- `mpc_python`: removido `resource/MPC_python` (duplicado só nas maiúsculas, partia o clone em Windows/macOS).
- Manifestos (`package.xml`, `CMakeLists.txt`, `setup.py`) com as dependências que faltavam; `.rviz` instalado.
- Removidos `build/`, `install/`, `log/`, `__pycache__` e os "submódulos" vazios (`f1tenth_system`,
  `OsqpEigen`, `LBFGSpp`, não usados por nenhum pacote).

## Calibrações diferentes entre MPC e DWA ⚠️

Os dois controladores assumem calibrações diferentes para o **mesmo** carro — só uma pode estar certa:

| | MPC | DWA |
|---|---|---|
| velocidade → comando | 7000 RPM por m/s (mín. 3000) | 7690 por m/s (23070 a 3 m/s) |
| centro do servo | 0,5751 | 0,5304 |
| servo no máximo | 0,8760 | 0,94299 |
| ângulo máximo | 22,5° | 30° |
| distância entre eixos | 0,335 m | 0,335 m (GUI usa 0,324 m) |

Os simuladores usam a calibração de cada controlador (`PROFILES` em `src/ca_sim/ca_sim/vesc.py`).

## Limitações do MPC (por resolver)

- **Obstáculo em cima do caminho → o carro para.** Com `OBSTACLE_RADIUS = 0,5` e `PATH_TUBE_LIMIT = 0,55`
  sobra um corredor de 5 cm para passar ao lado. Além disso:
  - o plano longo (`H_LONG = 16`), que é o recurso quando o curto colide, usa 3 pontos do caminho por
    passo (0,45 m / 0,3 s = 1,5 m/s > `V_MAX` 1,2 m/s): a referência foge à frente do carro e nas curvas
    o plano corta 1,2–1,7 m → sai sempre do tubo;
  - `filter_ref_points_near_obstacles` apaga os pontos perto do obstáculo e repete o último, o que
    desalinha a referência no tempo e volta a ativar a penalização de 1e9 de `TRACK_HALF_WIDTH`;
  - `states_outside_tube(..., tube=PATH_TUBE_LIMIT)` fixa o valor da constante quando o módulo é carregado.
  - Para resolver: desviar a referência à volta do obstáculo (em vez de apagar pontos) e escolher o
    raio/tubo de acordo com a largura real da pista.

## Testar o MPC sem ROS (`tools/mpc_offline.py`)

Corre o `mpc_controller` verdadeiro contra o modelo bicicleta, sem ROS a correr (determinístico, ~1 s por
percurso). Quando o MPC para por falta de trajetória livre, mostra porque é que cada plano foi rejeitado.

```bash
python3 tools/mpc_offline.py "7.5,2.0"                          # obstáculos x1,y1,...
python3 tools/mpc_offline.py "2.5,0.0" --verbose                # passo a passo
python3 tools/mpc_offline.py "7.5,2.0" --latency --seed 3       # atraso do update() + perturbação aleatória
python3 tools/mpc_offline.py "7.5,2.0" --patch USE_REJOIN=1     # mudar constantes do MPC
```

## O que ainda não está aqui

- **Localização / sensores** (`point_cloud_processor`, EKF, `f1tenth_stack`): estão na Jetson (`docs/Global.md`).
  No Gazebo ainda não há lidar.
- Driver VESC (`vesc_msgs`, `vesc_driver`): vem do `f1tenth_system`.

## Ambiente (WSL Ubuntu 22.04)

### Gazebo Fortress

Instalado com `sudo apt install ros-humble-ros-gz` (o `apt update` antes é obrigatório: sem ele o
`apt install` dá 404 nos pacotes ROS).

O `ca_gazebo` não usa o `gz_sim.launch.py` do `ros_gz_sim`: esse corre o Gazebo com `shell=True` e o
SIGINT do launch fica no `/bin/sh`, deixando o servidor órfão a correr física em fundo depois de cada
teste. O `gazebo.launch.py` arranca o `ign gazebo` diretamente.

### Python

- `numpy`: há um `numpy 1.26.4` instalado com pip em `~/.local` (por causa do open3d/pandas). Funciona,
  mas o `scipy 1.8` do apt avisa que quer `<1.25`. Com `numpy 2` o MPC e o GUI nem arrancam.
  Os testes correm com `PYTHONNOUSERSITE=1` (usam o numpy 1.21 do apt).
