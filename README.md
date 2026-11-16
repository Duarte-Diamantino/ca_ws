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
| `ca_gui` | **Novo.** GUI da simulação (Qt): mapa à escala real, caminhos pela estrada, obstáculos no Gazebo, conduzir (MPC/DWA/teclado), afinar o MPC | este repo |
| `gui_pista` | GUI Tkinter (antiga, `path:=gui_antigo`): imagem da pista → Hybrid A\* + DWA + MPC + linha de corrida → publica no ROS | `GUI_pista` @ `d61a0cd` |
| `mpc_python` | MPC (scipy L-BFGS-B, modelo bicicleta) que segue `/reference_path` e evita `/obstacles` | `TESE` @ `653131a` |
| `dwa_follower` | `dwa_follower_node`, `dwa_global_follower`, `dwa_spline_follower` (C++) | `TESE` @ `653131a` |
| `obstacle_publisher` | Publica obstáculos fixos (`positions:=[x1,y1,...]`) | `TESE` @ `653131a` |
| `pickup_description` | URDF (só visual), mapa, launch do RViz/map_server, geradores de caminhos de teste (Q, spline, retângulo) | `TESE` @ `653131a` |
| `spline_visualizer` | Desenha uma parábola no RViz | `TESE` @ `653131a` |
| `ca_sim` | **Novo.** Carro simulado (cinemático ou Gazebo), ponte VESC→Gazebo, terminais de condução (`drive`), monitor PASS/FAIL, launch de testes | este repo |
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

## Conduzir no Gazebo

### Com a GUI (`ca_gui`)

```bash
~/ca_ws/tools/sim.sh path:=gui
```

Abre o Gazebo, o RViz e a **GUI**: o mapa da pista à escala real (a textura do chão do Gazebo), com
o carro da simulação em tempo real (azul, velocidade, rasto de 20 s a laranja) e o plano previsto do
MPC (amarelo). Roda do rato = zoom, botão do meio (ou modo **Mover**) = arrastar, **Ajustar vista**,
**seguir o carro**.

- **Caminho** (modo **Pontos**): clicas pontos na pista e o caminho vai **pela estrada** entre eles,
  por ordem (não corta por cima das linhas contínuas; pode pisar o tracejado e a passadeira). Arrastas
  os pontos para o mudar, botão direito num ponto apaga-o, Ctrl+Z desfaz. **volta fechada** liga o
  último ao primeiro. Modo **Desenhar**: à mão (é suavizado). Também: **Caminho Q (teste)**,
  **Inverter sentido**, **Abrir/Guardar** (CSV). Curvas mais apertadas do que o carro faz (raio
  mínimo ~0,8 m) ficam a vermelho.
  **Publicar caminho** manda o `/reference_path` (republicado de 1 em 1 s); se o carro estiver parado,
  vai para o início do caminho (**Carro ao início do caminho** também o faz).
- **Obstáculos** (separador/modo **Obstáculos**): clicas para pôr um cilindro (raio à escolha), botão
  direito perto de um tira-o. Aparecem no Gazebo e vão para o `/obstacles` (MPC) e
  `/obstacles_marker` (DWA). O MPC usa o tamanho de cada um: mantém o centro do carro a raio do
  obstáculo + meia largura do carro (`CAR_HALF_WIDTH`, 0,10) + folga (`OBSTACLE_MARGIN`, 0,10); a folga
  muda-se no separador **MPC (afinar)**. **aplicar a todos** muda o raio dos que já lá estão;
  **mostrar essa distância** desenha-a a tracejado (desligado por defeito).
  O DWA também desvia o caminho à volta deles (ver **Afinar o DWA**). A GUI confirma de 15 em 15 s o
  que está mesmo no Gazebo e acerta (põe o que falta, tira o que sobra, também restos de uma GUI
  anterior); ao fechar a GUI, os obstáculos saem do Gazebo.
- **Conduzir**: **MPC** e **DWA** arrancam/param o controlador (o log aparece em baixo); **Teclado**:
  W/S (ou setas) acelera/trava, A/D vira, **e podes carregar duas ao mesmo tempo** (W+A, W+D...) —
  aqui as teclas são as reais, não o auto-repeat do terminal. Ao largar, desce até 0 e centra. O mapa
  tem de estar selecionado (clica nele). **PARAR** (ou Esc/Espaço) para tudo e trava.
- **MPC (afinar)**: os parâmetros do MPC numa tabela (passa o rato por cima para ver o que são e os
  limites). Com o MPC a correr, **Aplicar** muda já; **Guardar (os meus)** grava-os para os próximos
  arranques; **Repor os de origem** apaga os teus.
- A barra de baixo mostra sempre o carro (velocidade, pose) e o comando (velocidade, direção).
- Só um conduz de cada vez (`/ca_sim/driver`): se um terminal assumir o carro, a GUI larga-o.
- A GUI antiga continua disponível: `tools/sim.sh path:=gui_antigo`.

O `tools/sim.sh` (simulação) não abre uma segunda simulação se já houver uma aberta (diz o pid), e
fecha os Gazebos **órfãos** — os `ign gazebo` que ficaram a correr sem pai, por exemplo depois de
fechar o separador do terminal com o ✕ em vez de Ctrl+C. Com dois servidores do Gazebo ao mesmo tempo,
a GUI punha e tirava obstáculos num e a janela mostrava o outro (os obstáculos "não saíam"), e o
`/clock` e a odometria vinham dos dois. Ao sair (Ctrl+C) volta a verificar.

### Um terminal por controlador

Abre os terminais no Ubuntu (WSL) ou a partir do Windows (`wsl -d Ubuntu-22.04 --cd ~ -- ./ca_ws/tools/sim.sh mpc`):

| Terminal | Comando | Teclas |
|---|---|---|
| 1. simulação | `~/ca_ws/tools/sim.sh` (caminho Q) ou `~/ca_ws/tools/sim.sh path:=gui` | — (fecha com Ctrl+C ou fechando o Gazebo) |
| 2. MPC | `~/ca_ws/tools/sim.sh mpc` | **espaço** inicia / para o MPC; **p** mostra os parâmetros, **t** muda um (ex.: `V_MAX 1.5`), **g** grava-os |
| 3. DWA | `~/ca_ws/tools/sim.sh dwa` | **espaço** inicia / para o DWA |
| 4. comando | `~/ca_ws/tools/sim.sh remote` | como um comando de RC: **W** (ou ↑) acelera enquanto carregas, **S** (↓) trava e depois faz marcha atrás, **A/D** (←/→) viram; **ao largar, a velocidade desce até 0 e a direção volta ao centro**. **W+A / W+D / S+A / S+D**: carrega a segunda sem largar a primeira. **espaço** trava logo, **m** muda para o modo "manter" (W/S ±0,1 m/s e A/D ±2° por toque, o valor fica; **C** centra) |

Em todos os terminais de condução: **r** para e põe o carro no início do caminho, **q** sai.
A última linha de cada terminal mostra, ao vivo, o que está a ser mandado ao carro (velocidade em
m/s e rpm, direção em graus e posição do servo) e o que o carro faz (velocidade real e pose).

- No comando, o terminal não avisa quando largas uma tecla: só recebe a letra repetida pelo
  auto-repeat do teclado (~30/s, depois de ~0,5 s), e só repete a **última** tecla. A tecla conta
  como carregada até 0,12 s depois da última repetição. Duas ao mesmo tempo: se carregas A com o W
  ainda carregado, o W fica "preso" ao A enquanto o A repetir — o carro **mantém a velocidade** e vira
  (não dá para saber se o W continua carregado). Se carregas W com o A carregado, acelera e vira.
  Na GUI (botão **Teclado**) as duas teclas são lidas de verdade.
- Só um terminal conduz de cada vez: quem começa a conduzir anuncia-se em `/ca_sim/driver` e os
  outros largam o carro (o controlador deles para). Assumir o carro a alta velocidade e travar logo
  pode fazer o carro rodar (as rodas de trás perdem aderência), como no carro real.
- O controlador corre como subprocesso do terminal (`mpc_controller` / `dwa_global_follower` com
  `use_sim_time`), e o log dele aparece nesse terminal. Parar e voltar a iniciar recomeça-o do zero
  (para refazer o percurso: **r** e depois **espaço**).
- Quem conduz põe o carro simulado com a sua calibração (`ros2 param set /vesc_to_gz profile mpc|dwa`);
  o comando usa a do MPC (`tools/sim.sh remote --profile dwa` para a do DWA).
- Opções da simulação: `path:=q` (caminho Q, defeito) | `path:=gui` (GUI nova) | `path:=gui_antigo` |
  `path:=none`; `obstacles:="[8.1, 2.16]"`; `gui:=false` (sem janela do Gazebo), `rviz:=false`,
  `map:=false`.
- Sem o atalho: `ros2 launch ca_sim gazebo_sim.launch.py` e `ros2 run ca_sim drive mpc|dwa|remote`,
  com `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1` em **todos** os terminais (o
  `sim.sh` já o faz).
- O mapa no RViz vem de `/map_rviz`: o `ca_sim/map_relay` republica o `/map` de 2 em 2 s. O Reset
  do RViz 2 (Humble) apaga o mapa e não volta a subscrever; como o `map_server` só publica o mapa uma
  vez, o mapa desaparecia. Agora volta em ≤ 2 s.
- A simulação só fala por 127.0.0.1 (ver "Rede" em Ambiente). Se o terminal de condução disser
  `AVISO: a pose do carro (/odometry/filtered) nunca chegou`, o carro não vai andar: fecha a
  simulação e volta a abri-la.

### Afinar o MPC

As constantes do `mpc_controller` que interessa afinar (33: velocidades e acelerações, pesos do custo,
horizontes, solver, obstáculos e desvio, fim do caminho) são **parâmetros ROS** com o mesmo nome
(lista `TUNABLE` no `mpc_controller.py`, cada um com descrição e limites). Mudam no ciclo seguinte,
sem reiniciar o MPC; os de obstáculos/desvio recalculam logo o caminho desviado, sem perder o progresso.

- **GUI**: separador **MPC (afinar)**.
- **Terminal do MPC**: **p** (ver), **t** (mudar, ex.: `V_MAX 1.5`), **g** (gravar).
- **ROS**: `ros2 param set /mpc_controller V_MAX 1.5` (floats com ponto decimal: `2.0`, não `2`),
  `ros2 param dump /mpc_controller`, ou barras deslizantes com `ros2 run rqt_reconfigure rqt_reconfigure`
  (num terminal com `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1`).
- **Ficheiros**: o MPC arranca com `src/mpc_python/config/mpc.yaml` (os de origem) e depois
  `~/ca_ws/mpc_params.yaml` (os teus, gravados pela GUI ou pelo **g**; apagar = voltar aos de origem).
- `tools/mpc_offline.py ... --patch V_MAX=1.5` para experimentar sem ROS.

### Afinar o DWA

O `dwa_global_follower` também aceita mudanças com ele a correr:
`ros2 param set /dwa_global_follower v_max 1.5` (ou `rqt_reconfigure`). O simulador arranca-o com
`src/dwa_follower/config/dwa.yaml` (v_max 1,2, `window_pts` 15, `lookahead_dist` 1,2, `goal_tol` 0,3,
obstáculos com tamanho) e depois `~/ca_ws/dwa_params.yaml`, se existir. Os principais: `v_max`,
`window_pts` e `lookahead_dist` (até onde olha à frente: mais = corta mais as curvas), `w_path`,
`w_goal`, `w_obs`, `obs_influence`, `min_safe_distance`, `horizon_s`.

**Obstáculos em cima do caminho** (`use_ref_shift`, ligado): como o MPC, o DWA desvia o caminho para o
lado em cada passagem perto de um obstáculo, até ficar a (distância mínima + `avoid_margin`) dele —
com o raio 0,15 da GUI: 0,15 + `car_half_width` 0,10 + `obstacle_margin` 0,10 + 0,10 = 0,45 m do
centro —, com rampas de `avoid_ramp` (2 m) antes e depois. Se o obstáculo está mesmo em cima, vai para
o lado com mais espaço (empate: `avoid_default_side`, +1 = esquerda). O caminho desviado aparece no
RViz (`/reference_path_markers`). `ros2 param set /dwa_global_follower use_ref_shift false` volta ao
comportamento antigo (só vê os obstáculos no horizonte de 1 s).

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

### Resultados atuais (16 nov 2026)

| Cenário | Cinemático | Gazebo | Notas |
|---|---|---|---|
| MPC, caminho Q, sem obstáculos | **PASS** | **PASS** | 18,4 m em ~22 s, erro lateral máx. **0,09 m** (antes das alterações ao MPC: 0,17–0,25 m), para ~0,7 m antes do fim (de propósito: 0,8 m) |
| DWA global, caminho Q | **PASS** | **PASS** | erro lateral máx. 0,06–0,07 m, pára no fim (2 de 2 em cada). Até 13 nov: instável — sem progresso, a janela dava a volta ao caminho e andava a 3 m/s: parava logo na reta ou dava mais voltas com erro até ~4 m (ver "Alterações") |
| DWA com um obstáculo **em cima** da reta (2,70; 0) | **PASS** | **PASS** | desvia 0,45 m e passa a 0,42 m do centro (16 nov; antes passava a ~0,3 m, sem desviar o caminho) |
| DWA com um obstáculo **em cima** do laço (7,62; 2,58) / dois na reta / os do `terminal.md` | **PASS** | – | a 0,43–0,49 m do centro dos obstáculos |
| DWA pela GUI no Gazebo, dois obstáculos em cima da reta (raios 0,15 e 0,2) | – | **PASS** | folga carro–obstáculo 0,18 e 0,21 m, chega ao fim (MPC no mesmo: 0,25 e 0,27 m) |
| MPC com os obstáculos do `terminal.md` (×1,0793, como a pista) | **PASS** | **PASS** | só o (8,10; 2,16) interessa (0,43 m por fora da curva grande; os outros 3 estão a ~1,7 m do caminho). Contorna-o por dentro a ~0,7 m |
| MPC com um obstáculo **em cima** da reta (2,70; 0) | **PASS** | **PASS** | desvia ~0,6 m para o lado e volta ao caminho (até 12 nov parava ~1,5 m antes) |
| MPC com um obstáculo **em cima** do laço (7,62; 2,58) | **PASS** | – | idem |
| GUI: planear sem obstáculos | **PASS** | – | |
| GUI: contornar uma parede | **PASS** | – | era xfail: o DWA ficava preso atrás da parede e o segmento era deitado fora; agora usa o caminho do Hybrid A\* |

Sem ROS (`tools/mpc_offline.py --latency --seed 0..2`, 8 cenários com obstáculos na reta, no laço, por
dentro do laço, no topo, na descida e dois de uma vez): o MPC novo chega ao fim em todos, a ≥ 0,63 m
dos obstáculos, com erro lateral máx. 0,12–0,15 m e a volta em 22–24 s. O MPC antigo parava em todos
os que têm o obstáculo em cima do caminho.

O carro do Gazebo foi validado contra o modelo bicicleta: 3 s a 1 m/s → 2,78 m (esperado ~2,8 m com o
atraso de 0,2 s); 3 s a 1 m/s com 11,2° de direção → rodou 100,4° (bicicleta: 101,7°).

## A pista à escala real

- O desenho da pista FNR (`map_200.pgm`) estava a 0,0121 m/px → 9,41 × 6,49 m. **O lado menor real é
  7 m**, por isso tudo cresceu **×1,0793**: mapa (10,16 × 7,00 m), caminho Q (`q_path_publisher`, novo
  parâmetro `scale`; `scale:=1.0` dá o antigo) e os obstáculos dos testes.
- `pickup_description/maps/map.yaml` aponta agora para `fnr_ca.png`: o mesmo desenho com 2× mais
  resolução (0,0065 m/px, contornos limpos). Os `.pgm` antigos continuam lá.
- Tudo isto é gerado por `src/ca_gazebo/scripts/gerar_mundo.py` (mapa, textura do chão, cones e
  armazém). Para mudar a escala: `SHORT_SIDE_M` no script, e correr
  `python3 src/ca_gazebo/scripts/gerar_mundo.py` com o workspace carregado. A constante
  `TRACK_SCALE` em `ca_sim/vesc.py` e o `scale` do `q_path_publisher` têm de acompanhar.

## O carro e o mundo no Gazebo (`ca_gazebo`)

- **Carro: Traxxas 1/10 com as meshes do CAD** (repo `ca_sim_ros2`: `base_link.stl`, `roda_urfd_v3.stl`,
  em mm → `scale 0.001`). O URDF antigo desse repo não tinha escala e punha as rodas a ±0,5 m.
  Medidas tiradas do carro montado (`CA_SIM/carro_com_rodas.stl`): **distância entre eixos 0,324 m**,
  bitola 0,1975 m, rodas de 109 mm (raio 0,0545), carroçaria 568 × 193 × 117 mm. Os controladores
  assumem 0,335 m (ver tabela de calibrações). Colisões simples (caixa + esferas) para a física ser estável.
  A mesh da carroçaria é um bloco simples: por cima tem detalhes só visuais (sem massa nem colisão):
  vidros, retrovisores, faróis e farolins, para-choques, faixa no capot, aileron, lidar no tejadilho,
  antena e jantes.
- `base_link` = centro do eixo traseiro (a referência do modelo bicicleta).
- Atuação "tipo VESC", sem ros2_control: o `ca_sim/vesc_to_gz` converte `/commands/motor/speed` e
  `/commands/servo/position` (com a calibração do controlador que está a conduzir) em
  velocidade das rodas de trás (diferencial ideal) e ângulos de Ackermann das rodas da frente.
  O servo é um `JointPositionController` com velocidade máxima de 6 rad/s.
  (Não usei o plugin `AckermannSteering` do Fortress: tem o ganho da direção fixo em 1 → ~1 s de atraso.)
- `/joint_states` (rodas a girar e direção no RViz) vem do `vesc_to_gz` a 50 Hz. O `JointStatePublisher`
  do Gazebo publicava a **1000 Hz** (um por passo de física); o `robot_state_publisher` transformava
  cada um em TF e o RViz ficava atrasado — era por isso que o URDF não voltava ao início com o **r**.
- A odometria é a pose "verdadeira" do modelo (`OdometryPublisher`, frame `map`) — faz de EKF perfeito.
  Ainda não há lidar nem ruído.
- **Mundo: um armazém** (15,8 × 13,4 m, paredes de 3,2 m) com a pista no chão (asfalto, linhas brancas,
  passadeira), pilares, 4 estantes com caixas, paletes, faixas amarelas no chão e 17 cones fora da
  estrada (a ~25 cm das linhas e a mais de 0,7 m do caminho Q — há um teste que o garante).
  Paredes, pilares, estantes e cones são sólidos: se o carro sair da pista, bate.
- Com `sim:=gazebo` todos os nós usam o relógio da simulação (`use_sim_time`).

## Alterações em relação ao código original

Todas as alterações no código da TESE estão marcadas com `[ca_ws]` nos comentários.

- `pickup_description`: `map.yaml` à escala real (lado menor 7 m) com o novo `fnr_ca.png` (2× mais
  resolução), e `q_path_publisher` com o parâmetro `scale` (defeito 1,0793, igual ao mapa) — ver
  "A pista à escala real".

- `gui_pista` (todas as linhas alteradas marcadas com `# [ca_ws]`):
  - **Escala e origem do mapa** (`load_map_yaml`, botão "Carregar mapa (map.yaml)", `--map` ou
    `GUI_PISTA_MAP`): escala fixa 1/resolution e origem do `map.yaml`, como o `map_server`. Antes,
    qualquer imagem media sempre 4,8 m de altura (24 linhas da grelha / 5 por metro) e a origem ia para
    o início do caminho: na pista FNR as distâncias saíam ×0,686, com erros até 1,6 m; e mexer no
    slider da grelha mudava a escala. Agora o caminho Q desenhado no GUI sai com erro 0 mm.
  - **`/reference_path` latched e republicado a 1 Hz**: antes só era publicado ao carregar no botão e
    os controladores que arrancam depois (terminais MPC/DWA) nunca o recebiam.
  - Caminho reamostrado a 0,10 m ao publicar (o desenho vinha a ~6,5 mm entre pontos e o
    `dwa_global_follower` usa janelas de 50 pontos) e **rumo com o sinal certo** (45° saía −45°).
  - **DWA preso → usa o Hybrid A\***: quando a fase DWA falha mas o Hybrid A\* já chegou ao objetivo,
    segue com o caminho do Hybrid A\* (antes: 3 tentativas iguais, segmento saltado, caminho vazio).
  - Limites da pista: obstáculos dos limites com raio 0,75 × quadrícula (antes 1/3: ~2/3 da zona
    marcada continuava livre) e o "flood" já não salta por cima de linhas mais finas do que uma
    quadrícula.
  - Marcadores do GUI em `/gui_pista/markers` (eram `/obstacles_marker`, o tópico do
    `obstacle_publisher`: cada mensagem do GUI apagava a lista de obstáculos do DWA).
  - **Carro da simulação no mapa** (`mixins/carlive.py`, novo): com um `map.yaml` carregado, o GUI
    subscreve `/odometry/filtered` e desenha o carro, a velocidade e o rasto (até 4 vezes por segundo).
- `mpc_controller`, **raio de cada obstáculo** (`obs_radius`, `OBSTACLE_SIZE_FROM_Z`): o `/obstacles`
  pode trazer o tamanho de cada obstáculo em `pose.position.z` (raio). Com `OBSTACLE_SIZE_FROM_Z` ligado,
  o raio duro (distância mínima ao centro do carro) é o raio do obstáculo + `CAR_HALF_WIDTH` (0,10 m) +
  `OBSTACLE_MARGIN` (0,10 m), em vez dos 0,5 m fixos para todos; o custo suave, a validação, o filtro e
  o desvio da referência usam o raio de cada um. Sem tamanho (z = 0) fica o `OBSTACLE_RADIUS`.
  Desligado por defeito no código (não sei o que a perceção do carro real põe em z); ligado no
  `config/mpc.yaml` do simulador, onde a GUI manda o raio. `AVOID_CLEARANCE` (0,6 m absolutos)
  passou a `AVOID_MARGIN` (0,10 m além do raio duro).
- `mpc_controller`, **parâmetros ajustáveis** (`TUNABLE`, `declare_tunables`, `on_set_tunables`): as
  constantes de afinação passam a parâmetros ROS (ver "Afinar o MPC"), com os valores de origem em
  `config/mpc.yaml`. As funções com valores por defeito ligados às constantes (`resample_path`,
  `obstacles_relevant_for_points`, ...) passaram a ler o valor atual. Novo `MPC_MAXITER` (era 32 fixo).
- `mpc_controller`, **obstáculo em cima do caminho → contorna** (antes parava ~1,5 m antes).
  Era a lógica, não o solver: os pontos da referência perto do obstáculo eram apagados
  (`filter_ref_points_near_obstacles`) e qualquer plano que se afastasse mais de 0,55 m do caminho
  (`PATH_TUBE_LIMIT`) era rejeitado, por isso não havia nenhum plano aceite e o carro parava.
  - **Referência desviada** (`USE_REF_SHIFT`, `shift_path_around_obstacles`): em cada passagem perto de
    um obstáculo, o caminho é empurrado para o lado até ficar a raio duro + `AVOID_MARGIN` (0,10 m) do
    obstáculo, com rampas em cosseno de `AVOID_RAMP` = 2 m (o raio mínimo do carro é ~0,8 m). O lado
    é o oposto ao obstáculo; se o obstáculo está mesmo em cima do caminho, é o lado mais longe de
    outras partes do caminho e de outros obstáculos (no Q, o fim volta à reta inicial).
    O MPC (custo, tubo, progresso) segue esse caminho; os índices são os mesmos do caminho recebido.
    Recalcula quando `/obstacles` muda, sem mexer no progresso. O MPC não conhece a pista: o desvio
    pode sair da faixa se o obstáculo estiver junto à berma.
  - Os obstáculos a menos de raio + margem + `OBSTACLE_LOOKAROUND` (0,5 m) da referência entram no
    custo suave e na validação dura (a referência desviada já não passa a < 0,5 m deles).
- `mpc_controller`, **referência espaçada pela velocidade** (`REF_SPEED_SPACING`): o ponto k fica a
  (k+1)·v·dt do carro, com v = última velocidade comandada + 0,15 m/s. Antes eram sempre 3 pontos por
  passo (0,45 m / 0,3 s = 1,5 m/s, acima do `V_MAX` de 1,2): a referência fugia à frente, a distância
  ao ponto k passava os 0,5 m de `TRACK_HALF_WIDTH` (penalização de 1e9) e o plano cortava as curvas
  para a apanhar. Erro lateral máximo no Q: ~0,21 → ~0,13 m, com a mesma volta (~22–25 s).
- `mpc_controller`, **solver**:
  - `build_cost`: a referência local é calculada **uma vez por otimização** e não em cada avaliação do
    custo (o L-BFGS-B com diferenças finitas avalia-o 2H+1 vezes por iteração). Custo igual ao original
    (um teste compara os dois, diferença ~1e-15) e ~2,7× mais rápido por avaliação. Com o resto,
    `update()` passou de ~160 para ~50 ms em média com o PC carregado (Gazebo + RViz abertos).
  - Quando o L-BFGS-B chega às 32 iterações devolve `success=False`, e o MPC deitava fora o resultado
    e usava o warm start (não otimizado, às vezes contra o obstáculo). Agora usa o resultado se for
    melhor do que o warm start.
  - `states_outside_tube(..., tube=PATH_TUBE_LIMIT)` fixava o valor da constante ao carregar o módulo.
- Testes novos em `mpc_python/test/test_mpc_controller.py` (custo igual ao original, desvio da
  referência, circuito fechado com obstáculo na reta e na curva).
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
- `obstacle_publisher`: `declare_parameter<std::vector<double>>("positions", {})` → `std::vector<double>{}`
  (o mesmo para `goal`). No Humble o `{}` escolhe o overload com `ParameterDescriptor`: o parâmetro
  ficava sem valor e o nó abortava (`UninitializedStaticallyTypedParameterException`) se não lhe
  passassem `positions` e `goal` — e sem `/obstacles` o MPC nunca arranca.
- `dwa_global_follower`: `std::sort` → `std::stable_sort` ao ordenar o caminho por timestamp.
  Com todos os pontos com o mesmo stamp (é o que fazem `q_path_publisher`, `marker_publisher`,
  `rounded_rectangle_marker_publisher`) o `std::sort` **baralhava o caminho** a partir de 17 pontos.
- `dwa_global_follower`: `GOAL_TOL` (0,05 m fixo) passou a parâmetro `goal_tol` (default igual, 0,05 m).
  A 3 m/s e 10 Hz o carro anda 0,3 m por ciclo e nunca entrava nos 5 cm → ficava às voltas.
- `dwa_global_follower`, **o DWA "não conseguia fazer nada"** no simulador (13 nov):
  - **Progresso** (`updateProgress`, `windowClosestIdx`): o "ponto mais próximo" era procurado em
    **todo** o caminho, por isso saltava quando o caminho passa duas vezes perto do mesmo sítio. Agora
    só avança, procurando `search_ahead` (30) pontos à frente do último.
  - **Fim do caminho**: a janela do custo usava `(start + k) % N`, que perto do fim de um caminho
    aberto voltava aos primeiros pontos e puxava o carro para o início (repetia o percurso para
    sempre). Agora só dá a volta em caminhos fechados (1.º ≈ último ponto). O carro pára quando o
    progresso chega ao fim (ou a `goal_tol` do último ponto, mas só depois de 80% do caminho, como o
    MPC: o Q acaba em cima da reta inicial), com a direção ao centro, e fica parado até chegar um
    caminho novo.
  - **Velocidade** `v_max` (parâmetro; era sempre `V_MAX` = 3 m/s e nunca abaixo de 1,5 m/s): a 3 m/s, com
    30° de direção, a trajetória precisa de ~15 m/s² de aderência e o carro derrapava para fora da
    estrada. A conversão para rpm continua com a calibração de 3 m/s (`V_CAL`).
  - `obs_influence`: a recompensa "longe dos obstáculos" deixa de crescer a partir desta distância
    (era sem limite: um obstáculo a 5 m ainda afastava o carro do caminho).
  - `use_marker_size` (+ `car_half_width`, `obstacle_margin`): distância mínima a um obstáculo =
    raio do marker + meia largura + margem, como no MPC (era 0,5 m para todos).
  - Todos os parâmetros podem mudar com o DWA a correr (`ros2 param set /dwa_global_follower v_max 1.5`).
  - Valores do código = os de antes (carro real); o simulador usa `dwa_follower/config/dwa.yaml`
    (e `~/ca_ws/dwa_params.yaml` se existir), carregado pelo `tools/sim.sh dwa`, pela GUI e pelos testes.
- `dwa_global_follower`, **obstáculos em cima da pista** (16 nov):
  - **Caminho desviado** (`use_ref_shift`, `rebuildPath`, `shiftPathAroundObstacles`, `freeSide`): o
    mesmo desvio da referência do MPC, em C++. O caminho recebido fica em `raw_path_` e o `path_pts_`
    passa a ser o desviado, com os mesmos índices (o progresso não se perde quando os obstáculos
    mudam). Só é refeito quando o caminho ou os obstáculos mudam. Ligado também no código (carro real).
  - O `DELETEALL` que a GUI manda antes dos cilindros (`ns` "obstacles", pose 0) era lido como um
    obstáculo em (0, 0): agora só conta `action == ADD`.
  - Sem nenhuma trajetória segura o DWA não publicava nada e o VESC ficava com o último comando (o
    carro seguia para o obstáculo). Agora publica velocidade 0.
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
| distância entre eixos | 0,335 m | 0,335 m (o GUI e o CAD do carro dizem 0,324 m) |

Os simuladores usam a calibração de cada controlador (`PROFILES` em `src/ca_sim/ca_sim/vesc.py`).

## Limitações do MPC (por resolver)

- **Não conhece a pista.** O desvio à volta de um obstáculo é de raio duro + `AVOID_MARGIN` (com os
  obstáculos da GUI, de 0,15 m: 0,45 m do centro; sem tamanho: 0,6 m). Na reta do FNR (faixa total
  ~1,5 m) cabe, mas um obstáculo junto à berma pode mandar o carro para fora. O DWA desvia da mesma
  maneira e tem a mesma limitação.
- Obstáculos sem tamanho (o `obstacle_publisher`, ou o carro real com `OBSTACLE_SIZE_FROM_Z`
  desligado) continuam com o `OBSTACLE_RADIUS = 0,5` fixo, grande para um carro de 0,19 m de largura.
- O DWA usa o tamanho dos obstáculos só com `use_marker_size` (ligado no `dwa.yaml` do simulador); sem
  isso fica o `min_safe_distance` (0,5 m) para todos.
- Ao passar perto de um obstáculo, o MPC abranda para 0,35 m/s (`OBS_APPEARS_THRESH_FOR_SLOW`). É de
  propósito no código original e torna a passagem lenta.
- Obstáculos que se mexem: a referência é recalculada quando `/obstacles` muda (custo ~1 ms), mas não
  há previsão do movimento.
- O tempo de cálculo tem de ser **medido na Jetson** (o ciclo é de 300 ms). No PC: ~35–50 ms em média,
  máx. ~200 ms com o Gazebo e o RViz a usar ~6 núcleos.
- `filter_ref_points_near_obstacles` e o modo "contornar" (`USE_REJOIN`) continuam no código, mas com a
  referência desviada já não são usados.

## Limitações da GUI nova (`ca_gui`)

- A estrada sai do desenho da pista: zonas pretas fechadas pelas linhas contínuas + linhas pintadas
  pequenas (tracejado, pontos). A caixa da passadeira tem linhas grossas, por isso está marcada à mão
  como pisável (`CROSSABLE_ZONES` em `ca_gui/track.py`); outro mapa pode precisar de outras zonas.
- O caminho escolhe a faixa mais curta entre os pontos; para andar numa faixa, põe os pontos nela.
- Curvas mais apertadas do que o carro faz são assinaladas a vermelho, não corrigidas.
- O caminho desenhado não desvia dos obstáculos: são o MPC e o DWA que o desviam (cada um a partir
  do caminho original).
- Os obstáculos são cilindros de 0,3 m de altura (raio à escolha).

## Limitações do GUI_pista antigo (por resolver)

- As junções entre segmentos não são suavizadas antes de publicar: numa volta planeada com 13
  objetivos houve junções com curvatura até 4,1 1/m (o carro faz no máximo ~1,8 1/m). Carregar em
  "Race line (rápida)" antes de publicar ajuda.
- Com a animação ligada, o planeamento é lento: ~0,5 s por passo, uma volta inteira demora vários
  minutos. Sem animação, ~20 s.
- O exterior da pista são faixas estreitas que o "flood" não apanha (~84 % fica livre).
- "Deformar imagem" não é suportado com um `map.yaml` carregado.

## Testar o MPC sem ROS (`tools/mpc_offline.py`)

Corre o `mpc_controller` verdadeiro contra o modelo bicicleta, sem ROS a correr (determinístico, ~1 s por
percurso). Quando o MPC para por falta de trajetória livre, mostra porque é que cada plano foi rejeitado.

```bash
python3 tools/mpc_offline.py "8.1,2.16"                          # obstáculos x1,y1,...
python3 tools/mpc_offline.py "2.5,0.0" --verbose                # passo a passo
python3 tools/mpc_offline.py "8.1,2.16" --latency --seed 3       # atraso do update() + perturbação aleatória
python3 tools/mpc_offline.py "8.1,2.16" --patch USE_REJOIN=1     # mudar constantes do MPC
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

### Rede (WSL)

O WSL acompanha a rede do Windows (Wi-Fi, Tailscale/VPN). Por defeito, o Gazebo (ign-transport) e o ROS
(DDS) anunciam-se em todas as interfaces que existem quando arrancam. Se a rede muda com a simulação
aberta (o PC adormece, troca de Wi-Fi, a VPN liga), a descoberta parte-se. O terminal 1 enche-se de
`Exception sending a multicast message: Network is unreachable`, a ponte deixa de passar a odometria
e o MPC fica "a conduzir" com o carro parado. Por isso:

- o `tools/sim.sh` usa sempre o `ROS_DOMAIN_ID=42` (o do `~/.bashrc`; outro: `CA_SIM_DOMAIN_ID=N`):
  um terminal do Ubuntu lê o `~/.bashrc` e um aberto a partir do Windows (`wsl ... sim.sh`) não, e com
  domínios diferentes os terminais não se veem (o MPC fica "a conduzir" sem pose);
- o `tools/sim.sh` exporta `ROS_LOCALHOST_ONLY=1` e `IGN_IP=127.0.0.1`: tudo passa pelo loopback, que
  não muda. Bónus: a simulação não manda `/commands/...` para um carro real na mesma rede.
  O `setup.bash` do Humble põe `ROS_LOCALHOST_ONLY=0`, por isso o `sim.sh` força o 1 depois de o carregar
  (`CA_SIM_LOCALHOST_ONLY=0 tools/sim.sh` para desligar). Com esta rede (Wi-Fi + Tailscale no WSL) e sem
  o 1, o DDS chegou a ver só 3 dos ~10 nós da simulação;
- o `gazebo.launch.py` e o `drive` usam `IGN_IP=127.0.0.1` se não estiver definido;
- para ver os tópicos do Gazebo à mão: `IGN_IP=127.0.0.1 ign topic -l`.

### Python

- `numpy`: há um `numpy 1.26.4` instalado com pip em `~/.local` (por causa do open3d/pandas). Funciona,
  mas o `scipy 1.8` do apt avisa que quer `<1.25`. Com `numpy 2` o MPC e o GUI nem arrancam.
  Os testes correm com `PYTHONNOUSERSITE=1` (usam o numpy 1.21 do apt).
