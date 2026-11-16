# Simulação no Gazebo

Como abrir e conduzir a simulação, afinar o MPC e o DWA, e como foram feitos a pista e o carro do Gazebo. Testes automáticos: [testes.md](testes.md).

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
- A simulação só fala por 127.0.0.1 (ver [Rede](ambiente_wsl.md#rede-wsl)). Se o terminal de condução disser
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
  assumem 0,335 m (ver [calibrações](alteracoes.md#calibrações-diferentes-entre-mpc-e-dwa)). Colisões simples (caixa + esferas) para a física ser estável.
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
