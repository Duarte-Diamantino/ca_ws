# Alterações ao código original

O que mudou no código que veio dos repositórios originais (ver a origem de cada pacote no [README](../README.md#origem)). No código, todas as alterações estão marcadas com `[ca_ws]`.

## Alterações

- `point_cloud_processor` (Afonso Oliveira, `fnr_thesis`): só o `CMakeLists.txt` — o `zed_msgs`, o
  `point_cloud_interfaces`/`draco` e o GSL passaram a opcionais, para o pacote compilar também no PC
  (WSL). Sem eles não são compilados o `yolo_visualize`, o `mobile_net` (ZED) e o `test_compression`;
  no Jetson fica tudo igual. Os pesos da rede (`weights/*.onnx`, 231 MB) não vieram (ver o README do pacote).
- Ctrl+C na simulação sem tracebacks: os nós do `ca_sim` e o `q_path_publisher` apanham o
  `ExternalShutdownException` (e erros que só acontecem com o ROS já a fechar), que o launch
  mostrava como "process has died".
- `pickup_description`: `map_afonso.yaml` (o `map.pgm` que já cá estava é o mapa do Afonso) e
  `map_only.launch.py map_file:=` para escolher o mapa (ver `src/pickup_description/maps/README.md`).
- `pickup_description`: `map.yaml` à escala real (lado menor 7 m) com o novo `fnr_ca.png` (2× mais
  resolução), e `q_path_publisher` com o parâmetro `scale` (defeito 1,0793, igual ao mapa) — ver
  [A pista à escala real](simulacao.md#a-pista-à-escala-real).

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
  constantes de afinação passam a parâmetros ROS (ver [Afinar o MPC](simulacao.md#afinar-o-mpc)), com os valores de origem em
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
  - Diagnóstico e escolha dos valores com `tools/mpc_offline.py` (ver [testes.md](testes.md#testar-o-mpc-sem-ros-toolsmpc_offlinepy)).
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

## Calibrações diferentes entre MPC e DWA

Os dois controladores assumem calibrações diferentes para o **mesmo** carro — só uma pode estar certa:

| | MPC | DWA |
|---|---|---|
| velocidade → comando | 7000 RPM por m/s (mín. 3000) | 7690 por m/s (23070 a 3 m/s) |
| centro do servo | 0,5751 | 0,5304 |
| servo no máximo | 0,8760 | 0,94299 |
| ângulo máximo | 22,5° | 30° |
| distância entre eixos | 0,335 m | 0,335 m (o GUI e o CAD do carro dizem 0,324 m) |

Os simuladores usam a calibração de cada controlador (`PROFILES` em `src/ca_sim/ca_sim/vesc.py`).
