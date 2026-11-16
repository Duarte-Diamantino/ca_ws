# Limitações conhecidas

O que ainda não está resolvido (ver também os resultados em [testes.md](testes.md)).

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

## O que ainda não está aqui

- **Localização / sensores no simulador**: o `point_cloud_processor` (Afonso Oliveira: mapa + AMCL,
  EKF, localização pelas linhas da ZED) já está no workspace e compila no PC, mas só foi usado no carro
  real (`docs/Global.md`); no Gazebo não há câmara nem lidar, a odometria é a pose verdadeira.
- Driver VESC (`vesc_msgs`, `vesc_driver`): vem do `f1tenth_system`.
