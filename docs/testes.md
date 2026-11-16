# Testes

Testes unitários, cenários em circuito fechado (MPC/DWA + carro simulado, cinemático ou Gazebo) e o MPC sem ROS.

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
| DWA global, caminho Q | **PASS** | **PASS** | erro lateral máx. 0,06–0,07 m, pára no fim (2 de 2 em cada). Até 13 nov: instável — sem progresso, a janela dava a volta ao caminho e andava a 3 m/s: parava logo na reta ou dava mais voltas com erro até ~4 m (ver [alteracoes.md](alteracoes.md)) |
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

## Testar o MPC sem ROS (`tools/mpc_offline.py`)

Corre o `mpc_controller` verdadeiro contra o modelo bicicleta, sem ROS a correr (determinístico, ~1 s por
percurso). Quando o MPC para por falta de trajetória livre, mostra porque é que cada plano foi rejeitado.

```bash
python3 tools/mpc_offline.py "8.1,2.16"                          # obstáculos x1,y1,...
python3 tools/mpc_offline.py "2.5,0.0" --verbose                # passo a passo
python3 tools/mpc_offline.py "8.1,2.16" --latency --seed 3       # atraso do update() + perturbação aleatória
python3 tools/mpc_offline.py "8.1,2.16" --patch USE_REJOIN=1     # mudar constantes do MPC
```
