# ca_sim — carro simulado e testes

Faz o carro andar sem o carro: um modelo bicicleta em Python ou o Gazebo (`ca_gazebo`), com os mesmos
tópicos do carro real (`/commands/motor/speed`, `/commands/servo/position` → `/odometry/filtered`).

| Nó | |
|---|---|
| `kinematic_sim` | modelo bicicleta (rápido, sem física), com a calibração do MPC ou do DWA |
| `vesc_to_gz` | comandos do VESC → velocidade das rodas e ângulos de Ackermann no Gazebo; publica `/joint_states` |
| `gz_spawner` | põe o carro no Gazebo no início do caminho (e muda-o quando chega um caminho novo, se estiver parado) |
| `drive` | terminais de condução: `mpc` e `dwa` (espaço inicia/para), `remote` (teclado como um comando de RC) |
| `tracking_monitor` | mede o erro ao caminho e a distância aos obstáculos e dá PASS/FAIL |
| `map_relay` | republica o `/map` em `/map_rviz` de 2 em 2 s (o Reset do RViz apagava o mapa) |

| Launch | |
|---|---|
| `gazebo_sim.launch.py` | simulação interativa: Gazebo + RViz + mapa + caminho (`path:=q`, `gui`, `gui_antigo` ou `none`). Usar com `tools/sim.sh` |
| `sim_closed_loop.launch.py` | teste em circuito fechado (`controller:=mpc` ou `dwa`, `sim:=kinematic` ou `gazebo`), termina com PASS/FAIL. Usado pelo `tools/run_sim_tests.sh` |

O `ca_sim/vesc.py` tem as calibrações (`PROFILES`) e as medidas do carro (`CAR`).

Mais: [docs/simulacao.md](../../docs/simulacao.md) e [docs/testes.md](../../docs/testes.md).
