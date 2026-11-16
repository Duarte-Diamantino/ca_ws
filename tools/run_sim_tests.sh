#!/usr/bin/env bash
# Corre os cenarios em circuito fechado e compara com o resultado esperado.
#   ./tools/run_sim_tests.sh               # todos
#   ./tools/run_sim_tests.sh mpc_base      # so um
#   ./tools/run_sim_tests.sh gz_           # todos os que comecam por "gz_"
# Cenarios gz_* usam o Gazebo Fortress (ca_gazebo); sao SALTADOS se o "ign" nao existir.
# Codigo de saida != 0 se algum cenario nao der o resultado esperado
# (inclui um "FAIL conhecido" que passou a PASS -> atualiza a tabela).
WS="$(cd "$(dirname "$0")/.." && pwd)"
# (os setup.bash do ROS nao aguentam "set -u", por isso so depois)
source /opt/ros/humble/setup.bash
# Gazebo extraido sem sudo (temporario, ver README) - so se existir e o ign nao estiver instalado
if ! command -v ign >/dev/null && [ -f "$HOME/.cache/ca_gz/setup.bash" ]; then
  source "$HOME/.cache/ca_gz/setup.bash"
fi
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_TESTS:-77}" ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1
# Usa o numpy do apt (1.21), o que o scipy 1.8 do apt espera. Com o numpy 1.26 em ~/.local
# tambem funciona (o scipy so avisa); com numpy 2 em ~/.local o MPC e o GUI nem arrancam.
export PYTHONNOUSERSITE=1
LOGDIR="${LOGDIR:-/tmp/ca_sim_tests}"; mkdir -p "$LOGDIR"
# obstaculos do docs/terminal.md, (7.5,2.0) (0.5,1.8) (0.5,2.5) (0.7,2.0), x1.0793 como a pista e o
# caminho Q (mapa a escala real: lado menor 7 m, ver README)
OBS_TERMINAL_MD='obstacles:=[8.095,2.159,0.540,1.943,0.540,2.698,0.756,2.159] min_obstacle_clearance:=0.3'
# obstaculo EM CIMA do caminho: o MPC desvia a referencia ~0.6 m para o lado, por isso o erro lateral
# (medido ao caminho original) pode chegar a ~0.75 m
OBS_EM_CIMA='min_obstacle_clearance:=0.5 max_cte:=0.85'
# o DWA desvia o caminho para (raio 0.15 + meia largura 0.10 + margem 0.10) + 0.10 = 0.45 m do centro;
# passa a ~0.42 m (0.35 = a distancia minima dele)
OBS_EM_CIMA_DWA='min_obstacle_clearance:=0.35 max_cte:=0.85'

# nome | esperado | argumentos do launch
SCENARIOS=(
  "mpc_base|PASS|sim:=kinematic controller:=mpc"
  "dwa_base|PASS|sim:=kinematic controller:=dwa"
  "dwa_obstaculo_na_reta|PASS|sim:=kinematic controller:=dwa obstacles:=[2.698,0.0] $OBS_EM_CIMA_DWA"
  "dwa_obstaculo_no_laco|PASS|sim:=kinematic controller:=dwa obstacles:=[7.62,2.58] $OBS_EM_CIMA_DWA"
  "dwa_dois_obstaculos|PASS|sim:=kinematic controller:=dwa obstacles:=[2.0,0.0,4.5,0.05] $OBS_EM_CIMA_DWA"
  "dwa_obstaculos_terminal_md|PASS|sim:=kinematic controller:=dwa $OBS_TERMINAL_MD"
  "mpc_obstaculos_terminal_md|PASS|sim:=kinematic controller:=mpc $OBS_TERMINAL_MD"
  "mpc_obstaculo_na_reta|PASS|sim:=kinematic controller:=mpc obstacles:=[2.698,0.0] $OBS_EM_CIMA"
  "mpc_obstaculo_no_laco|PASS|sim:=kinematic controller:=mpc obstacles:=[7.62,2.58] $OBS_EM_CIMA"
  "gz_mpc_base|PASS|sim:=gazebo controller:=mpc"
  "gz_dwa_base|PASS|sim:=gazebo controller:=dwa"
  "gz_dwa_obstaculo_na_reta|PASS|sim:=gazebo controller:=dwa obstacles:=[2.698,0.0] $OBS_EM_CIMA_DWA"
  "gz_mpc_obstaculos_terminal_md|PASS|sim:=gazebo controller:=mpc $OBS_TERMINAL_MD"
  "gz_mpc_obstaculo_na_reta|PASS|sim:=gazebo controller:=mpc obstacles:=[2.698,0.0] $OBS_EM_CIMA"
)
# Esperado: PASS, FAIL (bug conhecido) ou INSTAVEL (qualquer resultado; nao conta para o exit code).
#  *dwa_*: ate 13 nov 2026 instavel (o DWA so olhava a distancia ao ultimo ponto, a janela dava a volta
#    ao caminho e andava a 3 m/s: parava logo na reta ou dava mais voltas com erro ate ~4 m). Agora
#    com progresso, fim pelo progresso e dwa_follower/config/dwa.yaml (v_max 1.2, ver README).
#  *mpc_obstaculos_terminal_md: so o (8.10, 2.16) interessa (0.43 m por fora da curva grande); o MPC
#    contorna-o por dentro a ~0.7 m. Antes do custo suave parava 2 m antes (ver README).
#  *obstaculo_na_reta / *obstaculo_no_laco: obstaculo EM CIMA do caminho. Ate 12 nov 2026 o MPC parava
#    ~1.5 m antes ("STOP: sem trajetoria livre"); agora desvia a referencia e contorna (ver README).
#    O DWA faz o mesmo desde 16 nov 2026 (use_ref_shift); antes so via o obstaculo no horizonte de 1 s.

only="${1:-}"; bad=0
have_gz=1; command -v ign >/dev/null || have_gz=0
printf '%-30s %-9s %-9s %s\n' CENARIO ESPERADO OBTIDO RESUMO
for s in "${SCENARIOS[@]}"; do
  IFS='|' read -r name expected args <<<"$s"
  [ -n "$only" ] && [[ "$name" != "$only"* ]] && continue
  if [[ "$name" == gz_* ]] && [ $have_gz = 0 ]; then
    printf '%-30s %-9s %-9s %s\n' "$name" "$expected" SALTADO "Gazebo (ign) nao instalado"
    continue
  fi
  log="$LOGDIR/$name.log"
  export IGN_PARTITION="ca_tests_$name"
  # shellcheck disable=SC2086
  timeout -s INT 180 ros2 launch ca_sim sim_closed_loop.launch.py $args timeout:=60 >"$log" 2>&1
  got=$(grep -oE 'RESULTADO: (PASS|FAIL)' "$log" | tail -1 | awk '{print $2}')
  got=${got:-ERRO}
  summary=$(grep -oE 'cte_max=[0-9.]+m|progresso=[0-9.]+%' "$log" | sort -u | tr '\n' ' ')
  printf '%-30s %-9s %-9s %s\n' "$name" "$expected" "$got" "$summary"
  [ "$expected" != "INSTAVEL" ] && [ "$got" != "$expected" ] && bad=1
done
# Gazebo orfao (o pai morreu: passou para o pid 1) = um teste nao fechou o Gazebo. Uma simulacao
# aberta noutro terminal nao conta.
orphans=$(for p in $(pgrep -f '^ign gazebo'); do [ "$(ps -o ppid= -p "$p" | tr -d ' ')" = 1 ] && echo -n "$p "; done)
if [ -n "$orphans" ]; then
  echo "AVISO: ficou um Gazebo orfao a correr (pids $orphans)"; bad=1
fi
echo "logs em $LOGDIR"
exit $bad
