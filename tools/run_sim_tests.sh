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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_TESTS:-77}" ROS_LOCALHOST_ONLY=1
# Usa o numpy do apt (1.21), o que o scipy 1.8 do apt espera. Com o numpy 1.26 em ~/.local
# tambem funciona (o scipy so avisa); com numpy 2 em ~/.local o MPC e o GUI nem arrancam.
export PYTHONNOUSERSITE=1
LOGDIR="${LOGDIR:-/tmp/ca_sim_tests}"; mkdir -p "$LOGDIR"
OBS_TERMINAL_MD='obstacles:=[7.5,2.0,0.5,1.8,0.5,2.5,0.7,2.0] min_obstacle_clearance:=0.3'

# nome | esperado | argumentos do launch
SCENARIOS=(
  "mpc_base|PASS|sim:=kinematic controller:=mpc"
  "dwa_base|INSTAVEL|sim:=kinematic controller:=dwa"
  "mpc_obstaculos_terminal_md|FAIL|sim:=kinematic controller:=mpc $OBS_TERMINAL_MD"
  "gz_mpc_base|PASS|sim:=gazebo controller:=mpc"
  "gz_dwa_base|INSTAVEL|sim:=gazebo controller:=dwa"
  "gz_mpc_obstaculos_terminal_md|FAIL|sim:=gazebo controller:=mpc $OBS_TERMINAL_MD"
)
# Esperado: PASS, FAIL (bug conhecido) ou INSTAVEL (qualquer resultado; nao conta para o exit code).
#  *dwa_base (INSTAVEL): o caminho Q acaba em (1.1, 0), em cima da reta inicial. O DWA da o
#    percurso por terminado so pela distancia ao ultimo ponto (sem olhar ao progresso), por isso
#    as vezes para logo na reta e outras falha a janela de 0.3 m e da mais uma volta (erro ate ~3.5 m).
#  *mpc_obstaculos_terminal_md: o MPC para ~1.8 m antes do obstaculo
#    ("STOP: sem trajetoria livre apos obstaculo") e nunca o contorna.

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
if pgrep -f '^ign gazebo' >/dev/null; then
  echo "AVISO: ficou um servidor do Gazebo a correr: $(pgrep -af '^ign gazebo')"; bad=1
fi
echo "logs em $LOGDIR"
exit $bad
