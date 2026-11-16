#!/usr/bin/env bash
# Atalho para os terminais da simulacao no Gazebo (carrega o ROS e o workspace):
#   ~/ca_ws/tools/sim.sh                 simulacao: Gazebo + carro + pista + RViz
#   ~/ca_ws/tools/sim.sh path:=gui       ... com o caminho desenhado no GUI_pista
#   ~/ca_ws/tools/sim.sh obstacles:="[8.1, 2.16]"
#   ~/ca_ws/tools/sim.sh mpc             terminal do MPC     (espaco = iniciar/parar)
#   ~/ca_ws/tools/sim.sh dwa             terminal do DWA     (espaco = iniciar/parar)
#   ~/ca_ws/tools/sim.sh remote          terminal do comando (W/S, A/D, espaco = travar)
# Em todos os terminais de conducao: r = carro ao inicio do caminho, q = sair.
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
# numpy do apt (o scipy 1.8 do apt quer numpy < 1.25; ver README)
export PYTHONNOUSERSITE=1
# A simulacao so fala por 127.0.0.1 (ROS e Gazebo): nao depende da rede do Windows (se o Wi-Fi ou
# a VPN mudam com a simulacao aberta, a descoberta parte-se e o carro deixa de receber comandos) e
# nao manda /commands/... para um carro real que esteja na mesma rede.
# Todos os terminais da simulacao tem de usar os mesmos valores (usa este script em todos).
# O setup.bash do Humble poe ROS_LOCALHOST_ONLY=0, por isso aqui e forcado (CA_SIM_LOCALHOST_ONLY=0
# para desligar).
export ROS_LOCALHOST_ONLY="${CA_SIM_LOCALHOST_ONLY:-1}"
export IGN_IP="${IGN_IP:-127.0.0.1}"
# Dominio ROS fixo para todos os terminais da simulacao: um terminal do Ubuntu le o ~/.bashrc
# (ROS_DOMAIN_ID=42) e um aberto a partir do Windows (wsl ... sim.sh) nao (dominio 0). Com
# dominios diferentes nao se veem: o MPC ficava "a conduzir" sem pose. 42 = o do ~/.bashrc, para
# os comandos ros2 a mao (com ROS_LOCALHOST_ONLY=1) verem a simulacao. Outro: CA_SIM_DOMAIN_ID=N.
export ROS_DOMAIN_ID="${CA_SIM_DOMAIN_ID:-42}"
case "${1:-}" in
  mpc|dwa|remote) exec ros2 run ca_sim drive "$@" ;;
esac

# --- simulacao (Gazebo + RViz + GUI) ---
# Launches da simulacao abertos com o mesmo ROS_DOMAIN_ID e IGN_PARTITION que nos.
same_sim_launch() {
  local p env
  for p in $(pgrep -f 'ros2 launch ca_sim gazebo_sim.launch.py'); do
    env=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null) || continue
    [ "$(sed -n 's/^IGN_PARTITION=//p' <<<"$env")" = "${IGN_PARTITION:-}" ] || continue
    [ "$(sed -n 's/^ROS_DOMAIN_ID=//p' <<<"$env")" = "$ROS_DOMAIN_ID" ] && echo "$p"
  done
}
# Gazebos orfaos: "ign gazebo ..." cujo pai ja morreu (passou para o pid 1). Fechar o separador do
# terminal com o X, ou um launch que nao acaba bem, deixa o "ign gazebo server" a correr sozinho; com
# dois servidores a GUI punha/tirava obstaculos num e via-se o outro, e o /clock e a odometria vinham
# dos dois. (O "ign gazebo" reescreve o titulo do processo e estraga o /proc/N/environ, por isso nao da
# para ver a IGN_PARTITION deles: conta qualquer Gazebo orfao.)
gz_orphans() {
  local p
  for p in $(pgrep -f '^ign gazebo'); do
    [ "$(ps -o ppid= -p "$p" | tr -d ' ')" = 1 ] && echo "$p"
  done
}
close_orphan_gazebo() {
  local left
  left=$(gz_orphans | tr '\n' ' ')
  [ -n "$left" ] || return 0
  echo "AVISO: $1 (pids $left) - a fechar."
  kill -INT $left 2>/dev/null
  for _ in 1 2 3 4 5 6; do sleep 0.5; left=$(gz_orphans | tr '\n' ' '); [ -z "$left" ] && return 0; done
  kill -KILL $left 2>/dev/null
}
other=$(same_sim_launch | tr '\n' ' ')
if [ -n "$other" ]; then
  echo "Ja ha uma simulacao aberta (ros2 launch, pid $other). Fecha-a primeiro (Ctrl+C nesse terminal)"
  echo "ou: kill -INT $other"
  exit 1
fi
close_orphan_gazebo "ficou aberto um Gazebo de uma simulacao anterior"
trap ':' INT   # Ctrl+C: o launch fecha tudo e depois este script ainda limpa o que sobrar
trap 'sleep 1; close_orphan_gazebo "o Gazebo nao fechou com o launch"; exit 129' HUP
ros2 launch ca_sim gazebo_sim.launch.py "$@"
rc=$?
sleep 1
close_orphan_gazebo "o Gazebo nao fechou com o launch"
exit $rc
