# Ambiente (WSL Ubuntu 22.04)

## Gazebo Fortress

Instalado com `sudo apt install ros-humble-ros-gz` (o `apt update` antes é obrigatório: sem ele o
`apt install` dá 404 nos pacotes ROS).

O `ca_gazebo` não usa o `gz_sim.launch.py` do `ros_gz_sim`: esse corre o Gazebo com `shell=True` e o
SIGINT do launch fica no `/bin/sh`, deixando o servidor órfão a correr física em fundo depois de cada
teste. O `gazebo.launch.py` arranca o `ign gazebo` diretamente.

## Rede (WSL)

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

## ~/.bashrc

A 16 nov 2026 o `~/.bashrc` tinha 3 linhas `echo "..." >> ~/.bashrc` que se acrescentavam a cada
terminal aberto (134 cópias do `source /opt/ros/humble/setup.bash`, ~47 s para abrir um terminal). Ficou
com as linhas originais uma vez cada (cópia antiga em `~/.bashrc.antes_limpeza_2026-11-16`):
`source /opt/ros/humble/setup.bash`, o `~/slam_ws` (só se existir), `ROS_LOCALHOST_ONLY=0` e
`ROS_DOMAIN_ID=42`. O `tools/sim.sh` força os valores da simulação por cima (ver Rede).

## Python

- `numpy`: há um `numpy 1.26.4` instalado com pip em `~/.local` (por causa do open3d/pandas). Funciona,
  mas o `scipy 1.8` do apt avisa que quer `<1.25`. Com `numpy 2` o MPC e o GUI nem arrancam.
  Os testes correm com `PYTHONNOUSERSITE=1` (usam o numpy 1.21 do apt).
