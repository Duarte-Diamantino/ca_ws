# extras — não é compilado

Código que veio dos repositórios originais mas não é um pacote do workspace (`COLCON_IGNORE`).

| Pasta | O que é | Origem |
|---|---|---|
| [`CA_SIM/`](CA_SIM) | Versões antigas dos controladores e da GUI, de antes do TESE: `MPC_python_final`, `MPC_python_vesc_odom`, `MPC_smooth.py`, `DWA_frente.cpp`, `dwa_frente_tras`, `dwa_follower_circulo.cpp`, `dwa_follower_espiral.cpp`, `GUI.py`, `spline_para_global`, `spline_para_teste.py`. Também o `map.pgm`, o `pista_rvizz.png` e os STL do carro (o `carro_com_rodas.stl` deu as medidas do carro do Gazebo) | [CA_SIM](https://github.com/Duarte-Diamantino/CA_SIM) @ `ecbc295` |
| [`semaforo_CA_FNR/`](semaforo_CA_FNR) | Semáforo da prova: um Raspberry Pi mostra em ecrã inteiro as imagens dos sinais (frente, esquerda, direita, stop, parque, fim). Como usar: o README da pasta | [semaforo_CA_FNR](https://github.com/Duarte-Diamantino/semaforo_CA_FNR) @ `95a5bab` |

As notas do `CA_SIM` (`Global.md`, `terminal.md`, `rviz_readme.md` e o README, agora
`gazebo_wsl_setup.md`) estão em [`docs/`](../docs).
