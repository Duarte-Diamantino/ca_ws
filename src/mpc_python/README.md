# mpc_python — MPC

`mpc_controller`: MPC com modelo bicicleta, otimizado com o L-BFGS-B do scipy. Segue o
`/reference_path` e contorna os `/obstacles`, desviando a referência à volta de cada obstáculo.

- **Subscreve:** `/reference_path`, `/obstacles` (`PoseArray`; `z` = raio com
  `OBSTACLE_SIZE_FROM_Z`), `/odometry/filtered`, `/joy`.
- **Publica:** `/commands/motor/speed`, `/commands/servo/position` e `/predicted_trajectories` (o plano).
- **Parâmetros:** as constantes que interessa afinar são parâmetros ROS (lista `TUNABLE`). O
  `config/mpc.yaml` tem os valores do simulador; o `~/ca_ws/mpc_params.yaml` (se existir) fica por cima.

```bash
ros2 run mpc_python mpc_controller --ros-args --params-file src/mpc_python/config/mpc.yaml
ros2 param set /mpc_controller V_MAX 1.5
```

- Como afinar: [docs/simulacao.md](../../docs/simulacao.md#afinar-o-mpc).
- Sem ROS: `tools/mpc_offline.py`.
- Testes: `test/test_mpc_controller.py`.
- Alterações ao original: [docs/alteracoes.md](../../docs/alteracoes.md).
