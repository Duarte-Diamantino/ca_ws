# dwa_follower — DWA

`dwa_global_follower` é o nó usado: um DWA que segue o `/reference_path` e contorna os obstáculos,
desviando o caminho à volta deles (`use_ref_shift`). O `dwa_follower_node` e o `dwa_spline_follower`
são os nós anteriores do TESE.

- **Subscreve:** `/reference_path`, `/odometry/filtered`, `/obstacles_marker` (raio = `scale.x/2` com
  `use_marker_size`), `/joy`.
- **Publica:** `/commands/motor/speed`, `/commands/servo/position`, `/predicted_trajectories` e
  `/reference_path_markers` (o caminho, já desviado).
- **Parâmetros:** o `config/dwa.yaml` tem os do simulador (v_max 1,2 m/s...). No carro real, sem o
  ficheiro, ficam os valores do código. Todos mudam com o nó a correr.

```bash
ros2 run dwa_follower dwa_global_follower --ros-args --params-file src/dwa_follower/config/dwa.yaml
ros2 param set /dwa_global_follower v_max 1.5
```

- Como afinar: [docs/simulacao.md](../../docs/simulacao.md#afinar-o-dwa).
- Alterações ao original: [docs/alteracoes.md](../../docs/alteracoes.md).
