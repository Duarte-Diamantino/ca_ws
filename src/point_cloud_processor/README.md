# point_cloud_processor — mapa e localização (Afonso Oliveira)

Pacote do Afonso Oliveira, do repositório
[olivas-afonso/fnr_thesis](https://github.com/olivas-afonso/fnr_thesis) @ `e0570de`. Faz a localização
do carro real com a câmara ZED 2i, o VESC e o mapa da pista.

| Parte | Executáveis / launches |
|---|---|
| **Mapa da pista e localização global** (AMCL) | `global_localization.launch.py`: map_server com `maps/map.yaml`, AMCL, EKF, `global_localization`, `map_transformer`, `map_republisher`, `dynamic_tf_publisher`. Também `pose_waiter` e `amcl_path_recorder` |
| **Criar mapas** | `map_creator_pc` (mapa a partir da nuvem de pontos, publica `/custom_map`), `slam_toolbox_offline.launch.py`, `map_testing.launch.py`, `config/slam_*.yaml`. Mapas em `maps/`: o `map.png` é o `pickup_description/maps/map.pgm`, e há ainda `map1/2/3.png` |
| Sensores e odometria | `sensors_init.launch.py` (câmara, VESC, URDF, EKF), `fusion_odom_test.launch.py`, `config/ekf_config.yaml` |
| Localização relativa (linhas da pista) | `point_cloud_processor`, `kalman_rel_localization`, `frenet_motion` |
| Perceção | `object_detector`; `yolo_visualize` e `mobile_net` (sinais: precisam da ZED e dos pesos); `zed_image_saver`, `test_compression` |

Como correr no carro: [run_robot.txt](run_robot.txt), [how_to_run.txt](how_to_run.txt),
[record_bags.txt](record_bags.txt) e [docs/Global.md](../../docs/Global.md).

```bash
ros2 launch point_cloud_processor sensors_init.launch.py launch_ekf:=true launch_camera:=true use_sim_time:=false
ros2 launch point_cloud_processor global_localization.launch.py use_sim_time:=false
```

## O que mudou ao juntar ao ca_ws

- **Só o `CMakeLists.txt`** (`[ca_ws]`): o `zed_msgs`, o `point_cloud_interfaces`/`draco` e o GSL
  passaram a opcionais, para o pacote compilar também no PC. Sem eles ficam de fora o `yolo_visualize`,
  o `mobile_net` e o `test_compression`; no Jetson compila tudo como antes.
- **Os pesos da rede não vieram** (`weights/*.onnx`, 231 MB): estão no repositório original, em
  `weights/`. O `yolo_visualize` e o `mobile_net` procuram-nos em
  `/home/jetson/test_ws/src/fnr_thesis/weights/` (caminho fixo no código). O `weights/classes.txt` veio.
- **Compilar demora ~7 min** (PCL). Para a simulação não é preciso:
  `colcon build --symlink-install --packages-skip point_cloud_processor`.
