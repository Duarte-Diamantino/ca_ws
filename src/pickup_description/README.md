# pickup_description — mapas, caminhos de teste e URDF simples

- **Mapas** da pista FNR em `maps/`: o da simulação e o do Afonso Oliveira (ver
  [maps/README.md](maps/README.md)). O
  `ros2 launch pickup_description map_only.launch.py [map_file:=map_afonso.yaml]` publica um deles no `/map`.
- **Caminhos de teste** no `/reference_path`:
  - `q_path_publisher`: o caminho Q dos testes (`scale` 1,0793 = pista à escala real);
  - `marker_publisher`: spline;
  - `rounded_rectangle_marker_publisher`: retângulo.

  Launches: `q_path.launch.py`, `spline.launch.py`, `launch_rounded_rectangle.launch.py`.
- **URDF simples** e `mov_robot_model` (move o modelo no RViz a partir dos comandos):
  `display.launch.py`, `robot.launch.py`, `set_pose.launch.py`, `all.launch.py`.

Notas antigas sobre o RViz: [docs/rviz_readme.md](../../docs/rviz_readme.md).
