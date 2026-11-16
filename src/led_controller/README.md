# led_controller

Recebe em `/signal` (`std_msgs/String`) o sinal de trânsito detetado (ex.: `perigo_geral`, `parque`,
`vel_60`, `hospital`) e pisca o LED com a cor e o número de ciclos desse sinal. Para já, só escreve no
log.

Do repositório [ca_sim_ros2](https://github.com/Duarte-Diamantino/ca_sim_ros2) @ `c29202d`.

```bash
ros2 run led_controller led_controller
ros2 topic pub --once /signal std_msgs/String "data: parque"
```

Notas: [docs/terminal.md](../../docs/terminal.md) (secção LED Control).
