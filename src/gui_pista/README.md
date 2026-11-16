# gui_pista — GUI antiga (Tkinter)

Com esta GUI:
1. desenha-se ou carrega-se a imagem da pista;
2. planeia-se a trajetória (Hybrid A\*, DWA, MPC, linha de corrida);
3. publica-se no ROS (`/reference_path`, `/occupancy_grid`, `/obstacles_marker`).

```bash
~/ca_ws/tools/sim.sh path:=gui_antigo                  # com a simulação
ros2 run gui_pista gui_pista --map src/pickup_description/maps/map.yaml
```

Com um `map.yaml`, a escala e a origem passam a ser as do mapa (antes a imagem media sempre 4,8 m de
altura), e o carro da simulação aparece em tempo real.

- Alterações ao original: [docs/alteracoes.md](../../docs/alteracoes.md).
- Limitações: [docs/limitacoes.md](../../docs/limitacoes.md).
- A GUI nova é o `ca_gui`.
