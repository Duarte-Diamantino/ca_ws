# Mapas da pista FNR (`pickup_description/maps`)

| Ficheiro | O que é | Origem |
|---|---|---|
| `map.yaml` + `fnr_ca.png` | **Mapa usado na simulação** (Gazebo, RViz, GUI): o desenho da pista à escala real (lado menor 7 m), 0,00653 m/px | gerado por `ca_gazebo/scripts/gerar_mundo.py` a partir do `map_200.pgm` |
| `map_afonso.yaml` + `map.pgm` | **Mapa do Afonso Oliveira** para a localização global (AMCL) no carro real, 0,026 m/px, rodado −90° | `map.pgm` = `maps/map.png` do `point_cloud_processor` ([olivas-afonso/fnr_thesis](https://github.com/olivas-afonso/fnr_thesis)) |
| `map_200.pgm` | Desenho da pista (GIMP), 778×536 px. Era o `map.yaml` original do TESE, a 0,0121 m/px (lado menor 6,49 m, pequeno demais) | TESE |
| `map_half.pgm`, `map_scaled.pgm` | Versões reduzidas do desenho | TESE |

O `map_only.launch.py` publica um deles no `/map` (`map_file:=` escolhe o ficheiro desta pasta, por defeito
`map.yaml`):

```bash
ros2 launch pickup_description map_only.launch.py                        # pista da simulacao
ros2 launch pickup_description map_only.launch.py map_file:=map_afonso.yaml   # mapa do Afonso
```

Os outros mapas do Afonso (`map1/2/3.png`) e a localização com o AMCL estão no pacote
`point_cloud_processor` (ver o README dele).
