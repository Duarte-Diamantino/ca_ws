# carro_com_rodas_mov_urdf

URDF do carro feito a partir do CAD (`meshes/base_link.stl`, `meshes/roda_urfd_v3.stl`, em mm), do
repositório [ca_sim_ros2](https://github.com/Duarte-Diamantino/ca_sim_ros2) @ `c29202d`.

Ficou como estava: as meshes não têm escala no URDF (estão em mm) e as rodas ficam a ±0,5 m. O carro
da simulação (`ca_gazebo/urdf/ca_car.urdf.xacro`) usa as mesmas meshes com `scale 0.001` e as medidas
do carro montado.
