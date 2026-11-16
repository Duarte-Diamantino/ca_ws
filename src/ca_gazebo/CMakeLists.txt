cmake_minimum_required(VERSION 3.8)
project(ca_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME})
# scripts/gerar_mundo.py gera worlds/pista.sdf, worlds/media/pista.png, meshes/cone*.stl e o
# mapa do pickup_description (corre-se a mao, o resultado esta no git)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  # o URDF (xacro) tem de gerar um modelo valido e o mundo tem de ser XML valido
  ament_add_pytest_test(test_model test/test_model.py
    ENV CA_GAZEBO_SRC=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
