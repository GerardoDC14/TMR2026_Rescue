cmake_minimum_required(VERSION 3.10)
project(jaguar_robot_full_description)

find_package(ament_cmake REQUIRED)

# Instalar directorios
install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# Instalar package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Crear archivo de recurso requerido por ament
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
