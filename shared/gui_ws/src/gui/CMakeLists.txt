cmake_minimum_required(VERSION 3.10)
project(gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
find_library(PULSE_SIMPLE_LIB pulse-simple REQUIRED)
find_library(PULSE_LIB pulse REQUIRED)
find_library(OPUS_LIB opus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(assimp REQUIRED)

find_library(ONNXRUNTIME_LIB onnxruntime HINTS /usr/local/lib REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS /usr/local/include/onnxruntime REQUIRED)

add_executable(gui
  src/gui.cpp
  src/main_window.cpp
  src/video_widget.cpp
  src/video_panel.cpp
  src/source_manager.cpp
  src/camera_hub.cpp
  src/dashboard_panel.cpp
  src/odometry_panel.cpp
  src/speech_processor.cpp
  src/urdf_viewer.cpp
  src/filters.cpp
  src/app_settings.cpp
  src/settings_dialog.cpp
  src/keybind_dialog.cpp
  src/ppm_calib_dialog.cpp
  # Headers listed for AUTOMOC to process Q_OBJECT classes
  include/gui/main_window.hpp
  include/gui/video_widget.hpp
  include/gui/video_panel.hpp
  include/gui/source_manager.hpp
  include/gui/dashboard_panel.hpp
  include/gui/digital_twin_panel.hpp
  include/gui/speech_processor.hpp
  include/gui/urdf_viewer.hpp
  include/gui/settings_dialog.hpp
  include/gui/odometry_panel.hpp
  include/gui/keybind_dialog.hpp
  include/gui/ppm_calib_dialog.hpp
)

target_include_directories(gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ASSIMP_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_compile_features(gui PUBLIC cxx_std_17)

target_link_libraries(gui
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  ${PULSE_SIMPLE_LIB}
  ${PULSE_LIB}
  ${OPUS_LIB}
  ${OpenCV_LIBS}
  ${ASSIMP_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  vosk
  zbar
)

ament_target_dependencies(gui
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  urdf
  ament_index_cpp
)

install(TARGETS gui
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/gst_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY assets launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
