cmake_minimum_required(VERSION 3.22)
project(gz_led_plugin)

# Vendored from https://github.com/jasmeet0915/gz_sim_led_plugin with a local
# patch (see src/LedPlugin.cpp) so identical model instances don't collide on
# scoped visual/light names. Built for Gazebo Harmonic (gz-sim8), matching the
# versions the existing dave_gz_world_plugins package uses in this workspace.

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/LedPlugin.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-msgs10::gz-msgs10
    gz-math7::gz-math7
    gz-transport13::gz-transport13
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Put libgz_led_plugin.so on GZ_SIM_SYSTEM_PLUGIN_PATH when the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/gz_led_plugin.sh.in")

ament_package()
