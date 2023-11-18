cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(autoaim_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set Release during the competition
set(CMAKE_BUILD_TYPE Debug)


find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(
  autoaim_bridge_parameters # cmake target name for the parameter library
  src/autoaim_bridge_parameters.yaml # path to input yaml file
)


ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  DIRECTORY src
  )

target_compile_options(${PROJECT_NAME} PUBLIC
  -ffast-math -march=native
)

target_link_libraries(
  ${PROJECT_NAME}
  autoaim_bridge_parameters
  OpenMP::OpenMP_CXX
  )
  
target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::AutoaimBridge
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)