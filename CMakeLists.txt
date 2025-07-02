cmake_minimum_required(VERSION 3.8)
project(arm2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/interactive_teach_pendant.py
  scripts/cartesian_path_planner.py
  scripts/workspace_analyzer.py
  scripts/test_system.py
  scripts/usage_guide.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
