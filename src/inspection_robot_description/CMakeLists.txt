cmake_minimum_required(VERSION 3.8)
project(inspection_robot_description)

find_package(ament_cmake REQUIRED)

# 这个 package 没有需要编译的 C++ 节点。
# 构建时只需把 URDF 和 launch 文件安装到 share 目录。
install(
  DIRECTORY
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_robot_description
    test/test_robot_description.py
  )
endif()

ament_package()
