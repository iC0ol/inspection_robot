cmake_minimum_required(VERSION 3.8)
project(inspection_robot_bringup)

find_package(ament_cmake REQUIRED)

# 安装运行时启动文件和参数配置。
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_controllers_config
    test/test_controllers_config.py
  )

  add_launch_test(
    test/test_diff_drive_runtime.test.py
  )
endif()

ament_package()
