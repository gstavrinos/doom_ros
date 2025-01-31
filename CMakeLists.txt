cmake_minimum_required(VERSION 3.8)
project(doom_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

find_program(PYTHON python3)

if(PYTHON)
  execute_process(
    COMMAND ${PYTHON} -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
  )
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()

install(DIRECTORY scripts/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING
      PATTERN "*.py"
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config/
)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)
