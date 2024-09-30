cmake_minimum_required(VERSION 3.20.0)
project(tb_control)

# Compilation flags
add_compile_options(-Wall)

# find dependencies
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME}
    SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# install Python execu

# Add executable
install(PROGRAMS 
	scripts/tb_openLoop_const_vel.py 
	scripts/tb_openLoop_with_acc.py
	scripts/tb_openLoop_with_decc.py 
	DESTINATION lib/${PROJECT_NAME})
ament_package()
