cmake_minimum_required(VERSION 3.5)
project(px4_ros2_crypto)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#################
# Setup targets #
#################

# encryption_flightdata
link_directories(/usr/local/lib)
add_executable(encryption_flightdata src/encryption_flightdata.cpp)
target_link_libraries(encryption_flightdata tss2-fapi) # tss2-esys
ament_target_dependencies(encryption_flightdata rclcpp px4_msgs)
install(TARGETS encryption_flightdata DESTINATION lib/${PROJECT_NAME})

# decryption_flightdata
link_directories(/usr/local/lib)
add_executable(decryption_flightdata src/decryption_flightdata.cpp)
target_link_libraries(decryption_flightdata tss2-fapi) # tss2-esys
ament_target_dependencies(decryption_flightdata rclcpp px4_msgs)
install(TARGETS decryption_flightdata DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
