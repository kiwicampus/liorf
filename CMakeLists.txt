cmake_minimum_required(VERSION 3.5)
project(liorf_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/rosidl_get_typesupport_target.cmake)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(yaml-cpp REQUIRED)

set(ros2_lib "rclcpp;rclpy;std_msgs;std_srvs;sensor_msgs;geometry_msgs;nav_msgs;cv_bridge;pcl_msgs;pcl_conversions;visualization_msgs;tf2;tf2_ros;tf2_eigen;tf2_sensor_msgs;tf2_geometry_msgs")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# include directories
include_directories(
  lib
	include
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

# link directories
link_directories(
  lib
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

###########
## Build ##
###########
# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# FactorGraphLoader (session load/join) - ROS-free, buildable/testable in isolation
add_library(${PROJECT_NAME}_factor_graph_loader src/FactorGraphLoader.cpp)
target_include_directories(${PROJECT_NAME}_factor_graph_loader PUBLIC include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_factor_graph_loader ${PCL_LIBRARIES} gtsam ${YAML_CPP_LIBRARIES} Boost::timer)

# # Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp src/dumpGraph.cpp ${common_lib} include/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_mapOptmization ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${PROJECT_NAME}_factor_graph_loader Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam ${YAML_CPP_LIBRARIES})

# # IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# # Wheel Odometry Preintegration
add_executable(${PROJECT_NAME}_wheelOdomPreintegration src/wheelOdomPreintegration.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_wheelOdomPreintegration ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_wheelOdomPreintegration Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# # Load/visualize a saved liorf mapping session (debug tool)
add_executable(${PROJECT_NAME}_load_liorf_session src/load_liorf_session.cpp)
ament_target_dependencies(${PROJECT_NAME}_load_liorf_session ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_load_liorf_session ${PROJECT_NAME}_factor_graph_loader "${cpp_typesupport_target}" ${PCL_LIBRARIES} gtsam ${YAML_CPP_LIBRARIES})

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS
	${PROJECT_NAME}_imageProjection
	${PROJECT_NAME}_imuPreintegration
	${PROJECT_NAME}_mapOptmization
	${PROJECT_NAME}_wheelOdomPreintegration
	${PROJECT_NAME}_load_liorf_session
  DESTINATION lib/${PROJECT_NAME}
)

# Shared libraries must install to lib/ (not lib/<pkg>/) so the ament runtime
# hook adds this directory to LD_LIBRARY_PATH for the executables above.
install(
  TARGETS
	${PROJECT_NAME}_factor_graph_loader
  DESTINATION lib
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

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

ament_export_include_directories(include)

ament_package()
