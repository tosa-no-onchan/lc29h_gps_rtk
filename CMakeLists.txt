cmake_minimum_required(VERSION 3.8)
project(lc29h_gps_rtk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)

# add by nishi
add_executable(${PROJECT_NAME}_node 
  src/node.cpp 
  src/gps.cpp
  src/ntrip_util.cc
  src/ntrip_client.cc)

  target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs")

target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
  
  #EXPORT export_${PROJECT_NAME}
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
)
  
# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
