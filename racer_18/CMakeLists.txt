cmake_minimum_required(VERSION 3.8)
project(racer_18)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosgame_bridge REQUIRED)
find_package(rosgame_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

# RACER EXECUTABLE (Hybrid Navigation)
# ==========================================
add_executable(racer src/racer.cpp)
target_link_libraries(racer jsoncpp)
ament_target_dependencies(racer
   rosgame_bridge
   rosgame_msgs
   rclcpp
   std_msgs
   geometry_msgs
   sensor_msgs
)


# INSTALLATION
install(TARGETS
   racer
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   launch
   DESTINATION share/${PROJECT_NAME}
)                  
                  
ament_package()