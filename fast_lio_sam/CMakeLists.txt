cmake_minimum_required(VERSION 3.10)
project(fast_lio_sam)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming and for Eigen (see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM QUIET)


if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  
  message_filters
  
  tf2
  tf2_eigen
  tf2_ros

  pcl_conversions
  pcl_msgs

  GTSAM
  Eigen3
  yaml-cpp
)

set(common_libraries
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBS}
  yaml-cpp::yaml-cpp 
  gtsam
)

add_library(${PROJECT_NAME}_modules SHARED src/loop_closure.cpp src/fast_lio_sam_2.cpp)
ament_target_dependencies(${PROJECT_NAME}_modules ${dependencies})
target_link_libraries(${PROJECT_NAME}_modules ${common_libraries})

add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_modules)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_modules
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME} 
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  
  ament_lint_auto_find_test_dependencies()
endif(BUILD_TESTING)

ament_export_dependencies(rosidl_default_runtime rclcpp) 
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_modules)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_modules)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_modules
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME} 
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  
  ament_lint_auto_find_test_dependencies()
endif(BUILD_TESTING)

ament_export_dependencies(rosidl_default_runtime rclcpp) 
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_modules)

ament_package()
