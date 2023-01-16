cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_filter_ros)

add_compile_options(-std=c++17)
set (CMAKE_SHARED_LINKER_FLAGS "-static-libstdc++")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS roscpp 
  DEPENDS
)

# pointcloud_filter_ros
add_executable(project_inliers_node src/project_inliers.cpp src/project_inliers_node.cpp)
target_link_libraries(project_inliers_node
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# install(TARGETS project_inliers_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY
#   launch/
#   config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )