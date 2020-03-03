cmake_minimum_required(VERSION 2.8.3)
project(image_warper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  project11
  std_msgs
  image_transport
  tf2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package( OpenCV REQUIRED )
#find_path(EIGEN_INCLUDES Eigen/Core /usr/include/eigen3)

#add_action_files(DIRECTORY action FILES manda_coverage.action)
#generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_helm
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
#  src/lib_geometry/
#  src/lib_mbutil/
  src/
  ${catkin_INCLUDE_DIRS}
#  ${EIGEN_INCLUDES}
)

set(NODE_SOURCES 
    src/cameraSetup.cpp
    src/Executor.cpp
)

#adding 2 executables lines bcos of 2 mains
add_executable(${PROJECT_NAME}_node_test_publisher src/test_Publisher.cpp)
add_executable(${PROJECT_NAME}_node_test_subscriber src/test_Subscriber.cpp)
#add_executable(${PROJECT_NAME}_node_test_openCV_1 src/test_openCV_1.cpp)
add_executable(${PROJECT_NAME}_node_imageStabilization ${NODE_SOURCES})


#1 dependency per executable - may not be reqd
add_dependencies(${PROJECT_NAME}_node_test_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node_test_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_node_test_openCV_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node_imageStabilization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#1 TLL per executable
target_link_libraries(${PROJECT_NAME}_node_test_publisher
    ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node_test_subscriber
    ${catkin_LIBRARIES}
)
#target_link_libraries(${PROJECT_NAME}_node_test_openCV_1
#    ${catkin_LIBRARIES}
#)
#note this
target_link_libraries(${PROJECT_NAME}_node_imageStabilization
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

#install(TARGETS ${PROJECT_NAME}_node_test_publisher
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



