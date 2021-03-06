cmake_minimum_required(VERSION 2.8.3)
project(flir_adk_ethernet)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

# If Spinnaker is already present, use the found version.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(Spinnaker QUIET)
if(NOT Spinnaker_FOUND)
  message(WARNING "libSpinnaker not found in system library path, this package will not be built")
# Download spinnaker. We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.
#   include(cmake/DownloadSpinnaker.cmake)
#   download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
else() # Spinnaker_FOUND

  message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
  message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")
endif() # NOT Spinnaker_FOUND

add_message_files(
  FILES
  MultiTimeHeader.msg
)

add_service_files(
  FILES
  GetNode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  image_transport
  cv_bridge
  nodelet
  sensor_msgs
  message_runtime
)

if(Spinnaker_FOUND)
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Spinnaker_INCLUDE_DIRS}
  )

  add_executable(${PROJECT_NAME}_node
    src/flir_adk_ethernet_node.cpp
  )

  add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Spinnaker_LIBRARIES}
  )

  add_library(BosonCameraEthernet
    src/nodelets/BaseCameraController.cpp
    src/nodelets/CameraController.cpp
    src/nodelets/SyncCameraController.cpp
    src/nodelets/TimeSynchronizer.cpp
    src/nodelets/TimeDifference.cpp
    src/flir_adk_ethernet/EthernetCamera.cpp
    src/flir_adk_ethernet/ImageEventHandler.cpp
    src/flir_adk_ethernet/ImageFormat.cpp
    src/flir_adk_ethernet/Util.cpp
    spinnaker_wrappers/CameraListWrapper.cpp
    spinnaker_wrappers/SystemWrapper.cpp
    spinnaker_wrappers/CameraWrapper.cpp
  )

  add_dependencies(BosonCameraEthernet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(BosonCameraEthernet
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Spinnaker_LIBRARIES}
  )

  set(ROSLINT_CPP_OPTS "--filter=-runtime/threadsafe_fn,-build/namespaces,-build/include_what_you_use")
  roslint_cpp()

  install(TARGETS ${PROJECT_NAME}_node BosonCameraEthernet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  #############
  ## Testing ##
  #############

  ## Add gtest based cpp test target and link libraries
  if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test
      test/test_flir_adk_ethernet.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test
      ${catkin_LIBRARIES}
      BosonCameraEthernet
    )

  endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)

endif() # Spinnaker_FOUND