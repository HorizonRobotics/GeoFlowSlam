cmake_minimum_required(VERSION 3.5)
project(orbslam3)
# set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

SET(CMAKE_BUILD_TYPE Release)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
option(ENABLE_VIEWER "enable render viewer" ON)
MESSAGE("ENABLE_VIEWER: ${ENABLE_VIEWER}")
option(ENABLE_ASYNC "enable async" ON)
MESSAGE("ENBLE_ASYNC: ${ENBLE_ASYNC}")
option(ENABLE_OMP "enable omp" ON)
MESSAGE("ENBLE_OMP: ${ENBLE_OMP}")

option(ENABLE_ROS2 "enable ros2" ON)
MESSAGE("ENBLE_ROS2: ${ENBLE_ROS2}")
if (ENABLE_VIEWER)
    add_definitions(-DENABLE_VIEWER)
    find_package(Pangolin REQUIRED)
endif()
if (ENABLE_ASYNC)
    add_definitions(-DENABLE_ASYNC)
endif()
if (ENABLE_OMP)
    add_definitions(-DENABLE_OMP)
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(realsense2)
find_package(GLEW REQUIRED)
find_package(Ceres REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# 指定第三方库路径
link_directories(${PROJECT_SOURCE_DIR}/lib)

include_directories(${GLEW_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#生成动态库
add_library(${PROJECT_NAME} SHARED
${PROJECT_SOURCE_DIR}/../../../src/System.cc
${PROJECT_SOURCE_DIR}/../../../src/Tracking.cc
${PROJECT_SOURCE_DIR}/../../../src/LocalMapping.cc
${PROJECT_SOURCE_DIR}/../../../src/LoopClosing.cc
${PROJECT_SOURCE_DIR}/../../../src/ORBextractor.cc
${PROJECT_SOURCE_DIR}/../../../src/ORBmatcher.cc
${PROJECT_SOURCE_DIR}/../../../src/FrameDrawer.cc
${PROJECT_SOURCE_DIR}/../../../src/Converter.cc
${PROJECT_SOURCE_DIR}/../../../src/MapPoint.cc
${PROJECT_SOURCE_DIR}/../../../src/KeyFrame.cc
${PROJECT_SOURCE_DIR}/../../../src/Atlas.cc
${PROJECT_SOURCE_DIR}/../../../src/Map.cc
${PROJECT_SOURCE_DIR}/../../../src/MapDrawer.cc
${PROJECT_SOURCE_DIR}/../../../src/Optimizer.cc
${PROJECT_SOURCE_DIR}/../../../src/Frame.cc
${PROJECT_SOURCE_DIR}/../../../src/KeyFrameDatabase.cc
${PROJECT_SOURCE_DIR}/../../../src/Sim3Solver.cc
${PROJECT_SOURCE_DIR}/../../../src/Viewer.cc
${PROJECT_SOURCE_DIR}/../../../src/ImuTypes.cc
${PROJECT_SOURCE_DIR}/../../../src/G2oTypes.cc
${PROJECT_SOURCE_DIR}/../../../src/CameraModels/Pinhole.cpp
${PROJECT_SOURCE_DIR}/../../../src/CameraModels/KannalaBrandt8.cpp
${PROJECT_SOURCE_DIR}/../../../src/OptimizableTypes.cpp
${PROJECT_SOURCE_DIR}/../../../src/MLPnPsolver.cpp
${PROJECT_SOURCE_DIR}/../../../src/GeometricTools.cc
${PROJECT_SOURCE_DIR}/../../../src/TwoViewReconstruction.cc
${PROJECT_SOURCE_DIR}/../../../src/Config.cc
${PROJECT_SOURCE_DIR}/../../../src/Settings.cc
${PROJECT_SOURCE_DIR}/../../../src/ImuInitializer.cc
${PROJECT_SOURCE_DIR}/../../../src/polynomial.cc
${PROJECT_SOURCE_DIR}/../../../src/ThreadPool.cpp
${PROJECT_SOURCE_DIR}/../../../src/RegistrationGICP.cc
${PROJECT_SOURCE_DIR}/../../../src/LidarMapping.cc
${PROJECT_SOURCE_DIR}/../../../src/Lidar.cc
${PROJECT_SOURCE_DIR}/../../../src/LidarProcess.cc
${PROJECT_SOURCE_DIR}/../../../include/System.h
${PROJECT_SOURCE_DIR}/../../../include/Tracking.h
${PROJECT_SOURCE_DIR}/../../../include/LocalMapping.h
${PROJECT_SOURCE_DIR}/../../../include/LoopClosing.h
${PROJECT_SOURCE_DIR}/../../../include/ORBextractor.h
${PROJECT_SOURCE_DIR}/../../../include/ORBmatcher.h
${PROJECT_SOURCE_DIR}/../../../include/FrameDrawer.h
${PROJECT_SOURCE_DIR}/../../../include/Converter.h
${PROJECT_SOURCE_DIR}/../../../include/MapPoint.h
${PROJECT_SOURCE_DIR}/../../../include/KeyFrame.h
${PROJECT_SOURCE_DIR}/../../../include/Atlas.h
${PROJECT_SOURCE_DIR}/../../../include/Map.h
${PROJECT_SOURCE_DIR}/../../../include/MapDrawer.h
${PROJECT_SOURCE_DIR}/../../../include/Optimizer.h
${PROJECT_SOURCE_DIR}/../../../include/Frame.h
${PROJECT_SOURCE_DIR}/../../../include/KeyFrameDatabase.h
${PROJECT_SOURCE_DIR}/../../../include/Sim3Solver.h
${PROJECT_SOURCE_DIR}/../../../include/Viewer.h
${PROJECT_SOURCE_DIR}/../../../include/ImuTypes.h
${PROJECT_SOURCE_DIR}/../../../include/G2oTypes.h
${PROJECT_SOURCE_DIR}/../../../include/CameraModels/GeometricCamera.h
${PROJECT_SOURCE_DIR}/../../../include/CameraModels/Pinhole.h
${PROJECT_SOURCE_DIR}/../../../include/CameraModels/KannalaBrandt8.h
${PROJECT_SOURCE_DIR}/../../../include/OptimizableTypes.h
${PROJECT_SOURCE_DIR}/../../../include/MLPnPsolver.h
${PROJECT_SOURCE_DIR}/../../../include/GeometricTools.h
${PROJECT_SOURCE_DIR}/../../../include/TwoViewReconstruction.h
${PROJECT_SOURCE_DIR}/../../../include/SerializationUtils.h
${PROJECT_SOURCE_DIR}/../../../include/Config.h
${PROJECT_SOURCE_DIR}/../../../include/Settings.h
${PROJECT_SOURCE_DIR}/../../../include/ImuInitializer.h
${PROJECT_SOURCE_DIR}/../../../include/util/eigen_defs.h 
${PROJECT_SOURCE_DIR}/../../../include/util/svd.h 
${PROJECT_SOURCE_DIR}/../../../include/imu_ceres.h 
${PROJECT_SOURCE_DIR}/../../../include/methods.h 
${PROJECT_SOURCE_DIR}/../../../include/polynomial.h
${PROJECT_SOURCE_DIR}/../../../include/RegistrationGICP.h
${PROJECT_SOURCE_DIR}/../../../include/LidarMapping.h
${PROJECT_SOURCE_DIR}/../../../include/Lidar.h
${PROJECT_SOURCE_DIR}/../../../include/LidarProcess.h
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/util
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o
${PROJECT_SOURCE_DIR}/../../../Thirdparty/small_gicp
${PROJECT_SOURCE_DIR}/../../../Thirdparty/small_gicp/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${GLEW_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)


# Build examples

## RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
ament_auto_add_executable(rgbd_inertial_ros2 
  src/rgbd_inertial_ros2.cc
)

ament_target_dependencies(rgbd_inertial_ros2 rclcpp sensor_msgs cv_bridge message_filters)

# find_library(DBoW2_LIB NAMES DBoW2 PATHS ${PROJECT_SOURCE_DIR}/lib)
# find_library(g2o_LIB NAMES g2o PATHS ${PROJECT_SOURCE_DIR}/lib)
# find_library(small_gicp_LIB NAMES small_gicp PATHS ${PROJECT_SOURCE_DIR}/lib)
# message(${DBoW2_LIB})
# message(${g2o_LIB})
# message(${small_gicp_LIB})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CERES_LIBRARIES}
${Boost_LIBRARIES}
${GLEW_LIBRARIES}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/lib/libg2o.so
${PROJECT_SOURCE_DIR}/lib/libsmall_gicp.so
# DBoW2::DBoW2
# g2o::g2o
# small_gicp::small_gicp
-lboost_serialization
-lcrypto
)

# message(${CERES_LIBRARIES})
# ament_target_dependencies(rgbd_inertial_ros2 ${PROJECT_NAME})
target_link_libraries(rgbd_inertial_ros2 ${PROJECT_NAME})
install(FILES 
   ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so 
   ${PROJECT_SOURCE_DIR}/lib/libg2o.so 
   ${PROJECT_SOURCE_DIR}/lib/libsmall_gicp.so 
   DESTINATION lib)
install(TARGETS
   ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(TARGETS 
   rgbd_inertial_ros2
   RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_auto_package()





