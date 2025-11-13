cmake_minimum_required(VERSION 3.19)
project(fast_livo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(WITH_OPENMP "Build with OpenMP support" ON)
option(WITH_MIMALLOC "Build with mimalloc support" OFF)

if(EXISTS "${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
  include(${CMAKE_BINARY_DIR}/conan_toolchain.cmake)
endif()

# Only set build type if not defined (to respect Conan/multi-config generators)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# Architecture-specific tuning (applied via target properties)
set(FAST_LIVO_RELEASE_FLAGS "")
set(FAST_LIVO_ARCH_DEFINES "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(FAST_LIVO_RELEASE_FLAGS "-mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: ${FAST_LIVO_RELEASE_FLAGS}")
  else()
    set(FAST_LIVO_RELEASE_FLAGS "-mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: ${FAST_LIVO_RELEASE_FLAGS}")
  endif()
  list(APPEND FAST_LIVO_ARCH_DEFINES ARM_ARCH)
else()
  set(FAST_LIVO_RELEASE_FLAGS "-march=native -mtune=native -funroll-loops")
  list(APPEND FAST_LIVO_ARCH_DEFINES X86_ARCH)
  message(STATUS "Using x86 optimizations: ${FAST_LIVO_RELEASE_FLAGS}")
endif()
string(REPLACE " " ";" FAST_LIVO_RELEASE_FLAGS_LIST "${FAST_LIVO_RELEASE_FLAGS}")

# Detect CPU core count for potential multithreading optimization
# include(ProcessorCount)
# ProcessorCount(N)
# message(STATUS "Processor count: ${N}")

# # Set the number of cores for multithreading
# if(N GREATER 4)
#   math(EXPR PROC_NUM "4")
#   add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
#   message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
# elseif(N GREATER 1)
#   math(EXPR PROC_NUM "${N}")
#   add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
#   message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
# else()
#   add_definitions(-DMP_PROC_NUM=1)
#   message(STATUS "Single core detected. Multithreading disabled.")
# endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Sophus CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS thread system)

add_subdirectory(rpg_vikit/vikit_common)
add_subdirectory(rpg_vikit/vikit_ros)

# ROS 2 dependencies
find_package(ament_cmake CONFIG REQUIRED)
find_package(rclcpp CONFIG REQUIRED)
find_package(sensor_msgs CONFIG REQUIRED)
find_package(visualization_msgs CONFIG REQUIRED)
find_package(geometry_msgs CONFIG REQUIRED)
find_package(nav_msgs CONFIG REQUIRED)
find_package(std_msgs CONFIG REQUIRED)
find_package(cv_bridge CONFIG REQUIRED)
find_package(tf2 CONFIG REQUIRED)
find_package(tf2_ros CONFIG REQUIRED)
find_package(tf2_geometry_msgs CONFIG REQUIRED)

find_package(pcl_conversions CONFIG REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(image_transport REQUIRED)


if(WITH_MIMALLOC)
  find_package(mimalloc CONFIG QUIET)
  if(mimalloc_FOUND)
    message(STATUS "mimalloc found")
  else()
    message(WARNING "mimalloc requested but not found")
  endif()
endif()

# Optional OpenMP
set(FAST_LIVO_OPENMP_TARGET "")
if(WITH_OPENMP)
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    set(FAST_LIVO_OPENMP_TARGET OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP enabled")
  else()
    message(WARNING "OpenMP requested but not found; continuing without it")
  endif()
endif()

# Interface library for common dependencies
add_library(fast_livo_interface INTERFACE)
target_include_directories(fast_livo_interface INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(fast_livo_interface INTERFACE
    Eigen3::Eigen
    PCL::PCL
    opencv_core opencv_imgproc opencv_imgcodecs
    Sophus::Sophus
    Boost::boost
    vikit_common
    vikit_ros
    ${FAST_LIVO_OPENMP_TARGET}
)
target_include_directories(fast_livo_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Common compiler definitions/options
target_compile_definitions(fast_livo_interface INTERFACE
    ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
    ${FAST_LIVO_ARCH_DEFINES}
)
if(FAST_LIVO_RELEASE_FLAGS_LIST)
  target_compile_options(fast_livo_interface INTERFACE
      $<$<CONFIG:Release>:${FAST_LIVO_RELEASE_FLAGS_LIST}>
      $<$<CONFIG:RelWithDebInfo>:${FAST_LIVO_RELEASE_FLAGS_LIST}>
  )
endif()

# Core libraries
add_library(vio 
    src/vio.cpp 
    src/frame.cpp 
    src/visual_point.cpp
)
target_link_libraries(vio PUBLIC fast_livo_interface)

add_library(lio src/voxel_map.cpp)
target_link_libraries(lio PUBLIC fast_livo_interface)

add_library(pre src/preprocess.cpp)
target_link_libraries(pre PUBLIC fast_livo_interface)

add_library(imu_proc src/IMU_Processing.cpp)
target_link_libraries(imu_proc PUBLIC fast_livo_interface)

add_library(laser_mapping src/LIVMapper.cpp)
target_link_libraries(laser_mapping PUBLIC 
    fast_livo_interface
    vio
    lio
    pre
    imu_proc
)

# Main executable
add_executable(fastlivo_mapping src/main.cpp)
target_link_libraries(fastlivo_mapping PRIVATE
    laser_mapping
)

# Optional mimalloc
if(WITH_MIMALLOC AND mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping PRIVATE mimalloc)
endif()

# ROS 2 support

message(STATUS "ROS 2 support enabled")
ament_target_dependencies(vio PUBLIC
    sensor_msgs
    geometry_msgs
    std_msgs
)

ament_target_dependencies(lio PUBLIC
    sensor_msgs
    std_msgs
)

ament_target_dependencies(pre PUBLIC
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
)

ament_target_dependencies(imu_proc PUBLIC
    sensor_msgs
    nav_msgs
)

ament_target_dependencies(laser_mapping PUBLIC
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    cv_bridge
    tf2
    tf2_ros
    image_transport
)

ament_target_dependencies(fastlivo_mapping PUBLIC
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    std_msgs
    cv_bridge
    tf2
    tf2_ros
    pcl_conversions
    image_transport
)

# Install targets
install(
    TARGETS
        fastlivo_mapping
        vio
        lio
        pre
        imu_proc
        laser_mapping
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install config and launch
install(
    DIRECTORY
        config
        launch
    DESTINATION share/${PROJECT_NAME}/
)

# Enable ros2 launch without install (for editable mode)
file(
    COPY
        package.xml
        config
        launch
    DESTINATION ${CMAKE_BINARY_DIR}/ament_cmake_index/share/${PROJECT_NAME}/
)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/ament_cmake_index/lib/${PROJECT_NAME}/)
add_custom_command(
    TARGET fastlivo_mapping POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:fastlivo_mapping>
        ${CMAKE_BINARY_DIR}/ament_cmake_index/lib/${PROJECT_NAME}/
)

ament_package()
