cmake_minimum_required(VERSION 3.5)
project(hk_camera)

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

# 开启编译优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
  /opt/MVS/include/
)

LINK_DIRECTORIES(
/opt/MVS/lib/64/
)

add_executable(${PROJECT_NAME} src/hk_camera.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  # GCBase_gcc421_v3_0
  MvCameraControl
  # GenApi_gcc421_v3_0
  MVGigEVisionSDK
  # log4cpp_gcc421_v3_0
  MVRender
  # Log_gcc421_v3_0
  MvUsb3vTL
  # MathParser_gcc421_v3_0
  # NodeMapData_gcc421_v3_0
  MediaProcess
  # XmlParser_gcc421_v3_0
  X11
)

add_executable(hk_camera_compressed src/hk_camera_compressed.cpp)
ament_target_dependencies(hk_camera_compressed
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(hk_camera_compressed
  ${OpenCV_LIBRARIES}
  # GCBase_gcc421_v3_0
  MvCameraControl
  # GenApi_gcc421_v3_0
  MVGigEVisionSDK
  # log4cpp_gcc421_v3_0
  MVRender
  # Log_gcc421_v3_0
  MvUsb3vTL
  # MathParser_gcc421_v3_0
  # NodeMapData_gcc421_v3_0
  MediaProcess
  # XmlParser_gcc421_v3_0
  X11
)

add_executable(hk_camera_compressed_sub src/hk_camera_compressed_sub.cpp)
ament_target_dependencies(hk_camera_compressed_sub
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(hk_camera_compressed_sub
  ${OpenCV_LIBRARIES}
  # GCBase_gcc421_v3_0
  MvCameraControl
  # GenApi_gcc421_v3_0
  MVGigEVisionSDK
  # log4cpp_gcc421_v3_0
  MVRender
  # Log_gcc421_v3_0
  MvUsb3vTL
  # MathParser_gcc421_v3_0
  # NodeMapData_gcc421_v3_0
  MediaProcess
  # XmlParser_gcc421_v3_0
  X11
)

install(TARGETS
  hk_camera
  hk_camera_compressed
  hk_camera_compressed_sub
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
