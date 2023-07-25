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
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

add_executable(hk_camera_compressed_pub src/hk_camera_compressed_pub.cpp)
ament_target_dependencies(hk_camera_compressed_pub
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(hk_camera_compressed_pub
  ${OpenCV_LIBRARIES}
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
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
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

# 用于测试发布压缩图像
add_executable(hk_camera_compressed_pub_test src/hk_camera_compressed_pub_test.cpp)
ament_target_dependencies(hk_camera_compressed_pub_test
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(hk_camera_compressed_pub_test
  ${OpenCV_LIBRARIES}
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

# 用于测试订阅压缩图像
add_executable(hk_camera_compressed_sub_test src/hk_camera_compressed_sub_test.cpp)
ament_target_dependencies(hk_camera_compressed_sub_test
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(hk_camera_compressed_sub_test
  ${OpenCV_LIBRARIES}
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

install(TARGETS
  hk_camera
  hk_camera_compressed_pub
  hk_camera_compressed_sub
  hk_camera_compressed_pub_test
  hk_camera_compressed_sub_test
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
