cmake_minimum_required(VERSION 2.8.3)
project(opt_pipeline)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  pcl_ros
  nav_msgs
  sensor_msgs
)
find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

# 手动添加 JsonCpp 头文件路径和库
# 头文件位于 /usr/include/jsoncpp
include_directories(
  ${PROJECT_SOURCE_DIR}/interactive_interface/include
  ${PROJECT_SOURCE_DIR}/interactive_interface/thirdparty/gl3w
  ${PROJECT_SOURCE_DIR}/interactive_interface/thirdparty/imgui
  ${PROJECT_SOURCE_DIR}/interactive_interface/thirdparty/imgui/examples
  ${PROJECT_SOURCE_DIR}/interactive_interface/thirdparty/portable-file-dialogs
  ${PROJECT_SOURCE_DIR}/odometry2graph/include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  /usr/include/jsoncpp
)
link_directories(
  ${G2O_LIBRARY_DIRS}
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

# #########
# Build ##
# #########
add_library(imgui STATIC
  interactive_interface/thirdparty/gl3w/gl3w.c
  interactive_interface/thirdparty/imgui/imgui.cpp
  interactive_interface/thirdparty/imgui/imgui_demo.cpp
  interactive_interface/thirdparty/imgui/imgui_draw.cpp
  interactive_interface/thirdparty/imgui/imgui_widgets.cpp
  interactive_interface/thirdparty/imgui/examples/imgui_impl_glfw.cpp
  interactive_interface/thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)
target_link_libraries(imgui
  ${OPENGL_LIBRARIES}
  glfw
  dl
)

add_library(guik STATIC
  interactive_interface/src/glk/mesh.cpp
  interactive_interface/src/glk/lines.cpp
  interactive_interface/src/glk/colormap.cpp
  interactive_interface/src/glk/glsl_shader.cpp
  interactive_interface/src/glk/frame_buffer.cpp
  interactive_interface/src/glk/pointcloud_buffer.cpp
  interactive_interface/src/glk/primitives/primitives.cpp
  interactive_interface/src/glk/loaders/ply_loader.cpp
  interactive_interface/src/guik/gl_canvas.cpp
  interactive_interface/src/guik/model_control.cpp
  interactive_interface/src/guik/camera_control.cpp
  interactive_interface/src/guik/projection_control.cpp
  interactive_interface/src/guik/imgui_application.cpp
)

target_link_libraries(guik
  imgui
)

add_library(odometry2graph STATIC
  odometry2graph/src/odometry2graph.cpp
  odometry2graph/src/jsonconvert.cpp
)

target_link_libraries(odometry2graph
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_TYPES_SLAM3D}
  ${OPENGL_LIBRARIES}
  ${catkin_LIBRARIES}
  jsoncpp
)

add_executable(test_odometry2graph
  test/test_odometry2graph.cpp
)
target_link_libraries(test_odometry2graph
  odometry2graph
)

add_executable(interactive_interface
  interactive_interface/src/interactive_interface.cpp
)
target_link_libraries(interactive_interface
  odometry2graph
  glfw
  guik
  imgui
)
