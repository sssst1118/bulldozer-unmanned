#==============================================================================
# CMakeLists.txt
# 推土机无人驾驶规控系统
# 版本: V1.0
#==============================================================================

cmake_minimum_required(VERSION 3.0.2)
project(qt_app)

#==============================================================================
# 编译器设置
#==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt MOC/UIC/RCC 自动处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/UI)

#==============================================================================
# 依赖查找
#==============================================================================
# ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs          # 栅格地图 OccupancyGrid 需要
  tf
  message_generation
  rviz              # 嵌入式 RViz 感知面板
)

# Qt5 依赖
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

#==============================================================================
# 自定义消息
#==============================================================================
add_message_files(
  FILES
  Persondata.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#==============================================================================
# Catkin 包声明
#==============================================================================
catkin_package(
  INCLUDE_DIRS 
    nodes/decision
    nodes/moldboard
    nodes/control
    nodes/can_to_ros
    nodes/ros_to_can
    common/include
    common/simulink
  LIBRARIES qt_app_common
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    message_runtime
    rviz
  DEPENDS Qt5Core Qt5Widgets Qt5Gui
)

#==============================================================================
# 公共头文件路径 (所有节点共用)
#==============================================================================
set(COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common/simulink
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)

#==============================================================================
# 公共静态库 qt_app_common
#==============================================================================
add_library(qt_app_common STATIC
  common/include/qt_app/data_type.h
  common/include/qt_app/log_helper.h
  common/include/qt_app/tool_utils.h
  common/src/log_helper.cpp
  common/src/tool_utils.cpp
)

target_include_directories(qt_app_common PUBLIC
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  pthread
)

#==============================================================================
# decision_node (决策系统)
# 源码目录: nodes/decision/
# 频率: 100Hz
#==============================================================================
add_executable(decision_node
  nodes/decision/decision_system.cpp
  nodes/decision/decision_node_main.cpp
)

target_include_directories(decision_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/decision
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(decision_node
  qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  pthread
)

add_dependencies(decision_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# moldboard_node (铲刀控制器)
# 源码目录: nodes/moldboard/
# 频率: 50Hz
#==============================================================================
add_executable(moldboard_node
  nodes/moldboard/moldboard_controller.cpp
  nodes/moldboard/moldboard_node_main.cpp
)

target_include_directories(moldboard_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/moldboard
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(moldboard_node
  qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  pthread
)

add_dependencies(moldboard_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# control_node (运动控制器)
# 源码目录: nodes/control/
# 频率: 100Hz
#==============================================================================
add_executable(control_node
  nodes/control/control_node.cpp
  nodes/control/control_node_main.cpp
)

target_include_directories(control_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/control
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(control_node
  qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  pthread
)

add_dependencies(control_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# can_to_ros_node (CAN接收节点)
# 源码目录: nodes/can_to_ros/
#==============================================================================
add_executable(can_to_ros_node
  nodes/can_to_ros/can_to_ros.cpp
  nodes/can_to_ros/can_to_ros_main.cpp
)

target_include_directories(can_to_ros_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/can_to_ros
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(can_to_ros_node
  qt_app_common
  ${catkin_LIBRARIES}
)

add_dependencies(can_to_ros_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# ros_to_can_node (CAN发送节点)
# 源码目录: nodes/ros_to_can/
#==============================================================================
add_executable(ros_to_can_node
  nodes/ros_to_can/ros_to_can.cpp
  nodes/ros_to_can/ros_to_can_main.cpp
)

target_include_directories(ros_to_can_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/ros_to_can
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(ros_to_can_node
  qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  pthread
)

add_dependencies(ros_to_can_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# qt_app_node (主应用节点)
# 源码目录: nodes/qt_app_main/
#==============================================================================
add_executable(qt_app_node
  nodes/qt_app_main/main.cpp
  nodes/qt_app_main/manual_control_widget.cpp
  nodes/qt_app_main/manual_control_widget.h    # Qt MOC 需要
  nodes/qt_app_main/grid_map_widget.cpp
  nodes/qt_app_main/grid_map_widget.h          # Qt MOC 需要
  nodes/qt_app_main/rviz_perception_widget.cpp
  nodes/qt_app_main/rviz_perception_widget.h   # Qt MOC 需要
  resources.qrc                                # Qt RCC: img/ 图片资源
)

target_include_directories(qt_app_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/qt_app_main  # manual_control_widget.h
  ${COMMON_INCLUDE_DIRS}
)

# 静默 rviz → OGRE 第三方头文件中的 C++17 -Wregister 警告
target_compile_options(qt_app_node PRIVATE -Wno-register -Wno-deprecated-register)

target_link_libraries(qt_app_node
  qt_app_common
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

add_dependencies(qt_app_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#==============================================================================
# 安装规则
#==============================================================================
# 安装可执行文件
install(TARGETS 
  qt_app_node 
  qt_app_common 
  can_to_ros_node 
  ros_to_can_node 
  decision_node 
  control_node 
  moldboard_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# 安装头文件
install(DIRECTORY common/include/qt_app/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY common/simulink/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/simulink
)

# 安装各节点头文件
install(DIRECTORY nodes/decision/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/decision
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY nodes/moldboard/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/moldboard
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY nodes/control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/control
  FILES_MATCHING PATTERN "*.h"
)

# 安装消息文件
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装UI文件
install(DIRECTORY UI/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/UI
)
