cmake_minimum_required(VERSION 2.8.3)
project(dashboard)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

set(QT Core Gui Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(INC_DIR .)
set(UI_DIR .)
set(MOC_DIR .)
set(DEPEND_PATH .)

find_package(Qt5 REQUIRED ${QT})

set(SOURCES
  main.cpp
  LineItemWidget.cpp
)

set(HEADERS
  QDashboard.h
  LineItemWidget.h
)

set(FORMS
  QDashboard.ui
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  .
)


add_message_files(FILES Request.msg)
generate_messages(DEPENDENCIES std_msgs) 

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)

add_executable(abc_node ${SOURCES} ${HEADERS} ${FORMS})

qt5_use_modules(abc_node ${QT})

target_link_libraries(abc_node
   ${catkin_LIBRARIES}
   Qt5::Widgets
)