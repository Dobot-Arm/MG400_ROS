cmake_minimum_required(VERSION 3.0.2)
project(rosdemo)

set(CMAKE_CXX_STANDARD 17)

## 添加所需的依赖项
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
)
find_package(jsoncpp REQUIRED PATHS ${JSONCPP_DIR})

## 声明自定义的srv文件
add_service_files(
  FILES
  EnableRobot.srv
  DisableRobot.srv
  ClearError.srv
  GetErrorID.srv
  MovL.srv
  Continues.srv
  Sync.srv
)

## 声明msg所依赖的package
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

## 设置包含路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_subdirectory(json)

## 声明要编译的源文件
add_executable(${PROJECT_NAME}
         src/main.cpp
         src/rosDemoMG400.cpp
        )

## 链接catkin和你的依赖项
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## 添加依赖项，确保服务文件和头文件被正确地编译
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
