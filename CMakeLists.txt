cmake_minimum_required(VERSION 3.14)

project(gros-pote LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core SerialPort REQUIRED)

#ROS 2 stuff
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

FILE(GLOB_RECURSE SRC ${CMAKE_SOURCE_DIR}/src/source/*.cpp ${CMAKE_SOURCE_DIR}/src/include/*.hpp)

include_directories(gros-pote ${CMAKE_SOURCE_DIR}/src/include)

add_executable(gros-pote-node ${SRC}
)

target_include_directories(gros-pote-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gros-pote-node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gros-pote-node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_target_dependencies(gros-pote-node rclcpp geometry_msgs)
target_link_libraries(gros-pote-node Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::SerialPort)


# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_package()