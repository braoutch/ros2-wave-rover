cmake_minimum_required(VERSION 3.14)

project(GrosPote LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core SerialPort REQUIRED)

FILE(GLOB_RECURSE SRC ${CMAKE_SOURCE_DIR}/source/*.cpp)

include_directories(GrosPote ${CMAKE_SOURCE_DIR}/include)

add_executable(GrosPote main.cpp ${SRC}
)

target_link_libraries(GrosPote Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::SerialPort)
