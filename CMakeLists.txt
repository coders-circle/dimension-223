cmake_minimum_required (VERSION 2.6)
project (d223)
 
file(GLOB SOURCES src/*.cpp src/ui/*.cpp src/graphics/*.cpp src/backend/*.cpp)
add_executable(d223 ${SOURCES})

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)
pkg_check_modules(BULLET bullet)
find_package( OpenCV REQUIRED )

include_directories("include")
include_directories(${BULLET_INCLUDE_DIRS})
include_directories(${GTKMM_INCLUDE_DIRS})

link_directories(${BULLET_LIBRARY_DIRS})
link_directories(${GTKMM_LIBRARY_DIRS})

target_link_libraries(d223 ${OpenCV_LIBS} ${BULLET_LIBRARIES} ${GTKMM_LIBRARIES} GL GLEW sfml-system sfml-window sfml-graphics SOIL assimp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
