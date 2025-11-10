cmake_minimum_required(VERSION 4.0)
project(opencv_5g)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

add_executable(opencv_5g src/main.cpp src/json.hpp src/config.hpp src/garage.hpp
        src/guided.h)
add_executable(line_detection src/line_detection.cpp)

add_executable(guided src/guided_main.cpp src/json.hpp src/config.hpp src/garage.hpp)

target_link_libraries(opencv_5g ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(line_detection ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(guided ${OpenCV_LIBS}
        spdlog::spdlog)

target_include_directories(opencv_5g PRIVATE src)

target_include_directories(line_detection PRIVATE src)

target_include_directories(guided PRIVATE src)

