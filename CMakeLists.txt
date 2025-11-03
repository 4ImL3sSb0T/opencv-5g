cmake_minimum_required(VERSION 4.0)
project(opencv_5g)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

add_executable(opencv_5g src/main.cpp src/json.hpp src/config.hpp src/garage.hpp)

target_link_libraries(opencv_5g ${OpenCV_LIBS}
        spdlog::spdlog)

target_include_directories(opencv_5g PRIVATE src)

