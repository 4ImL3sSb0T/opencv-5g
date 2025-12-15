cmake_minimum_required(VERSION 3.10)
project(opencv_5g)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

add_executable(opencv_5g src/main.cpp src/json.hpp src/config.hpp src/garage.hpp
        src/guided.h src/image_Q.cpp)
add_executable(line_detection src/line_detection.cpp)

add_executable(guided src/guided_main.cpp src/json.hpp src/config.hpp src/garage.hpp
        )

add_executable(trace src/trace.cpp src/json.hpp src/config.hpp src/garage.hpp src/pigpio.h src/image_Q.cpp)

add_executable(onnx_test src/onnx_test.cpp src/onnx_inference.hpp)
add_executable(arrow_test src/car/arrow_test.cpp src/car/ArrowDetector.cpp)
add_executable(yolo_test src/car/yolo_test.cpp src/car/YOLOv5Detector.cpp)

target_link_libraries(opencv_5g ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(line_detection ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(guided ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(trace ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(onnx_test ${OpenCV_LIBS})
target_link_libraries(arrow_test ${OpenCV_LIBS})
target_link_libraries(yolo_test ${OpenCV_LIBS})

target_include_directories(opencv_5g PRIVATE src)

target_include_directories(line_detection PRIVATE src)

target_include_directories(guided PRIVATE src)

target_include_directories(trace PRIVATE src)

target_include_directories(onnx_test PRIVATE src)
target_include_directories(arrow_test PRIVATE src)
target_include_directories(yolo_test PRIVATE src)
