cmake_minimum_required(VERSION 3.10)
project(opencv_5g)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# ONNX Runtime配置
set(ONNXRUNTIME_DIR "${CMAKE_SOURCE_DIR}/lib/onnxruntime-win-x64-1.23.2")
set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_DIR}/lib")
set(ONNXRUNTIME_LIBS "${ONNXRUNTIME_LIB_DIR}/onnxruntime.lib")

add_executable(trace src/trace.cpp src/json.hpp src/config.hpp)

add_executable(ab_test src/car/ab_test.cpp src/car/ABDetector.cpp)
# YOLO推理测试程序
add_executable(test_yolo_infer src/car/test_yolo.cpp src/car/yolo_detector.cpp src/car/yolo_detector.hpp)

target_link_libraries(trace ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(ab_test ${OpenCV_LIBS})

# YOLO推理测试程序链接
target_link_libraries(test_yolo_infer ${OpenCV_LIBS} ${ONNXRUNTIME_LIBS})
target_include_directories(test_yolo_infer PRIVATE src ${ONNXRUNTIME_INCLUDE_DIRS})

target_include_directories(trace PRIVATE src)

target_include_directories(ab_test PRIVATE src)
