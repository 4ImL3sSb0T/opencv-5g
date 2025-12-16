cmake_minimum_required(VERSION 4.0)
project(opencv_5g)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# ONNX Runtime配置
set(ONNXRUNTIME_DIR "${CMAKE_SOURCE_DIR}/lib/onnxruntime-win-x64-1.23.2")
set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_DIR}/lib")
set(ONNXRUNTIME_LIBS "${ONNXRUNTIME_LIB_DIR}/onnxruntime.lib")

add_executable(opencv_5g src/main.cpp src/json.hpp src/config.hpp src/garage.hpp
        src/guided.h src/image_Q.cpp)
add_executable(guided src/guided_main.cpp src/json.hpp src/config.hpp src/garage.hpp
        )

add_executable(trace src/trace.cpp src/json.hpp src/config.hpp src/garage.hpp src/pigpio.h src/image_Q.cpp)

add_executable(onnx_test src/onnx_test.cpp src/onnx_inference.hpp)
add_executable(arrow_test src/car/arrow_test.cpp src/car/ArrowDetector.cpp)

# YOLO推理测试程序
add_executable(test_yolo_infer src/car/test_yolo.cpp src/car/yolo_detector.cpp src/car/yolo_detector.hpp)

target_link_libraries(opencv_5g ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(guided ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(trace ${OpenCV_LIBS}
        spdlog::spdlog)

target_link_libraries(onnx_test ${OpenCV_LIBS})
target_link_libraries(arrow_test ${OpenCV_LIBS})

# YOLO推理测试程序链接
target_link_libraries(test_yolo_infer ${OpenCV_LIBS} ${ONNXRUNTIME_LIBS})
target_include_directories(test_yolo_infer PRIVATE src ${ONNXRUNTIME_INCLUDE_DIRS})

target_include_directories(opencv_5g PRIVATE src)

target_include_directories(guided PRIVATE src)

target_include_directories(trace PRIVATE src)

target_include_directories(onnx_test PRIVATE src)
target_include_directories(arrow_test PRIVATE src)
