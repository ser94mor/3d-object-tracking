cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

set(YOLOV3_WEIGHTS_URL "https://pjreddie.com/media/files/yolov3.weights")
set(YOLOV3_WEIGHTS_PATH "../dat/yolo/yolov3.weights")
set(YOLOV3_TINY_WEIGHTS_URL "https://pjreddie.com/media/files/yolov3-tiny.weights")
set(YOLOV3_TINY_WEIGHTS_PATH "../dat/yolo/yolov3-tiny.weights")

if (NOT EXISTS "${YOLOV3_WEIGHTS_PATH}")
	execute_process(COMMAND echo -e "Downloading yolov3.weights from ${YOLOV3_WEIGHTS_URL}."
	                                "\nIt may take a significant amount of time.")
    file(DOWNLOAD "${YOLOV3_WEIGHTS_URL}" "${YOLOV3_WEIGHTS_PATH}")
endif()

if (NOT EXISTS "${YOLOV3_TINY_WEIGHTS_PATH}")
	execute_process(COMMAND echo "Downloading yolov3-tiny.weights from ${YOLOV3_TINY_WEIGHTS_URL}."
                                     "\nIt may take a significant amount of time.")
    file(DOWNLOAD "${YOLOV3_TINY_WEIGHTS_URL}" "${YOLOV3_TINY_WEIGHTS_PATH}")
endif()

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (3D_object_tracking src/camFusion_Student.cpp src/FinalProject_Camera.cpp src/lidarData.cpp src/matching2D_Student.cpp src/objectDetection2D.cpp)
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})
