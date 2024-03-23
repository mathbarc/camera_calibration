project(sphere_visualizer)

find_package(OpenCV COMPONENTS core highgui imgproc videoio calib3d REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

add_executable(sphere_visualizer "main.cpp")
target_link_libraries(sphere_visualizer ${OpenCV_LIBRARIES})

add_executable(camera_calibration "camera_calibration.cpp")
target_link_libraries(camera_calibration ${OpenCV_LIBRARIES})