cmake_minimum_required(VERSION 3.14)

find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

find_package(GLFW3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/deps/dynamic-manipulation/inc)
include_directories(${PROJECT_SOURCE_DIR}/deps/dynamic-manipulation/sample)

add_library(pcl_util pcl_util.hpp pcl_util.cpp)
target_link_libraries(
    pcl_util
    ${PCL_LIBRARIES}
    DynamicManipulation
)

add_library(pcl_viewer pcl_viewer.hpp pcl_viewer.cpp)
target_link_libraries(
    pcl_viewer
    pcl_util
    DynamicManipulation
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLFW3_LIBRARY}
)

include_directories(${PCL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})
add_library(pcl_grabber pcl_grabber.cpp pcl_grabber.hpp)
target_link_libraries(
    pcl_grabber
    pcl_util
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
)

add_executable(pcl_viewer_test pcl_viewer_test.cpp)
target_link_libraries(
    pcl_viewer_test
    pcl_viewer
    pcl_grabber
    ${realsense2_LIBRARY}
)

add_library(ActionHandler
    ActionHandler.hpp
    ActionHandler.cpp
)
add_executable(ActionHandlerTest
    ActionHandlerTest.cpp
)
target_link_libraries(ActionHandlerTest
    ActionHandler
)
set_target_properties(ActionHandlerTest PROPERTIES FOLDER "test")

add_library(HandStateReader
    HandStateReader.hpp
    HandStateReader.cpp
)    
target_link_libraries(
    HandStateReader
    pcl_util
    ${PCL_LIBRARIES}
    pcl_grabber
    DynamicManipulation
)

add_executable(HandStateReaderTest
    HandStateReaderTest.cpp
)
target_link_libraries(
    HandStateReaderTest
    DynamicManipulation
    StereoTracker
    CameraDevice
    ImgProcUtil
    HandStateReader
    pcl_util
    pcl_grabber
    pcl_viewer
)
set_target_properties(HandStateReaderTest PROPERTIES FOLDER "test")

add_executable(
    graspable
    graspable.cpp
)
target_link_libraries(
    graspable
    DynamicManipulation
    StereoTracker
    CameraDevice
    ImgProcUtil
    HandStateReader
    pcl_util
    pcl_grabber
    pcl_viewer
    ActionHandler
)

add_executable(
    demo_grasp
    demo_grasp.cpp
)
target_link_libraries(
    demo_grasp
    DynamicManipulation
    StereoTracker
    CameraDevice
    ImgProcUtil
    HandStateReader
    pcl_util
    pcl_grabber
    pcl_viewer
    ActionHandler
)
set_target_properties(demo_grasp PROPERTIES FOLDER "demos")

add_executable(
    demo_hand_state
    demo_hand_state.cpp
)
target_link_libraries(
    demo_hand_state
    DynamicManipulation
    StereoTracker
    CameraDevice
    ImgProcUtil
    HandStateReader
    pcl_util
    pcl_grabber
    pcl_viewer
    ActionHandler
)
set_target_properties(demo_hand_state PROPERTIES FOLDER "demos")