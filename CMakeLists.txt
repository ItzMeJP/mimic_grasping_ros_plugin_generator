cmake_minimum_required(VERSION 3.16)
project(mimic_grasping_ros_plugin_generator
        VERSION 24.09.2021
        DESCRIPTION
        "")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
#add_compile_options(-std=c++2a)

include(GNUInstallDirs)

get_filename_component(CURRENT_DIR ./ ABSOLUTE)

#set (INSTALL_PLUGIN_SYSTEM_MANAGEMENT_IN_ROOT True)
set(PLUGINS_DIR_FOLDER ${CURRENT_DIR}/plugins)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        photoneo_skill_msgs
        object_recognition_skill_server
        geometry_msgs
        )

find_package(Eigen3 REQUIRED) # To matrix operations
find_package(Boost REQUIRED COMPONENTS system thread timer)

## Add other libs that does not have an find_package
add_library(transform_manipulation SHARED IMPORTED GLOBAL)
set_property(TARGET transform_manipulation PROPERTY IMPORTED_LOCATION "/usr/local/lib/transform_manipulation/libtransform_manipulation.so")
add_library(plugin_system_management SHARED IMPORTED GLOBAL)
set_property(TARGET plugin_system_management PROPERTY IMPORTED_LOCATION "/usr/local/lib/plugin_system_management/libplugin_system_management.so")

# Just to debug the generated plugin
#add_library(object_localization_ros_debug_plugin SHARED IMPORTED GLOBAL)
#set_property(TARGET object_localization_ros_debug_plugin PROPERTY IMPORTED_LOCATION "${PLUGINS_DIR_FOLDER}/libobject_localization_ros.so")

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(object_localization_ros SHARED
        src/object_recognition_client/obj_localization_ros.cpp
        )

target_link_libraries(object_localization_ros
        transform_manipulation
        plugin_system_management
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        Boost::thread
        Boost::timer
        jsoncpp
        )

add_library(phoxi_cam_object_localization_ros SHARED
        src/object_recognition_client/phoxi_cam_obj_localization_ros.cpp
        )

target_link_libraries(phoxi_cam_object_localization_ros
        transform_manipulation
        plugin_system_management
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        Boost::thread
        Boost::timer
        jsoncpp
        )

set_target_properties(object_localization_ros PROPERTIES
        PUBLIC_HEADER "include/object_recognition_client/obj_localization_ros.h"
        LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_DIR_FOLDER}
        )

set_target_properties(phoxi_cam_object_localization_ros PROPERTIES
        PUBLIC_HEADER "include/object_recognition_client/phoxi_cam_obj_localization_ros.h"
        LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_DIR_FOLDER}
        )

configure_file(install_configurations/obj_localization_ros.pc.in obj_localization_ros.pc @ONLY)
configure_file(install_configurations/phoxi_obj_localization_ros.pc.in phoxi_obj_localization_ros.pc @ONLY)


install(TARGETS object_localization_ros phoxi_cam_object_localization_ros
        LIBRARY DESTINATION ${PLUGINS_DIR_FOLDER}
        PUBLIC_HEADER DESTINATION ${CMAKE_BINARY_DIR}/public_headers/plugins)

install(FILES ${CMAKE_BINARY_DIR}/obj_localization_ros.pc ${CMAKE_BINARY_DIR}/phoxi_obj_localization_ros.pc
        DESTINATION ${CMAKE_BINARY_DIR}/pkgconfig)

add_executable(raw_test
        src/examples/raw_object_recognition_client_node.cpp)

target_link_libraries(
        raw_test
        ${catkin_LIBRARIES}
        plugin_system_management
        transform_manipulation
        Eigen3::Eigen
        Boost::thread
        Boost::timer
        jsoncpp

)

#[[
add_executable(phoxi_cam_test
        src/examples/phoxi_cam_object_recognition_client_node.cpp)

target_link_libraries(
        phoxi_cam_test
        ${catkin_LIBRARIES}
        plugin_system_management
        transform_manipulation
        Eigen3::Eigen
        Boost::thread
        Boost::timer
        jsoncpp


)
]]