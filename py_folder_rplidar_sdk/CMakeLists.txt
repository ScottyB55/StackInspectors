cmake_minimum_required(VERSION 3.5)

project(py_rplidar_sdk)

# Find python and Boost - both are required dependencies
find_package(PythonLibs 3.8 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

# Add a shared module - modules are intended to be imported at runtime.
# - This is where you add the source files
add_library(s2lidar MODULE s2lidar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/sl_lidar_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/arch/linux/timer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/arch/linux/net_serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/arch/linux/net_socket.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/hal/thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/sl_crc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/sl_serial_channel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/sl_tcp_channel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src/sl_udp_channel.cpp
)

# Set up the libraries and header search paths for this target
target_link_libraries(s2lidar ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(s2lidar PRIVATE ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src
)
