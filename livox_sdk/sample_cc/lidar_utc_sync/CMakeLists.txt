cmake_minimum_required(VERSION 3.0)

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif ()

set(DEMO_NAME lidar_utc_sync)
add_executable(${DEMO_NAME} main.cpp lds_lidar.cpp synchro.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
