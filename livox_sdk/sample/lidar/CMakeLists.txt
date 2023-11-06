cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_sample)
add_executable(${DEMO_NAME} main.c)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
