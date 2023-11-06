cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME trouble_shooting)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
