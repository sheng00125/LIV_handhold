cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME hub_sample_cc)
add_executable(${DEMO_NAME} lds_hub.cpp main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
