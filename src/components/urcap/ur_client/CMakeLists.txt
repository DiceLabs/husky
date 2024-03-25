cmake_minimum_required(VERSION 3.5.1)
project(urclient)

find_package(ur_client_library REQUIRED)

set(SOURCES
    src/release_brakes.cpp
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ur_client_library::urcl)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})