cmake_minimum_required(VERSION 3.10)

project(interactive-invigoration VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true)

find_package(glfw3 3.3 REQUIRED)
find_package(glm CONFIG REQUIRED)

find_package(CGAL REQUIRED)

# external libraries
add_subdirectory(external/glad)

# source files by directory
file(GLOB CORE_SOURCES src/core/*.cpp)
file(GLOB SIMULATION_SOURCES src/simulation/*.cpp)
file(GLOB GEOMETRY_SOURCES src/geometry/*.cpp)
file(GLOB MAIN_SOURCE src/main.cpp)

# combine all source files
set(SOURCES
    ${CORE_SOURCES}
    ${SIMULATION_SOURCES}
    ${GEOMETRY_SOURCES}
    ${MAIN_SOURCE}
)

# include directories
include_directories(
    include
    include/core
    include/simulation
    include/geometry
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} glfw glm::glm CGAL::CGAL glad)

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR})
