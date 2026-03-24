cmake_minimum_required(VERSION 3.14)
project(AstroSim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)


# Profiling
option(PROFILE "Enable profiling flags" OFF)
if(PROFILE)
    message(STATUS "Profiling enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-inline")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -fno-inline")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g -fno-inline")
endif()

# Dependencies
add_library(glad STATIC external/glad/src/glad.c)

find_package(PkgConfig REQUIRED)
find_package(GSL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(PNG REQUIRED)

# GLFW
find_package(glfw3 REQUIRED)

# Executable
add_executable(runSim
    src/main.cpp
    src/tree.cpp
    src/utils.cpp
    src/gl_utils.cpp
)

# Linking
target_include_directories(runSim PRIVATE
    ${GLEW_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
)

set_target_properties(glad PROPERTIES LINKER_LANGUAGE C)
target_include_directories(glad PUBLIC external/glad/include)
# target_include_directories(runSim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(runSim
    GSL::gsl
    GSL::gslcblas
    OpenMP::OpenMP_CXX
    Eigen3::Eigen

    OpenGL::GL
    glfw
    glad

    PNG::PNG
)

# Optimization flags
target_compile_options(runSim PRIVATE -O3 -march=native -ffast-math)

# Export
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
