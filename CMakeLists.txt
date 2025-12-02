cmake_minimum_required(VERSION 3.14)
project(AstroSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Find dependencies
find_package(PkgConfig REQUIRED)
find_package(GSL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

pkg_check_modules(CAIRO REQUIRED cairo)

include_directories(${CAIRO_INCLUDE_DIRS})

# Add your code
add_executable(runSim
    src/main.cpp
    # src/integrator.cpp
    # src/model.cpp
)

# Link libraries
target_link_libraries(runSim
    GSL::gsl
    GSL::gslcblas
    OpenMP::OpenMP_CXX
	Eigen3::Eigen
)

target_link_libraries(runSim ${CAIRO_LIBRARIES})

# Optimization flags
target_compile_options(runSim PRIVATE -O3 -march=native -ffast-math)