cmake_minimum_required(VERSION 3.5)
project( run_me )
find_package( OpenCV REQUIRED )
message("OpenCV version: ${OpenCV_VERSION}")

# to set file, run `cmake -DFILE=filename.cpp`, and then to build run `cmake --build .`
set(FILE "main.c" CACHE STRING "C file to compile")

message(STATUS "Compiling file: ${FILE}")

add_executable(run ${FILE})

target_link_libraries(run ${OpenCV_LIBS})

# add_executable( run_me current_file.cpp )
# target_link_libraries( run_me ${OpenCV_LIBS} )
