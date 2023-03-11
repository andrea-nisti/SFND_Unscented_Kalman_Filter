cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)
add_subdirectory(src/simpleukf)

add_library(fusion src/ukf.cpp src/ukf.h src/simpleukf_tools.h)
target_include_directories(fusion PUBLIC src)
target_link_libraries(fusion PRIVATE simpleukf )

# polluting eigen namespace
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (ukf_highway src/main.cpp src/tools.cpp src/render/render.cpp)
target_include_directories(ukf_highway PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries (ukf_highway ${PCL_LIBRARIES} fusion) 