cmake_minimum_required(VERSION 3.16)
project(route_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
add_compile_options(-Wall -Wextra)

add_library(routecore src/graph.cpp src/search.cpp)
target_include_directories(routecore PUBLIC src)

add_executable(route_engine src/main.cpp)
target_link_libraries(route_engine routecore)

enable_testing()
add_executable(test_search tests/test_search.cpp)
target_link_libraries(test_search routecore)
add_test(NAME search COMMAND test_search)
