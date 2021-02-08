cmake_minimum_required(VERSION 3.5)
project(murmurations)

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
include_directories(${SFML_INCLUDE_DIR})
link_libraries(sfml-graphics)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Generate the static library from the library sources
add_library(murmurations STATIC 
    src/Helpers.cpp
    src/Flock.cpp
    src/Boid.cpp
)

target_include_directories(murmurations
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(main src/main.cpp)
target_link_libraries(main murmurations sfml-graphics)

# Tests
# enable_testing()
# find_package(GTest MODULE REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# add_executable(runtests test/main.cpp)
# target_link_libraries(runtests PUBLIC murmurations PRIVATE GTest::GTest GTest::Main)
# add_test(NAME alltests COMMAND runtests)
