cmake_minimum_required(VERSION 3.5)
project(murmurations)

if(WIN32)
  message("Searching Windows for dependencies ...")
  # GLAD
  include_directories("C:/Dependencies/GLAD/include")
  add_library(glad STATIC "C:/Dependencies/GLAD/src/glad.c")
  link_libraries(glad)
  # GLFW
  include_directories("C:/Dependencies/GLFW/include")
  link_libraries("C:/Dependencies/GLFW/lib-vc2019/glfw3.lib")
elseif(UNIX)
  message("Searching Linux for dependencies ...")
  # GLAD
  add_library(glad STATIC /usr/local/src/glad.c)
  link_libraries(glad)
  # GLFW
  # from https://www.glfw.org/docs/3.0/build.html
  find_package(PkgConfig REQUIRED)
  pkg_search_module(GLFW REQUIRED glfw3)
  include_directories(${GLFW_INCLUDE_DIRS})
  link_libraries(${GLFW_LIBRARIES})
  link_libraries(${GLFW_STATIC_LIBRARIES})
endif()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES
  src/Helpers.cpp
  src/Flock.cpp
  src/Boid.cpp
  src/Shader.cpp
)

add_library(murmurations STATIC ${SOURCE_FILES})

target_include_directories(murmurations
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(main src/Main.cpp)
target_link_libraries(main murmurations)

# Tests
# enable_testing()
# find_package(GTest MODULE REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# add_executable(runtests test/Main.cpp)
# target_link_libraries(runtests PUBLIC murmurations PRIVATE GTest::GTest GTest::Main)
# add_test(NAME alltests COMMAND runtests)
