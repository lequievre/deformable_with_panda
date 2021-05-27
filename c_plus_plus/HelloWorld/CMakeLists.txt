# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# Institut Pascal UMR6602
# laurent.lequievre@uca.fr

cmake_minimum_required(VERSION 2.4.3)

PROJECT(MY_TEST)

set(CMAKE_CXX_STANDARD 11)
set(BULLET_ROOT "/home/laurent/git_projects/bullet3")
# locate FindPackageHandleStandardArgs.cmake -> /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake
set(CMAKE_MODULE_PATH "/usr/share/cmake-3.16/Modules")

include(FindBullet.cmake)

# When we call find_package(Bullet), CMake will execute FindBullet.cmake
find_package(Bullet)

# Enable to compile Bullet with a double precision.
# By default, btScalar is a typedef to float. It can be double by defining BT_USE_DOUBLE_PRECISION in your build system.
ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION)

INCLUDE_DIRECTORIES(
    ${BULLET_INCLUDE_DIR}
)


ADD_EXECUTABLE(App_My_HelloWorld
    HelloWorld.cpp
)

target_link_libraries(App_My_HelloWorld ${BULLET_LIBRARIES})

