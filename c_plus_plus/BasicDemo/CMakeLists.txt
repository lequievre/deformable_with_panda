# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# Institut Pascal UMR6602
# laurent.lequievre@uca.fr

cmake_minimum_required(VERSION 2.4.3)

PROJECT(BASIC_EXAMPLE)

IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
   if(POLICY CMP0042)
      # Enable MACOSX_RPATH by default.
      cmake_policy(SET CMP0042 NEW)
   endif(POLICY CMP0042)
ENDIF(COMMAND cmake_policy)

message(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_LIST_DIR = ${CMAKE_CURRENT_LIST_DIR}")

set(CMAKE_CXX_STANDARD 11)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/FindBullet.cmake)

# When we call find_package(Bullet), CMake will execute FindBullet.cmake
find_package(Bullet)

INCLUDE_DIRECTORIES(
    ${BULLET_ROOT}/src
    ${BULLET_ROOT}/Extras
    ${BULLET_ROOT}/btgui
    ${BULLET_ROOT}/examples
    ${BULLET_ROOT}/examples/ThirdPartyLibs/glad
)


SET(AppBasicExampleGui_SRCS
	BasicExample.cpp
    ${BULLET_ROOT}/build3/bullet.rc
	${BULLET_ROOT}/examples/StandaloneMain/main_opengl_single_example.cpp
    ${BULLET_ROOT}/examples/ExampleBrowser/OpenGLGuiHelper.cpp
    ${BULLET_ROOT}/examples/ExampleBrowser/GL_ShapeDrawer.cpp
    ${BULLET_ROOT}/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp
    ${BULLET_ROOT}/examples/Utils/b3Clock.cpp
)

#B3_USE_STANDALONE_EXAMPLE : this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION -DB3_USE_STANDALONE_EXAMPLE)


ADD_EXECUTABLE(AppBasicExampleGui
                ${AppBasicExampleGui_SRCS}
)

target_link_libraries(AppBasicExampleGui ${BULLET_LIBRARIES} -lpthread -ldl) 
