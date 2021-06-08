# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# Institut Pascal UMR6602
# laurent.lequievre@uca.fr

# pthread -> sudo apt-get install libpthread-stubs0-dev
# OpenGL -> sudo apt-get install libegl1-mesa-dev

cmake_minimum_required(VERSION 2.4.3)

set(CMAKE_BUILD_TYPE Release)

PROJECT(MY_FIRST_OPENGL_WINDOW)

IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
   if(POLICY CMP0042)
      # Enable MACOSX_RPATH by default.
      cmake_policy(SET CMP0042 NEW)
   endif(POLICY CMP0042)
ENDIF(COMMAND cmake_policy)

message(STATUS "PROJECT-> PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
message(STATUS "PROJECT -> PROJECT_BINARY_DIR = ${PROJECT_BINARY_DIR}")
message(STATUS "PROJECT-> CMAKE_CURRENT_LIST_DIR = ${CMAKE_CURRENT_LIST_DIR}")

set(CMAKE_CXX_STANDARD 11)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/FindBullet.cmake)

# When we call find_package(Bullet), CMake will execute FindBullet.cmake
find_package(Bullet)

message(STATUS "PROJECT-> BULLET_ROOT = ${BULLET_ROOT}")

INCLUDE_DIRECTORIES(
    ${BULLET_ROOT}/src
    ${BULLET_ROOT}/data
    ${BULLET_ROOT}/examples
    ${BULLET_ROOT}/examples/SharedMemory
	${BULLET_ROOT}/examples/ThirdPartyLibs
	${BULLET_ROOT}/examples/ThirdPartyLibs/enet/include
	${BULLET_ROOT}/examples/ThirdPartyLibs/clsocket/src
)


#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "inclue dir='${dir}'")
#endforeach()

GET_DIRECTORY_PROPERTY(output_all_include_dirs INCLUDE_DIRECTORIES)
message(STATUS "PROJECT-> INCLUDE_DIRECTORIES = ${output_all_include_dirs}")


#B3_USE_STANDALONE_EXAMPLE : this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION -DB3_USE_STANDALONE_EXAMPLE -DB3_USE_ROBOTSIM_GUI)


SET(MyBrowser_SRCS
    main.cpp
	ExampleEntries.cpp
	LargeDeformation.cpp
	${BULLET_ROOT}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h
	${BULLET_ROOT}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
	#Importers/ImportBullet/SerializeSetup.cpp
)

ADD_EXECUTABLE(App_MyBrowser
                ${MyBrowser_SRCS}
)

target_link_libraries(App_MyBrowser ${BULLET_LIBRARIES} -lpthread -ldl) 


ADD_CUSTOM_COMMAND(
                TARGET App_MyBrowser
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory ${PROJECT_SOURCE_DIR}/data ${PROJECT_BINARY_DIR}/data
        )
        
        
ADD_CUSTOM_COMMAND(
                TARGET App_MyBrowser
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory ${PROJECT_SOURCE_DIR}/urdf ${PROJECT_BINARY_DIR}/urdf
        )
