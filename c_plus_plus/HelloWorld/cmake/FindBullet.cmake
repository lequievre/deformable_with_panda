# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# Institut Pascal UMR6602
# laurent.lequievre@uca.fr

set(PRINT_INFOS 1)

set(BULLET_ROOT "/home/laurent/git_projects/bullet3")

message(STATUS "BULLET_ROOT = ${BULLET_ROOT}")

# locate FindPackageHandleStandardArgs.cmake -> /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-3.16/Modules")

message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

macro(_FIND_BULLET_LIBRARY _var)
  find_library(${_var}
     NAMES
        ${ARGN}
     HINTS
        ${BULLET_ROOT}/build_cmake/src/BulletDynamics
        ${BULLET_ROOT}/build_cmake/src/BulletCollision
        ${BULLET_ROOT}/build_cmake/src/LinearMath
        ${BULLET_ROOT}/build_cmake/src/BulletSoftBody
     PATH_SUFFIXES lib
  )
  mark_as_advanced(${_var})
endmacro()

macro(_BULLET_APPEND_LIBRARIES _list _release)
  set(_debug ${_release}_DEBUG)
  if(${_debug})
    set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
  else()
    set(${_list} ${${_list}} ${${_release}})
  endif()
endmacro()

find_path(BULLET_INCLUDE_DIR NAMES btBulletCollisionCommon.h
  HINTS
    ${BULLET_ROOT}/include
    ${BULLET_ROOT}/src
  PATH_SUFFIXES bullet
)

# Find the libraries

_FIND_BULLET_LIBRARY(BULLET_DYNAMICS_LIBRARY        BulletDynamics)
_FIND_BULLET_LIBRARY(BULLET_DYNAMICS_LIBRARY_DEBUG  BulletDynamics_Debug BulletDynamics_d)
_FIND_BULLET_LIBRARY(BULLET_COLLISION_LIBRARY       BulletCollision)
_FIND_BULLET_LIBRARY(BULLET_COLLISION_LIBRARY_DEBUG BulletCollision_Debug BulletCollision_d)
_FIND_BULLET_LIBRARY(BULLET_MATH_LIBRARY            BulletMath LinearMath)
_FIND_BULLET_LIBRARY(BULLET_MATH_LIBRARY_DEBUG      BulletMath_Debug BulletMath_d LinearMath_Debug LinearMath_d)
_FIND_BULLET_LIBRARY(BULLET_SOFTBODY_LIBRARY        BulletSoftBody)
_FIND_BULLET_LIBRARY(BULLET_SOFTBODY_LIBRARY_DEBUG  BulletSoftBody_Debug BulletSoftBody_d)


include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Bullet DEFAULT_MSG
    BULLET_DYNAMICS_LIBRARY BULLET_COLLISION_LIBRARY BULLET_MATH_LIBRARY
    BULLET_SOFTBODY_LIBRARY BULLET_INCLUDE_DIR)

set(BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
if(BULLET_FOUND)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_DYNAMICS_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_COLLISION_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_MATH_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_SOFTBODY_LIBRARY)
endif()

if(PRINT_INFOS)
  if(BULLET_FOUND)
    message(STATUS "BULLET_FOUND = ${BULLET_FOUND}")
    message(STATUS "BULLET_INCLUDE_DIR = ${BULLET_INCLUDE_DIR}")
    message(STATUS "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
  else()
    message(STATUS "BULLET NOT FOUND !!")
  endif()
endif()



