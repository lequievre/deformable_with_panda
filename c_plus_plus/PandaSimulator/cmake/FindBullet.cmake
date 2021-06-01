# Laurent LEQUIEVRE
# Research Engineer, CNRS (France)
# Institut Pascal UMR6602
# laurent.lequievre@uca.fr

set(PRINT_INFOS 1)

set(BULLET_ROOT "/home/laurent/git_projects/bullet3")

message(STATUS "FindBullet -> BULLET_ROOT = ${BULLET_ROOT}")

# locate FindPackageHandleStandardArgs.cmake -> /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-3.16/Modules")

message(STATUS "FindBullet -> CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

macro(_FIND_BULLET_LIBRARY _var)
  find_library(${_var}
     NAMES
        ${ARGN}
     HINTS
        ${BULLET_ROOT}/build_cmake/src/BulletDynamics
        ${BULLET_ROOT}/build_cmake/src/BulletCollision
        ${BULLET_ROOT}/build_cmake/src/LinearMath
        ${BULLET_ROOT}/build_cmake/src/BulletSoftBody
        ${BULLET_ROOT}/build_cmake/examples/OpenGLWindow
        ${BULLET_ROOT}/build_cmake/src/Bullet3Common
        ${BULLET_ROOT}/build_cmake/examples/ExampleBrowser
        ${BULLET_ROOT}/build_cmake/src/Bullet3Serialize/Bullet2FileLoader
        ${BULLET_ROOT}/build_cmake/src/Bullet3Geometry
        ${BULLET_ROOT}/build_cmake/src/Bullet3OpenCL
        ${BULLET_ROOT}/build_cmake/src/Bullet3Collision
        ${BULLET_ROOT}/build_cmake/src/Bullet3Dynamics
        ${BULLET_ROOT}/build_cmake/Extras/Serialize/BulletFileLoader
        ${BULLET_ROOT}/build_cmake/src/BulletInverseDynamics
        ${BULLET_ROOT}/build_cmake/Extras/InverseDynamics
        ${BULLET_ROOT}/build_cmake/Extras/BulletRobotics
        ${BULLET_ROOT}/build_cmake/Extras/BulletRoboticsGUI
        ${BULLET_ROOT}/build_cmake/Extras/Serialize/BulletWorldImporter
        ${BULLET_ROOT}/build_cmake/Extras/Serialize/BulletXmlWorldImporter
        ${BULLET_ROOT}/build_cmake/examples/ThirdPartyLibs/BussIK
        ${BULLET_ROOT}/build_cmake/examples/ThirdPartyLibs/clsocket
        ${BULLET_ROOT}/build_cmake/Extras/ConvexDecomposition
        ${BULLET_ROOT}/build_cmake/Extras/GIMPACTUtils
        ${BULLET_ROOT}/build_cmake/examples/ThirdPartyLibs/Gwen
        ${BULLET_ROOT}/build_cmake/Extras/HACD
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

 _FIND_BULLET_LIBRARY(BULLET_2_FILE_LOADER_LIBRARY            Bullet2FileLoader)
 _FIND_BULLET_LIBRARY(BULLET_3_COLLISION_LIBRARY              Bullet3Collision)
 _FIND_BULLET_LIBRARY(BULLET_3_COMMON_LIBRARY                 Bullet3Common)
 _FIND_BULLET_LIBRARY(BULLET_3_DYNAMICS_LIBRARY               Bullet3Dynamics)
 _FIND_BULLET_LIBRARY(BULLET_3_GEOMETRY_LIBRARY               Bullet3Geometry)
 _FIND_BULLET_LIBRARY(BULLET_3_OPENCL_CLEW_LIBRARY            Bullet3OpenCL_clew)
 _FIND_BULLET_LIBRARY(BULLET_COLLISION_LIBRARY                BulletCollision)
 #_FIND_BULLET_LIBRARY(BULLET_COLLISION_LIBRARY_DEBUG    BulletCollision_Debug BulletCollision_d)
 _FIND_BULLET_LIBRARY(BULLET_DYNAMICS_LIBRARY                 BulletDynamics)
 #_FIND_BULLET_LIBRARY(BULLET_DYNAMICS_LIBRARY_DEBUG     BulletDynamics_Debug BulletDynamics_d)
 _FIND_BULLET_LIBRARY(BULLET_EXAMPLE_BROWSER_LIBRARY          BulletExampleBrowserLib)
 _FIND_BULLET_LIBRARY(BULLET_FILE_LOADER_LIBRARY              BulletFileLoader)
 _FIND_BULLET_LIBRARY(BULLET_INVERSE_DYNAMICS_LIBRARY         BulletInverseDynamics)
 _FIND_BULLET_LIBRARY(BULLET_INVERSE_DYNAMICS_UTILS_LIBRARY   BulletInverseDynamicsUtils)
 _FIND_BULLET_LIBRARY(BULLET_ROBOTICS_LIBRARY                 BulletRobotics)
 _FIND_BULLET_LIBRARY(BULLET_ROBOTICS_GUI_LIBRARY             BulletRoboticsGUI)
 _FIND_BULLET_LIBRARY(BULLET_SOFTBODY_LIBRARY                 BulletSoftBody)
 #_FIND_BULLET_LIBRARY(BULLET_SOFTBODY_LIBRARY_DEBUG           BulletSoftBody_Debug BulletSoftBody_d)
 _FIND_BULLET_LIBRARY(BULLET_WORLD_IMPORTER_LIBRARY           BulletWorldImporter)
 _FIND_BULLET_LIBRARY(BULLET_XML_WORLD_IMPORTER_LIBRARY       BulletXmlWorldImporter)
 _FIND_BULLET_LIBRARY(BUSS_IK_LIBRARY                         BussIK)
 _FIND_BULLET_LIBRARY(CL_SOCKET_LIBRARY                       clsocket)
 _FIND_BULLET_LIBRARY(CONVEX_DECOMPOSITION_LIBRARY            ConvexDecomposition)
 _FIND_BULLET_LIBRARY(GIMPACT_Utils_LIBRARY                   GIMPACTUtils)
 _FIND_BULLET_LIBRARY(GWEN_LIBRARY                            gwen)
 _FIND_BULLET_LIBRARY(HACD_LIBRARY                            HACD)
 _FIND_BULLET_LIBRARY(LINEAR_MATH_LIBRARY                     LinearMath)
 #_FIND_BULLET_LIBRARY(LINEAR_MATH_LIBRARY_DEBUG     LinearMath_Debug LinearMath_d)
 _FIND_BULLET_LIBRARY(OPENGL_WINDOW_LIBRARY                   OpenGLWindow)

include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Bullet DEFAULT_MSG
    BULLET_2_FILE_LOADER_LIBRARY BULLET_3_COLLISION_LIBRARY BULLET_3_COMMON_LIBRARY
BULLET_3_DYNAMICS_LIBRARY BULLET_3_GEOMETRY_LIBRARY BULLET_3_OPENCL_CLEW_LIBRARY
BULLET_COLLISION_LIBRARY BULLET_DYNAMICS_LIBRARY BULLET_EXAMPLE_BROWSER_LIBRARY
BULLET_FILE_LOADER_LIBRARY BULLET_INVERSE_DYNAMICS_LIBRARY BULLET_INVERSE_DYNAMICS_UTILS_LIBRARY
BULLET_ROBOTICS_LIBRARY BULLET_ROBOTICS_GUI_LIBRARY BULLET_SOFTBODY_LIBRARY
BULLET_WORLD_IMPORTER_LIBRARY BULLET_XML_WORLD_IMPORTER_LIBRARY BUSS_IK_LIBRARY
CL_SOCKET_LIBRARY CONVEX_DECOMPOSITION_LIBRARY GIMPACT_Utils_LIBRARY
GWEN_LIBRARY HACD_LIBRARY LINEAR_MATH_LIBRARY OPENGL_WINDOW_LIBRARY BULLET_INCLUDE_DIR)

set(BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
if(BULLET_FOUND)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_ROBOTICS_GUI_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_EXAMPLE_BROWSER_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_ROBOTICS_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_FILE_LOADER_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_WORLD_IMPORTER_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_SOFTBODY_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_DYNAMICS_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_COLLISION_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_INVERSE_DYNAMICS_UTILS_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_INVERSE_DYNAMICS_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES LINEAR_MATH_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES OPENGL_WINDOW_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES GWEN_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BUSS_IK_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLET_LIBRARIES BULLET_3_COMMON_LIBRARY) 
endif()

if(PRINT_INFOS)
  if(BULLET_FOUND)
    message(STATUS "FindBullet -> BULLET_FOUND = ${BULLET_FOUND}")
    message(STATUS "FindBullet -> BULLET_INCLUDE_DIR = ${BULLET_INCLUDE_DIR}")
    message(STATUS "FindBullet -> BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
  else()
    message(STATUS "FindBullet -> BULLET NOT FOUND !!")
  endif()
endif()



