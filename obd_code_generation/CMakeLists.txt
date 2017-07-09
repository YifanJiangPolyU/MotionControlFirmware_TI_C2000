cmake_minimum_required(VERSION 2.8.3)
project(obd_code_generation)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
   COMPONENTS
)

## System dependencies are found with CMake's conventions
##find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS
       include
       ${catkin_INCLUDE_DIRS}
   LIBRARIES
       ${PROJECT_NAME}
   CATKIN_DEPENDS
   DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(
#   ${PROJECT_NAME}
#   src/serialHelper.c
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(TPTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(
    obd_code_generation
    src/obd_code_generation.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries(
   obd_code_generation
   ${catkin_LIBRARIES}
 )
