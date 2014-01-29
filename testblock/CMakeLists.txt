CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(testblock)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
FIND_PACKAGE(BRICS_3D REQUIRED)
FIND_PACKAGE(Microblx REQUIRED)

# add include directories (-I)
INCLUDE_DIRECTORIES(${BRICS_3D_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${MICROBLX_INCLUDE_DIR})

# add library directories (-L)
LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES})

# compile and link the executables
ADD_LIBRARY(testblock SHARED testblock.cpp) # -fPIC is already included
SET_TARGET_PROPERTIES(testblock PROPERTIES PREFIX "") # should be testblock.so and not libtestblock.so ... 
TARGET_LINK_LIBRARIES(testblock ${BRICS_3D_LIBRARIES} ${MICROBLX_LIBRARIES})


