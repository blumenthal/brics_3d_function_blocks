CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(brics_3d_function_blocks)

# Compiler and flags required to comile microblx
#SET(CMAKE_CXX_COMPILER /usr/bin/clang++ CACHE PATH "Configure the compiler.")
# or call with: 
# cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/clang++

SET(CMAKE_BUILD_TYPE DEBUG) # optional
SET(CMAKE_CXX_FLAGS "-Wall -Werror -fvisibility=hidden")

# Common output filders bin and lib
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")

set(INSTALL_LIB_TYPES_DIR lib/ubx/types CACHE PATH "Installation directory for types libraries")
set(INSTALL_LIB_BLOCKS_DIR lib/ubx/blocks CACHE PATH "Installation directory for blocks libraries")
set(INSTALL_INCLUDE_DIR include/ubx CACHE PATH "Installation directory for header files (types)")
set(INSTALL_CMAKE_DIR  share/ubx/cmake CACHE PATH  "Installation directory for CMake files") # default ${DEF_INSTALL_CMAKE_DIR}
ADD_DEFINITIONS(-DFBX_MODELS_DIR="${CMAKE_HOME_DIRECTORY}/models") # folder for models (schemas)

OPTION(USE_OSG "Enable Open Scene Graph" OFF) #there are still some issues with the clang compiler
OPTION(USE_FBX "Enable Function Blocks" OFF) # to avoid be abele to break cyclic dependencies.

ADD_SUBDIRECTORY(./types/rsg)
IF(USE_FBX)
    ADD_SUBDIRECTORY(./testblock)
    ADD_SUBDIRECTORY(./roifilter)
    ADD_SUBDIRECTORY(./posehistory)
    ADD_SUBDIRECTORY(./poselist)
    ADD_SUBDIRECTORY(./nodesinarea)
    ADD_SUBDIRECTORY(./octreefilter)
    ADD_SUBDIRECTORY(./onattributechange)
    ADD_SUBDIRECTORY(./dotgenerator)
    IF(USE_OSG)
        ADD_SUBDIRECTORY(./pointcloudloader)
        #ADD_SUBDIRECTORY(./osgvisualizer) # != clang++
    ENDIF(USE_OSG)
    ADD_SUBDIRECTORY(./osmloader)
ENDIF(USE_FBX)