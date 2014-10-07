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

OPTION(USE_OSG "Enable Open Scene Graph" OFF) #there are still some issues with the clang compiler
OPTION(USE_FBX "Enable Function Blocks" ON) # to avoid be abele to break cyclic dependencies.

ADD_SUBDIRECTORY(./types/rsg)
IF(USE_FBX)
    ADD_SUBDIRECTORY(./testblock)
    ADD_SUBDIRECTORY(./roifilter)
    ADD_SUBDIRECTORY(./octreefilter)
    IF(USE_OSG)
        ADD_SUBDIRECTORY(./pointcloudloader)
    ENDIF(USE_OSG)
ENDIF(USE_FBX)