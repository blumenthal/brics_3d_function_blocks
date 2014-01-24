CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(brics_3d_function_blocks)

# Compiler and flags required to comile microblx
#SET(CMAKE_CXX_COMPILER /usr/bin/clang++ CACHE PATH "Configure the compiler.")
# or call with: 
# cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/clang++

#ADD_DEFINITIONS(-DBRICS_XERCES_ENABLE)
SET(CMAKE_BUILD_TYPE DEBUG) # optional
SET(CMAKE_CXX_FLAGS "-Wall -Werror -fvisibility=hidden")
#SET(CMAKE_SHARED_LINKER_FLAGS -g -ggdb -shared)

#CFLAGS=-Wall -Werror -g -ggdb -fvisibility=hidden
#CFLAGS_SHARED=-g -ggdb -shared



# Common output filders bin and lib
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")

ADD_SUBDIRECTORY(./testblock)


