#!/bin/sh

source ./env.sh
cd $MICROBLX_DIR && luajit $BRICS_3D_FUNCTION_BLOCKS_DIR/lua/function_blocks_test.lua 
#cd $BRICS_3D_FUNCTION_BLOCKS_DIR # go back
