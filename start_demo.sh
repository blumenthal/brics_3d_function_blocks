#!/bin/sh

#source ./env.sh
#cd $MICROBLX_DIR && luajit $BRICS_3D_FUNCTION_BLOCKS_DIR/lua/function_blocks_test.lua
#cd $MICROBLX_DIR && tools/ubx_launch -c $BRICS_3D_FUNCTION_BLOCKS_DIR/lua/function_blocks_test.usc -webif
#cd $BRICS_3D_FUNCTION_BLOCKS_DIR # go back


# pointers to the relveant modules (this has to be needed to be adopted for individual systms)
export UBX_MODULES=$UBX_ROOT
#export FBX_MODULES=$BRICS_3D_FUNCTION_BLOCKS_DIR

# set up some other environtment scripts
source $UBX_ROOT/env.sh
source $FBX_MODULES/env.sh

# Start the mircoblox system
exec $UBX_ROOT/tools/ubx_launch -webif 8888 -c lua/function_blocks_test.usc
