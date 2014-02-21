-- NOTE: You have to start this scrip relative to the microblox root folder.

ffi = require("ffi")
ubx = require("ubx")

ni=ubx.node_create("function_block_node") -- create a new handle that holds all function blocks

ubx_path = os.getenv("MICROBLX_DIR") .. "/" -- NOTE: this requires the MICROBLX_DIR environemtn variable to be set
fbx_path = os.getenv("BRICS_3D_FUNCTION_BLOCKS_DIR") .. "/"  -- NOTE: this requires the BRICS_3D_FUNCTION_BLOCKS_DIR environemtn variable to be set (typically done in env.sh)

-- std ubx modules
ubx.load_module(ni, ubx_path .. "std_types/stdtypes/stdtypes.so")
ubx.load_module(ni, ubx_path .. "std_types/testtypes/testtypes.so")
ubx.load_module(ni, ubx_path .. "std_blocks/lfds_buffers/lfds_cyclic.so")
ubx.load_module(ni, ubx_path .. "std_blocks/webif/webif.so")
ubx.load_module(ni, ubx_path .. "std_blocks/ptrig/ptrig.so")
-- fbx modules
ubx.load_module(ni, fbx_path .. "lib/rsg_types.so")
ubx.load_module(ni, fbx_path .. "lib/roifilter.so")
ubx.load_module(ni, fbx_path .. "lib/octreefilter.so")
ubx.load_module(ni, fbx_path .. "lib/pointcloudloader.so")

ubx.ffi_load_types(ni)

-- wm handle
world_model = ffi.load(fbx_path .. "lib/world_model.so")
 
ffi.cdef[[
typedef struct WorldModel WorldModel;
typedef struct rsg_wm_handle rsg_wm_handle;
 
WorldModel* WorldModel_WorldModel();
void WorldModel__gc(WorldModel *);

rsg_wm_handle* WorldModel_getHandle(WorldModel *);
]]

-- mimic the brics_3d::WordlModel with a lua version of that class
local mt = {}
mt.__index = mt
function mt.getHandle(self, ...)
	return world_model.WorldModel_getHandle(self.super, ...)
end
 
function WorldModel(...)
	local self = {super = world_model.WorldModel_WorldModel(...)}
	ffi.gc(self.super, world_model.WorldModel__gc)
return setmetatable(self, mt)
end

myWm = WorldModel()
rsg_wm_handle = myWm:getHandle()
print(myWm)
print(myWm:getHandle().wm)

-- web interface for introspeciton
print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })
print("running webif init", ubx.block_init(webif1))
print("running webif start", ubx.block_start(webif1))


-- create function blocks
print("creating instance of 'roifilter/roifilter'")
roifilter1=ubx.block_create(ni, "roifilter/roifilter", "roifilter1", { wm_handle={wm = myWm:getHandle().wm}, max_x="2"}) 
ubx.block_init(roifilter1)
ubx.block_start(roifilter1)

print("creating instance of 'pointcloudloader/pointcloudloader'")
pointcloudloader1=ubx.block_create(ni, "pointcloudloader/pointcloudloader", "pointcloudloader1", { wm_handle={wm = myWm:getHandle().wm}}) 
ubx.block_init(pointcloudloader1)
ubx.block_start(pointcloudloader1)

-- a default tigger to fire the function blocks
print("creating instance of 'std_triggers/ptrig'")
ptrig1=ubx.block_create(ni, "std_triggers/ptrig", "ptrig1",
			{
			   period = {sec=1, usec=000000 },
			   sched_policy="SCHED_OTHER", sched_priority=0,
			   trig_blocks={ 
			   		{ b=pointcloudloader1, num_steps=1, measure=0 },
			    	{ b=roifilter1, num_steps=1, measure=0 }		   				
			   } } )
print("running ptrig1 init", ubx.block_init(ptrig1))

-- connections
--ubx.conn_uni(pointcloudloader1, "outputDataIds", roifilter1, "inputDataIds", iblock_type, iblock_config, dont_start)
--assert(ubx.conn_lfds_cyclic(pointcloudloader1, "outputDataIds", roifilter1, "inputDataIds"))

print("creating instance of 'lfds_buffers/cyclic'")
fifo1=ubx.block_create(ni, "lfds_buffers/cyclic", "fifo1", {buffer_len=4, type_name="struct rsg_ids"})
ubx.block_init(fifo1)
ubx.block_start(fifo1)

pointcloud1out=ubx.port_get(pointcloudloader1, "outputDataIds")
roifilter1in=ubx.port_get(roifilter1, "inputDataIds")

ubx.port_connect_out(pointcloud1out, fifo1)
ubx.port_connect_in(roifilter1in, fifo1)

-- start!
--roifilter1.step()
--local roifilter1Table = ubx.block_totab(roifilter1)
--roifilter1Table.step()
print("running ptrig1 init", ubx.block_start(ptrig1))


-- wait for user input for exit
io.read()

print("Shutting down.")
ubx.node_cleanup(ni)
os.exit(1)
