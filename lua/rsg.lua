-- ******************************************************************************
-- * BRICS_3D - 3D Perception and Modeling Library
-- * Copyright (c) 2014, KU Leuven
-- *
-- * Author: Sebastian Blumenthal
-- *
-- *
-- * This software is published under a dual-license: GNU Lesser General Public
-- * License LGPL 2.1 and Modified BSD license. The dual-license implies that
-- * users of this code may choose which terms they prefer.
-- *
-- * This program is distributed in the hope that it will be useful,
-- * but WITHOUT ANY WARRANTY; without even the implied warranty of
-- * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- * GNU Lesser General Public License LGPL and the BSD license for
-- * more details.
-- *
-- ******************************************************************************

local M={} 

local ffi = require("ffi")
local ubx = require("ubx")

local ubx_path = os.getenv("MICROBLX_DIR") .. "/" -- NOTE: this requires the MICROBLX_DIR environemtn variable to be set
local fbx_path = os.getenv("BRICS_3D_FUNCTION_BLOCKS_DIR") .. "/"  -- NOTE: this requires the BRICS_3D_FUNCTION_BLOCKS_DIR environemtn variable to be set (typically done in env.sh)

local ni=ubx.node_create("world_model_node_tmp")  -- better: load and parse ffi without ubx
ubx.load_module(ni, fbx_path .. "lib/rsg_types.so")
--world_model = ffi.load(fbx_path .. "lib/rsg_types.so")

-- Loading of library that contains to the worldModelLuaBinding  
local world_model = ffi.load(fbx_path .. "lib/world_model.so")

-- Mapping to the worldModelLuaBinding   
ffi.cdef[[
typedef struct WorldModel WorldModel;
typedef struct rsg_wm_handle rsg_wm_handle;
 
WorldModel* WorldModel_WorldModel();
void WorldModel__gc(WorldModel *);

rsg_wm_handle* WorldModel_getHandle(WorldModel *);
]]

-- Mimic the brics_3d::WordlModel with a lua version (mt) of that class
local mt = {}
mt.__index = mt

--- Retrive a "handle" than can pa passes to a config file of a function block.
function mt.getHandle(self, ...)
	return world_model.WorldModel_getHandle(self.super, ...)
end

--- Constructor for a new instance of a world model.
-- This will be used within a function blocks for instance. 
-- @return Table/class instance of a world model
function M.WorldModel(...)
	local self = {super = world_model.WorldModel_WorldModel(...)}
	ffi.gc(self.super, world_model.WorldModel__gc)
return setmetatable(self, mt)
end

--myWm = M.WorldModel()
--print(myWm)
--print(myWm:getHandle().wm)

return M