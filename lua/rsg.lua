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

local ubx_path = os.getenv("UBX_ROOT") .. "/" -- NOTE: this requires the MICROBLX_DIR environemtn variable to be set
local fbx_path = os.getenv("FBX_MODULES") .. "/"  -- NOTE: this requires the BRICS_3D_FUNCTION_BLOCKS_DIR environemtn variable to be set (typically done in env.sh)

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
WorldModel* WorldModel_WorldModelWithId(const char* idAsString);
void WorldModel__gc(WorldModel *);

rsg_wm_handle* WorldModel_getHandle(WorldModel *);
const char* WorldModel_getRootId(WorldModel *);
bool WorldModel_addNodeAttribute(WorldModel *, const char* idAsString, const char* key, const char* value);
bool WorldModel_setLogLevel(WorldModel *, int logLevel); 
]]

-- Mimic the brics_3d::WordlModel with a lua version (mt) of that class
local mt = {}
mt.__index = mt

--- Retrive a "handle" than can pa passes to a config file of a function block.
function mt.getHandle(self, ...)
	return world_model.WorldModel_getHandle(self.super, ...)
end

--- Get the root id in hhhhhhhh-hhhh-hhhh-hhhh-hhhhhhhhhhhh format.
function mt.getRootId(self, ...)
  return ffi.string(world_model.WorldModel_getRootId(self.super, ...))
end

--- Add an attribute-value pair to a node, indentified by its Id.
-- @param id A UUID in hhhhhhhh-hhhh-hhhh-hhhh-hhhhhhhhhhhh format.
-- @param key Key as a string
-- @param value values as a string
-- @return True on success. 
function mt.addNodeAttribute(self, id, key, value)
--  print (id .. " " .. key .. " " .. value)
  return world_model.WorldModel_addNodeAttribute(self.super, id, key, value);
end

--- Constructor for a new instance of a world model.
-- This will be used within a function blocks for instance. 
-- @return Table/class instance of a world model
function M.WorldModel(...)
	local self = {super = world_model.WorldModel_WorldModel(...)}
	ffi.gc(self.super, world_model.WorldModel__gc)
return setmetatable(self, mt)
end

--- Constructor for a new instance of a world model with a predifined root Id for the world model agent.
-- This will be used within a function blocks for instance.
-- @param id A UUID in hhhhhhhh-hhhh-hhhh-hhhh-hhhhhhhhhhhh format. Invalid id will casse a fallbach to 
--            the default behaviour as defined by the M.WorldModel(...) constructor.            
-- @return Table/class instance of a world model
function M.WorldModelWithId(id)
  local self = {super = world_model.WorldModel_WorldModelWithId(id)}
  ffi.gc(self.super, world_model.WorldModel__gc)
return setmetatable(self, mt)
end

function mt.setLogLevel(self, logLevel)
--  print (id .. " " .. key .. " " .. value)
  return world_model.WorldModel_setLogLevel(self.super, logLevel);
end

--myWm = M.WorldModel()
--print(myWm)
--print(myWm:getHandle().wm)

return M