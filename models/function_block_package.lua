return pkg
{
  name="microblx_function_block",
  path="../",
      
  dependencies = {
    { name="rsg_types", type="cmake" },
  },
  
  
  blocks = {
    { name="function_block", file="/opt/src/sandbox/brics_3d_function_blocks/models/function_block.lua", src_dir="functionblock" },
  },
  
  libraries = {
    { name="functionblock", blocks={"function_block"} },
  },  
}
