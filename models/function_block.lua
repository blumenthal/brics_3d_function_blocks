-- This serves as a generic template for a function block.
-- For creation of individual blocks, replace all occurences of 
-- function_block or functionblock to the desired name

return block
{
      name="function_block",
      meta_data="",
      port_cache=true,

      configurations= {
		{ name="wm_handle", type_name = "struct rsg_wm_handle", doc="Handle to the world wodel instance. This parameter is mandatory."},
      },

      ports = {
		{ name="inputDataIds", in_type_name="struct rsg_ids", doc="Set of RSG IDs that refeer to the input data."}, 
		{ name="outputDataIds", out_type_name="struct rsg_ids", doc="Set of RSG IDs that refeer to the output data."},
      },
      
      operations = { start=true, stop=true, step=true },
      
      cpp=true
}
