/*
 * A demo C++ function block
 */

#define DEBUG 1

#include <iostream>
using namespace std;

#include "ubx.h"
#include <cstring>


/* function block meta-data
 * used by higher level functions.
 */
char testblock_meta[] =
	"{ doc='A testblock number generator function block',"
	"  license='LGPL',"
	"  real-time=true,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t testblock_config[] = {
	{ .name="test_conf", .type_name = "double" },
	{ NULL },
};


ubx_port_t testblock_ports[] = {
	{ .name="foo", .attrs=PORT_DIR_IN, .in_type_name="unsigned int" },
	{ .name="bar", .attrs=PORT_DIR_OUT, .out_type_name="unsigned int" },
	{ NULL },
};

static int testblock_init(ubx_block_t *c)
{
	cout << "testblock_init: hi from " << c->name << endl;
	return 0;
}


static void testblock_cleanup(ubx_block_t *c)
{
	cout << "testblock_cleanup: hi from " << c->name << endl;
}

static int testblock_start(ubx_block_t *c)
{
	cout << "testblock_start: hi from " << c->name << endl;
	return 0; /* Ok */
}

static void testblock_step(ubx_block_t *c) {
	cout << "testblock_step: hi from " << c->name << endl;
}


/* put everything together */
ubx_block_t testblock_comp = {
	.name = "testblock/testblock",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = testblock_meta,
	.configs = testblock_config,
	.ports = testblock_ports,

	/* ops */
	.init = testblock_init,
	.start = testblock_start,
	.step = testblock_step,
	.cleanup = testblock_cleanup,
};

static int testblock_init(ubx_node_info_t* ni)
{
	DBG(" ");
	return ubx_block_register(ni, &testblock_comp);
}

static void testblock_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	ubx_block_unregister(ni, "testblock/testblock");
}

UBX_MODULE_INIT(testblock_init)
UBX_MODULE_CLEANUP(testblock_cleanup)
