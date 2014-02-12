/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

/**
 * @file octreefilter
 *
 * A function block to to sub-sample a point cloud based on an octree voxelization.
 *
 */

#define DEBUG 1
#define BRICS_MICROBLX_ENABLE

/* std includes */
#include <iostream>
#include <cstring>

/* microblx includes */
#include "ubx.h"

/* microblx type for the robot scene graph */
#include "../types/rsg/types/rsg_types.h"
#include "brics_3d/util/UbxTypecaster.h"

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>

#include <brics_3d/core/HomogeneousMatrix44.h> // concrete type
#include <brics_3d/core/PointCloud3D.h>		// concrete type
#include <brics_3d/algorithm/filtering/Octree.h>



using namespace std;
using brics_3d::Logger;

brics_3d::WorldModel* wmHandle = 0;
brics_3d::rsg::DotGraphGenerator* wmPrinter = 0;

brics_3d::Octree* filter = 0;
brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr center;

double octreeCellSize;


/* function block meta-data
 * used by higher level functions.
 */
char octreefilter_meta[] =
	"{ doc='A function block to to sub-sample a point cloud based on an octree voxelization."
	"  license='BSD',"
	"  real-time=false,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t octreefilter_config[] = {
	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
	{ .name="referenceFrameId", .type_name = "unsigned int" },
	{ .name="octreeCellSize", .type_name = "double" },
	{ NULL },
};

ubx_port_t octreefilter_ports[] = {
	{ .name="inputDataIds", .attrs=PORT_DIR_IN, .in_type_name="struct rsg_ids"},
	{ .name="outputDataIds", .attrs=PORT_DIR_OUT, .out_type_name="struct rsg_ids"},
	{ NULL },
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
def_read_fun(read_rsg_ids, rsg_ids)
def_write_fun(write_rsg_ids, rsg_ids)

static int octreefilter_init(ubx_block_t *c)
{
	LOG(INFO) << "octreefilter: initializing: " << c->name;

//	wmHandle = brics_3d::WorldModel::microBlxWmHandle;
	unsigned int clen;
	rsg_wm_handle tmpWmHandle =  *((rsg_wm_handle*) ubx_config_get_data_ptr(c, "wm_handle", &clen));
	assert(clen != 0);
	wmHandle = reinterpret_cast<brics_3d::WorldModel*>(tmpWmHandle.wm); // We know that this pointer stores the world model type
	if(wmHandle == 0) {
		LOG(FATAL) << "ROIFilter: World modle handle could not be initialized.";
		return -1;
	}

	wmPrinter = new brics_3d::rsg::DotGraphGenerator();

	/* init algorithm */
	filter = new brics_3d::Octree();

	return 0;
}


static void octreefilter_cleanup(ubx_block_t *c)
{
	if (wmPrinter != 0) {
		delete wmPrinter;
		wmPrinter = 0;
	}
	if (filter != 0) {
		delete filter;
		filter = 0;
	}
	if (filter != 0) {
		delete filter;
		filter = 0;
	}

	LOG(INFO) << "octreefilter: cleaning up: " << c->name;
}

static int octreefilter_start(ubx_block_t *c)
{
	LOG(INFO) << "octreefilter: starting. " << c->name;
	return 0; /* Ok */
}

static void octreefilter_step(ubx_block_t *c) {
	LOG(INFO) << "octreefilter: executing: " << c->name;

	/* Just print what the world model has to offer. */
	wmPrinter->reset();
	wmHandle->scene.executeGraphTraverser(wmPrinter, wmHandle->getRootNodeId());
	LOG(DEBUG) << "octreefilter: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

	/* read Id(s) from input port */
	std::vector<brics_3d::rsg::Id> inputDataIds;
	inputDataIds.clear();
	ubx_port_t* inputPort = ubx_port_get(c, "inputDataIds");
	rsg_ids recievedInputDataIs;
	recievedInputDataIs.numberOfIds = 0u;

	int ret = read_rsg_ids(inputPort, &recievedInputDataIs);
	if (ret < 1) {
		LOG(WARNING) << "octreefilter: No input IDs given.";
	}

	brics_3d::rsg::UbxTypecaster::convertIdsFromUbx(recievedInputDataIs, inputDataIds);
	if (inputDataIds.size() < 2) {
		LOG(ERROR) << "octreefilter: Not enough IDs specified. Expected 2 but it is: " << inputDataIds.size();
		return;
	}
    brics_3d::rsg::Id outputHookId = inputDataIds[0]; // First ID is always the output hook.

	/* prepare input (retrieve a proper point cloud) */
    brics_3d::rsg::Shape::ShapePtr inputShape;
    brics_3d::rsg::TimeStamp inputTime;
    brics_3d::rsg::Id pointCloudId = inputDataIds[1];
    wmHandle->scene.getGeometry(pointCloudId, inputShape, inputTime);// retrieve a point cloud
    brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr inputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
    inputPointCloudContainer = boost::dynamic_pointer_cast<brics_3d::rsg::PointCloud<brics_3d::PointCloud3D> >(inputShape);

    if(inputPointCloudContainer == 0) {
            LOG(ERROR) << "octreefilter: Input data at input data Id does not contain a point cloud.";
            return;
    }


	/* get and set config data */
//	unsigned int clen;
    octreeCellSize = 0.01;// *((double*) ubx_config_get_data_ptr(c, "octreeCellSize", &clen));
    filter->setVoxelSize(octreeCellSize);

    /* define where to store results */
	brics_3d::PointCloud3D::PointCloud3DPtr outputPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr outputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	outputPointCloudContainer->data = outputPointCloud;

	/* do computation */
	LOG(INFO) << "octreefilter: Computing for input with " << inputPointCloudContainer->data->getSize() << " points.";
    LOG(INFO) << " using setVoxelSize = " << filter->getVoxelSize();
    filter->filter(inputPointCloudContainer->data.get(), outputPointCloudContainer->data.get());
    LOG(INFO) << "octreefilter: Computing done with output of " << outputPointCloudContainer->data->getSize() << " points.";

	/* prepare output */
    brics_3d::rsg::Id roiPointCloudId = 21;
    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
    attributes.push_back(brics_3d::rsg::Attribute("name","subsampled_point_cloud"));
    attributes.push_back(brics_3d::rsg::Attribute("origin","octreefilter"));
	wmHandle->scene.addGeometricNode(outputHookId, roiPointCloudId, attributes, outputPointCloudContainer, wmHandle->now());

//	brics_3d::rsg::Box::BoxPtr someBox(new brics_3d::rsg::Box(2, 3, 4));
//    attributes.clear();
//    attributes.push_back(brics_3d::rsg::Attribute("name","some_box"));
//	wmHandle->scene.addGeometricNode(outputHookId, roiPointCloudId, attributes, someBox, wmHandle->now(), true);



	/* store what we did to the world model in the output vector */
	std::vector<brics_3d::rsg::Id> output;
	output.clear();
	output.push_back(outputHookId); // We feed forward the output hook as first ID.
	output.push_back(roiPointCloudId); // This is what we added.

	/* push output to microblx */
	ubx_port_t* outputPort = ubx_port_get(c, "outputDataIds");
	rsg_ids toBeSendOutputDataIs;
	toBeSendOutputDataIs.numberOfIds = 0u;
	brics_3d::rsg::UbxTypecaster::convertIdsToUbx(output, toBeSendOutputDataIs);
	write_rsg_ids(outputPort, &toBeSendOutputDataIs);

}


/* put everything together */
ubx_block_t octreefilter_comp = {
	.name = "octreefilter/octreefilter",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = octreefilter_meta,
	.configs = octreefilter_config,
	.ports = octreefilter_ports,

	/* ops */
	.init = octreefilter_init,
	.start = octreefilter_start,
	.step = octreefilter_step,
	.cleanup = octreefilter_cleanup,
};

static int octreefilter_init(ubx_node_info_t* ni)
{
	LOG(DEBUG) << "octreefilter_init(ubx_node_info_t* ni)";
	return ubx_block_register(ni, &octreefilter_comp);
}

static void octreefilter_cleanup(ubx_node_info_t *ni)
{
	LOG(DEBUG) << "octreefilter_cleanup(ubx_node_info_t *ni)";
	ubx_block_unregister(ni, "octreefilter/octreefilter");
}

UBX_MODULE_INIT(octreefilter_init)
UBX_MODULE_CLEANUP(octreefilter_cleanup)
