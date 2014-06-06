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
 * @file pointcloudloader
 *
 * A function block to load a point cloud from a file.
 *
 */

#define DEBUG 1
#define BRICS_MICROBLX_ENABLE

/* std includes */
#include <iostream>
#include <cstring>
#include <string>

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

#ifdef ENABLE_OSG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
    // Member declaration raising the warning.
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#pragma clang diagnostic pop
#endif



using namespace std;
using brics_3d::Logger;

brics_3d::WorldModel* wmHandle = 0;
brics_3d::rsg::DotGraphGenerator* wmPrinter = 0;

brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr center;

double octreeCellSize;


/* function block meta-data
 * used by higher level functions.
 */
char pointcloudloader_meta[] =
	"{ doc='A function block to load a point cloud from a file.',"
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
ubx_config_t pointcloudloader_config[] = {
	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
//	{ .name="enable_osg_visualization", .type_name = "boolean", .doc="Weather to activete the OSGViewer or not."},
	{ NULL },
};

ubx_port_t pointcloudloader_ports[] = {
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

static int pointcloudloader_init(ubx_block_t *c)
{
	LOG(INFO) << "pointcloudloader: initializing: " << c->name;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

//	wmHandle = brics_3d::WorldModel::microBlxWmHandle;
	unsigned int clen;
	rsg_wm_handle tmpWmHandle =  *((rsg_wm_handle*) ubx_config_get_data_ptr(c, "wm_handle", &clen));
	assert(clen != 0);
	wmHandle = reinterpret_cast<brics_3d::WorldModel*>(tmpWmHandle.wm); // We know that this pointer stores the world model type
	if(wmHandle == 0) {
		LOG(FATAL) << "PointCloudLoader: World model handle could not be initialized.";
		return -1;
	}

#ifdef ENABLE_OSG
	brics_3d::rsg::OSGVisualizer* geometryVizualizer = new brics_3d::rsg::OSGVisualizer(); // Create the visualizer.
	brics_3d::rsg::VisualizationConfiguration osgConfiguration; // _Optional_ configuration file.
	osgConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	osgConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	osgConfiguration.abbreviateIds = true;       // Vizualize only the lower 2 bytes of an Id iff true.
	geometryVizualizer->setConfig(osgConfiguration);
	wmHandle->scene.attachUpdateObserver(geometryVizualizer); // Enable 3D visualization
#endif

	wmPrinter = new brics_3d::rsg::DotGraphGenerator();

	return 0;
}


static void pointcloudloader_cleanup(ubx_block_t *c)
{
	if (wmPrinter != 0) {
		delete wmPrinter;
		wmPrinter = 0;
	}

	LOG(INFO) << "pointcloudloader: cleaning up: " << c->name;
}

static int pointcloudloader_start(ubx_block_t *c)
{
	LOG(INFO) << "pointcloudloader: starting. " << c->name;
	return 0; /* Ok */
}

static void pointcloudloader_step(ubx_block_t *c) {
	LOG(INFO) << "pointcloudloader: executing: " << c->name;

	/* Just print what the world model has to offer. */
	wmPrinter->reset();
	wmHandle->scene.executeGraphTraverser(wmPrinter, wmHandle->getRootNodeId());
	LOG(DEBUG) << "pointcloudloader: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

	/* read Id(s) from input port */
	std::vector<brics_3d::rsg::Id> inputDataIds;
	inputDataIds.clear();
	ubx_port_t* inputPort = ubx_port_get(c, "inputDataIds");
	rsg_ids recievedInputDataIs;
	recievedInputDataIs.numberOfIds = 0u;

	int ret = read_rsg_ids(inputPort, &recievedInputDataIs);
	if (ret < 1) {
		LOG(WARNING) << "pointcloudloader: No input IDs given.";
	}

	brics_3d::rsg::Id outputHookId;
	brics_3d::rsg::UbxTypecaster::convertIdsFromUbx(recievedInputDataIs, inputDataIds);
	if (inputDataIds.size() < 1) {
		LOG(WARNING) << "pointcloudloader: Not enough IDs specified. Expected 1 but it is: " << inputDataIds.size() << std::endl << "Using root instead.";
		outputHookId = wmHandle->getRootNodeId();
	} else {
		outputHookId = inputDataIds[0]; // First ID is always the output hook.
	}




	/* get and set config data */
//	unsigned int clen;

    /* define where to store results */
	brics_3d::PointCloud3D::PointCloud3DPtr outputPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr outputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	outputPointCloudContainer->data = outputPointCloud;

	/* do computation */
//	std::string dataSetFolder = BRICS_MODELS_DIR;
	std::string dataSetFolder = "/home/sblume/workspace/brics_3d/brics_3d/examples/test_data/3d_models";
	std::string dataSet = dataSetFolder + "/bunny000.txt";
	outputPointCloudContainer->data->readFromTxtFile(dataSet);
    LOG(INFO) << "pointcloudloader: Loading done with output of " << outputPointCloudContainer->data->getSize() << " points.";

	/* prepare output */
    brics_3d::rsg::Id roiPointCloudId = 21;
    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
    attributes.push_back(brics_3d::rsg::Attribute("name","point_cloud"));
    attributes.push_back(brics_3d::rsg::Attribute("origin","pointcloudloader"));
	wmHandle->scene.addGeometricNode(outputHookId, roiPointCloudId, attributes, outputPointCloudContainer, wmHandle->now());


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
ubx_block_t pointcloudloader_comp = {
	.name = "pointcloudloader/pointcloudloader",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = pointcloudloader_meta,
	.configs = pointcloudloader_config,
	.ports = pointcloudloader_ports,

	/* ops */
	.init = pointcloudloader_init,
	.start = pointcloudloader_start,
	.step = pointcloudloader_step,
	.cleanup = pointcloudloader_cleanup,
};

static int pointcloudloader_init(ubx_node_info_t* ni)
{
	LOG(DEBUG) << "pointcloudloader_init(ubx_node_info_t* ni)";
	return ubx_block_register(ni, &pointcloudloader_comp);
}

static void pointcloudloader_cleanup(ubx_node_info_t *ni)
{
	LOG(DEBUG) << "pointcloudloader_cleanup(ubx_node_info_t *ni)";
	ubx_block_unregister(ni, "pointcloudloader/pointcloudloader");
}

UBX_MODULE_INIT(pointcloudloader_init)
UBX_MODULE_CLEANUP(pointcloudloader_cleanup)
