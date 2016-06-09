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
 * @file roifilter
 *
 * A function block to extract a Region Of Interest (ROI) from a point cloud.
 * Both have to be stored in the world model.
 *
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
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>

#include <brics_3d/core/HomogeneousMatrix44.h> // concrete type
#include <brics_3d/core/PointCloud3D.h>		// concrete type
#include <brics_3d/algorithm/filtering/BoxROIExtractor.h>

/***********FBX***************/

class RoiFilter: public brics_3d::rsg::IFunctionBlock {
public:

	RoiFilter(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "roiblock";
		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name;

		/* get and set config data */
		min_x = -0.5;
		max_x = 0.1;
		min_y = -0.5;
		max_y = 0.1;
		min_z = -0.5;
		max_z = 0.5;

		/* init algorithm */
	    this->filter = new brics_3d::BoxROIExtractor();
	    this->center = brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr(new brics_3d::HomogeneousMatrix44());
	    this->filter->setBoxOrigin(center);
	};

	~RoiFilter(){
		LOG(INFO) << name << ": Stopping block " << name;
		delete this->filter;
	};

	bool configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";
		return true;
	}

	bool execute(std::vector<brics_3d::rsg::Id>& inputDataIds, std::vector<brics_3d::rsg::Id>& outputDataIds) {
		LOG(INFO) << name << ": Executing block " << name << " with " << inputDataIds.size() << " ids as input.";

		if (inputDataIds.size() < 2) {
			LOG(ERROR) << "ROIFilter: Not enough IDs specified. Expected 2 but it is: " << inputDataIds.size();
			return false;
		}
	    brics_3d::rsg::Id outputHookId = inputDataIds[0]; // First ID is always the output hook.

		/* prepare input (retrieve a proper point cloud) */
	    brics_3d::rsg::Shape::ShapePtr inputShape;
	    brics_3d::rsg::TimeStamp inputTime;
	    brics_3d::rsg::Id pointCloudId = inputDataIds[1];
	    wm->scene.getGeometry(pointCloudId, inputShape, inputTime);// retrieve a point cloud
	    brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr inputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	    inputPointCloudContainer = boost::dynamic_pointer_cast<brics_3d::rsg::PointCloud<brics_3d::PointCloud3D> >(inputShape);

	    if(inputPointCloudContainer == 0) {
	            LOG(ERROR) << "ROIFilter: Input data at input data Id does not contain a point cloud.";
	            return false;
	    }


	    filter->setSizeX(fabs(min_x)); // set new dimensions
	    filter->setSizeY(fabs(min_y));
	    filter->setSizeZ(fabs(min_z));
	    double* transformMatrix = center->setRawData(); // set new center
	    transformMatrix[12] = max_x - ((max_x-min_x)/2.0);
	    transformMatrix[13] = max_y - ((max_y-min_y)/2.0);
	    transformMatrix[14] = max_z - ((max_z-min_z)/2.0);

	    /* define where to store results */
		brics_3d::PointCloud3D::PointCloud3DPtr outputPointCloud(new brics_3d::PointCloud3D());
		brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr outputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		outputPointCloudContainer->data = outputPointCloud;

		/* do computation */
		LOG(INFO) << "ROIFilter: Computing for input with " << inputPointCloudContainer->data->getSize() << " points.";
	    filter->filter(inputPointCloudContainer->data.get(), outputPointCloudContainer->data.get());
	    LOG(INFO) << "ROIFilter: Computing done with output of " << outputPointCloudContainer->data->getSize() << " points.";

		/* prepare output */
	    brics_3d::rsg::Id roiPointCloudId = 21;
	    std::vector<brics_3d::rsg::Attribute> attributes;
	    attributes.clear();
	    attributes.push_back(brics_3d::rsg::Attribute("name","roi_point_cloud"));
	    attributes.push_back(brics_3d::rsg::Attribute("origin","roifilter"));
		wm->scene.addGeometricNode(outputHookId, roiPointCloudId, attributes, outputPointCloudContainer, wm->now());

		/* store what we did to the world model in the output vector */
		outputDataIds.clear();
		outputDataIds.push_back(outputHookId); // We feed forward the output hook as first ID.
		outputDataIds.push_back(roiPointCloudId); // This is what we added.

		return true;
	}

	bool execute(std::string inputModel, std::string& outputModel) {
		LOG(ERROR) << "ROIFilter: model based io not supported.";
		return false;
	}

private:
	std::string name;

	/* Algorithm(s) */
	brics_3d::BoxROIExtractor* filter;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr center;

	/* Parameters */
	double min_x;
	double max_x;
	double min_y;
	double max_y;
	double min_z;
	double max_z;
};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(RoiFilter)
FUNCTION_BLOCK_DESTROY

/***********UBX***************/

UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)

using namespace std;
using brics_3d::Logger;

brics_3d::WorldModel* wmHandle = 0;
brics_3d::rsg::DotGraphGenerator* wmPrinter = 0;

brics_3d::BoxROIExtractor* filter = 0;
brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr center;
RoiFilter* roifilter;

double min_x;
double max_x;
double min_y;
double max_y;
double min_z;
double max_z;

/* function block meta-data
 * used by higher level functions.
 */
char roifilter_meta[] =
	"{ doc='A function block to extract a Region Of Interest (ROI) from a point cloud."
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
ubx_config_t roifilter_config[] = {
	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
	{ .name="referenceFrameId", .type_name = "unsigned int" },
	{ .name="min_x", .type_name = "double" },
	{ .name="max_x", .type_name = "double" },
	{ .name="min_y", .type_name = "double" },
	{ .name="max_y", .type_name = "double" },
	{ .name="min_z", .type_name = "double" },
	{ .name="max_z", .type_name = "double" },
	{ NULL },
};

ubx_port_t roifilter_ports[] = {
	{ .name="inputDataIds", .attrs=PORT_DIR_IN, .in_type_name="struct rsg_ids"}, //unsigned int" },
	{ .name="outputDataIds", .attrs=PORT_DIR_OUT, .out_type_name="struct rsg_ids"},
	{ NULL },
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
def_read_fun(read_rsg_ids, rsg_ids)
def_write_fun(write_rsg_ids, rsg_ids)

static int roifilter_init(ubx_block_t *c)
{
	LOG(INFO) << "ROIFilter: initializing: " << c->name;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

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
//    filter = new brics_3d::BoxROIExtractor();
//    center = brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr(new brics_3d::HomogeneousMatrix44());
//    filter->setBoxOrigin(center);
    roifilter = new RoiFilter(wmHandle);


	return 0;
}


static void roifilter_cleanup(ubx_block_t *c)
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
		delete roifilter;
		filter = 0;
	}

	LOG(INFO) << "ROIFilter: cleaning up: " << c->name;
}

static int roifilter_start(ubx_block_t *c)
{
	LOG(INFO) << "ROIFilter: starting. " << c->name;
	return 0; /* Ok */
}

static void roifilter_step(ubx_block_t *c) {
	LOG(INFO) << "ROIFilter: executing: " << c->name;

	/* Just print what the world model has to offer. */
	wmPrinter->reset();
	wmHandle->scene.executeGraphTraverser(wmPrinter, wmHandle->getRootNodeId());
	LOG(DEBUG) << "ROIFilter: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

	/* read Id(s) from input port */
	std::vector<brics_3d::rsg::Id> inputDataIds;
	inputDataIds.clear();
	ubx_port_t* inputPort = ubx_port_get(c, "inputDataIds");
	rsg_ids recievedInputDataIs;
	recievedInputDataIs.numberOfIds = 0u;

	int ret = read_rsg_ids(inputPort, &recievedInputDataIs);
	if (ret < 1) {
		LOG(WARNING) << "ROIFilter: No input IDs given.";
	}

	brics_3d::rsg::UbxTypecaster::convertIdsFromUbx(recievedInputDataIs, inputDataIds);
	std::vector<brics_3d::rsg::Id> output;
	roifilter->execute(inputDataIds, output);

#ifdef NEVER
	if (inputDataIds.size() < 2) {
		LOG(ERROR) << "ROIFilter: Not enough IDs specified. Expected 2 but it is: " << inputDataIds.size();
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
            LOG(ERROR) << "ROIFilter: Input data at input data Id does not contain a point cloud.";
            return;
    }


	/* get and set config data */
//	unsigned int clen;
	min_x = -0.5;// *((double*) ubx_config_get_data_ptr(c, "min_x", &clen));
	max_x = 0.1;//*((double*) ubx_config_get_data_ptr(c, "max_x", &clen));
	min_y = -0.5;//*((double*) ubx_config_get_data_ptr(c, "min_y", &clen));
	max_y = 0.1;//*((double*) ubx_config_get_data_ptr(c, "max_y", &clen));
	min_z = -0.5;//*((double*) ubx_config_get_data_ptr(c, "min_z", &clen));
	max_z = 0.5;//*((double*) ubx_config_get_data_ptr(c, "max_z", &clen));

    filter->setSizeX(fabs(min_x)); // set new dimensions
    filter->setSizeY(fabs(min_y));
    filter->setSizeZ(fabs(min_z));
    double* transformMatrix = center->setRawData(); // set new center
    transformMatrix[12] = max_x - ((max_x-min_x)/2.0);
    transformMatrix[13] = max_y - ((max_y-min_y)/2.0);
    transformMatrix[14] = max_z - ((max_z-min_z)/2.0);

    /* define where to store results */
	brics_3d::PointCloud3D::PointCloud3DPtr outputPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr outputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	outputPointCloudContainer->data = outputPointCloud;

	/* do computation */
	LOG(INFO) << "ROIFilter: Computing for input with " << inputPointCloudContainer->data->getSize() << " points.";
    filter->filter(inputPointCloudContainer->data.get(), outputPointCloudContainer->data.get());
    LOG(INFO) << "ROIFilter: Computing done with output of " << outputPointCloudContainer->data->getSize() << " points.";

	/* prepare output */
    brics_3d::rsg::Id roiPointCloudId = 21;
    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
    attributes.push_back(brics_3d::rsg::Attribute("name","roi_point_cloud"));
    attributes.push_back(brics_3d::rsg::Attribute("origin","roifilter"));
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
#endif


	/* push output to microblx */
	ubx_port_t* outputPort = ubx_port_get(c, "outputDataIds");
	rsg_ids toBeSendOutputDataIs;
	toBeSendOutputDataIs.numberOfIds = 0u;
	brics_3d::rsg::UbxTypecaster::convertIdsToUbx(output, toBeSendOutputDataIs);
	write_rsg_ids(outputPort, &toBeSendOutputDataIs);

}


/* put everything together */
ubx_block_t roifilter_comp = {
	.name = "roifilter/roifilter",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = roifilter_meta,
	.configs = roifilter_config,
	.ports = roifilter_ports,

	/* ops */
	.init = roifilter_init,
	.start = roifilter_start,
	.step = roifilter_step,
	.cleanup = roifilter_cleanup,
};

static int roifilter_init(ubx_node_info_t* ni)
{
	LOG(DEBUG) << "roifilter_init(ubx_node_info_t* ni)";
	return ubx_block_register(ni, &roifilter_comp);
}

static void roifilter_cleanup(ubx_node_info_t *ni)
{
	LOG(DEBUG) << "roifilter_cleanup(ubx_node_info_t *ni)";
	ubx_block_unregister(ni, "roifilter/roifilter");
}

UBX_MODULE_INIT(roifilter_init)
UBX_MODULE_CLEANUP(roifilter_cleanup)

