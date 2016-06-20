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
 * @file posehistory
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
#include <fstream>
#include <cstring>

/* microblx includes */
#include "ubx.h"

/* microblx type for the robot scene graph */
#include "../types/rsg/types/rsg_types.h"
#include "brics_3d/util/UbxTypecaster.h"

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h> // concrete type
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/PathCollector.h>
#include <brics_3d/worldModel/sceneGraph/INodeVisitor.h>
#include <brics_3d/util/JSONTypecaster.h>


/***********FBX***************/

/**
 * Downwards graph traverser. Starts at accepted node and stores all paths to specified.
 */
class TransformsAlongPathCollector : public brics_3d::rsg::INodeVisitor {
public:
	TransformsAlongPathCollector() : INodeVisitor(brics_3d::rsg::INodeVisitor::upwards){};
	virtual ~TransformsAlongPathCollector(){};

	virtual void visit(brics_3d::rsg::Node* node) {
		assert (node != 0);
		currentPath.insert(currentPath.begin(), node);
		if (node->getNumberOfParents() == 0) { //root reached so collect data in new path
			if (static_cast<unsigned int>(nodePaths.size()) == 0u) { //remove the "caller's" pointer
				currentPath.pop_back();
			}
			nodePaths.push_back(currentPath);
			currentPath.clear();
		}
	}

	virtual void visit(brics_3d::rsg::Group* node) {
		this->visit(dynamic_cast<brics_3d::rsg::Node*>(node)); //just feed forward to be handled as node
	}
	virtual void visit(brics_3d::rsg::Transform* node) {
		this->visit(dynamic_cast<brics_3d::rsg::Node*>(node)); //just feed forward to be handled as node
	}
	virtual void visit(brics_3d::rsg::GeometricNode* node) {
		this->visit(dynamic_cast<brics_3d::rsg::Node*>(node)); //just feed forward to be handled as node
	}
	virtual void visit(brics_3d::rsg::Connection* connection) {
		this->visit(dynamic_cast<brics_3d::rsg::Node*>(connection)); //just feed forward to be handled as node
	}

	virtual void reset();

	brics_3d::rsg::Node::NodePathList getNodePaths() const
    {
        return nodePaths;
    }

protected:

	brics_3d::rsg::Node::NodePath currentPath;
	brics_3d::rsg::Node::NodePathList nodePaths;

};

class PoseHistory: public brics_3d::rsg::IFunctionBlock {
public:

	PoseHistory(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "posehistory";
		inputMetaModelFile = "fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */


	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;
	};

	~PoseHistory(){
		LOG(INFO) << name << ": Stopping block " << name;
	};

	bool configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";

		return true;
	}

	bool execute(std::vector<brics_3d::rsg::Id>& inputDataIds, std::vector<brics_3d::rsg::Id>& outputDataIds) {
		LOG(WARNING) << name << " Cannot execute Id based i/o as this is a query function block";
		return false;
	}

	bool execute(std::string inputModel, std::string& outputModel) {
		LOG(DEBUG) << name << ": Execute.";
		outputModel = "{}";
		libvariant::Variant inputModelAsJSON;
		libvariant::Variant outputModelAsJSON;
		brics_3d::rsg::JSONTypecaster::stringToJSON(inputModel, inputModelAsJSON);

		/* validate input */
		bool enableValidation = true;
		if (enableValidation) {
			if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(inputModelAsJSON, inputMetaModelFile, modelsDefaultPath, outputModel)) {
				LOG(ERROR) << "PoseHistory: Model validation for input model failed: " << outputModel;
				return false;
			}
		}



		/* get data */
		brics_3d::rsg::Id id = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "id");
		brics_3d::rsg::Id idReferenceNode = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "idReferenceNode");

		/* output data */
		brics_3d::rsg::TemporalCache<brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> history;

		/* execute query
		 * 1.) Quick look up for direct connection
		 */

		brics_3d::rsg::PathCollector pathCollector = brics_3d::rsg::PathCollector();
		wm->scene.executeGraphTraverser(&pathCollector, id); //

		if (static_cast<unsigned int>(pathCollector.getNodePaths().size()) > 0) { // != root
			//Lookup first path with length of 1, htat has a transform
			brics_3d::rsg::Node::NodePath nodePath = pathCollector.getNodePaths().back();

			/* Go along path and accumulate the time same of all transform caches */
			std::vector<brics_3d::rsg::TimeStamp> stamps;
			for (unsigned int i = 0; i < static_cast<unsigned int>(nodePath.size()); ++i) {
				if ((*nodePath[i]).getId() == idReferenceNode) {
					break;// do not further go towards the root node
				}
				brics_3d::rsg::Transform* tmpTransform = dynamic_cast<brics_3d::rsg::Transform*>(nodePath[i]);
				if (tmpTransform) {
					//got it
					LOG(DEBUG) << name << ": found a pose history :" << tmpTransform->cacheToString();
					history = tmpTransform->getHistory();
					for(std::vector<std::pair<brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr, brics_3d::rsg::TimeStamp> >::const_iterator it = history.begin();
							it != history.end();++it) {
						stamps.push_back(it->second);
					}
					break; //FIXME
				}
			}


		}

		/* prepare output */
		bool result = true;

//		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
//		TimeStamp timeStamp, bool forcedId) {
//		history.insertData(transform, timeStamp);
		brics_3d::rsg::JSONTypecaster::addTransformCacheToJSON(history, outputModelAsJSON);

		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation && false) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "PoseHistory: Model validation for output model failed: " << outputModel;
					return false;
				}
			}
		}

		return result;
	}

	bool getMetaModel(std::string& inputMetaModel, std::string& outputMetaModel) {
		std::stringstream buffer;

		std::string file = modelsDefaultPath + "/" + inputMetaModelFile;
		std::ifstream inputMetaModelFileStream(file.c_str());
		buffer.str("");
		buffer << inputMetaModelFileStream.rdbuf();
		inputMetaModel = buffer.str();

		file = modelsDefaultPath + "/" + outputMetaModelFile;
		std::ifstream outputMetaModelFileStream(file.c_str());
		buffer.str("");
		buffer << outputMetaModelFileStream.rdbuf();
		outputMetaModel = buffer.str();

		return true;
	}

private:

	/* Meta data */
	std::string name;
	std::string modelsDefaultPath;
	std::string inputMetaModelFile;
	std::string outputMetaModelFile;

	/* Algorithm(s) */


	/* Parameters */

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(PoseHistory)
FUNCTION_BLOCK_DESTROY

/***********UBX***************/

UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)

using namespace std;
using brics_3d::Logger;

brics_3d::WorldModel* wmHandle = 0;
PoseHistory* posehistory;


/* function block meta-data
 * used by higher level functions.
 */
char posehistory_meta[] =
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
ubx_config_t posehistory_config[] = {
	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
	{ NULL },
};

ubx_port_t posehistory_ports[] = {
	{ .name="inputDataIds", .attrs=PORT_DIR_IN, .in_type_name="struct rsg_ids"}, //unsigned int" },
	{ .name="outputDataIds", .attrs=PORT_DIR_OUT, .out_type_name="struct rsg_ids"},
	{ NULL },
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
//def_read_fun(read_rsg_ids, rsg_ids)
//def_write_fun(write_rsg_ids, rsg_ids)

static int posehistory_init(ubx_block_t *c)
{
	LOG(INFO) << "PoseHistory: initializing: " << c->name;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	unsigned int clen;
	rsg_wm_handle tmpWmHandle =  *((rsg_wm_handle*) ubx_config_get_data_ptr(c, "wm_handle", &clen));
	assert(clen != 0);
	wmHandle = reinterpret_cast<brics_3d::WorldModel*>(tmpWmHandle.wm); // We know that this pointer stores the world model type
	if(wmHandle == 0) {
		LOG(FATAL) << "PoseHistory: World modeld handle could not be initialized.";
		return -1;
	}

	/* init algorithm */
    posehistory = new PoseHistory(wmHandle);

	return 0;
}


static void posehistory_cleanup(ubx_block_t *c)
{
	if (posehistory != 0) {
		delete posehistory;
		posehistory = 0;
	}
	LOG(INFO) << "PoseHistory: cleaning up: " << c->name;
}

static int posehistory_start(ubx_block_t *c)
{
	LOG(INFO) << "PoseHistory: starting. " << c->name;
	return 0; /* Ok */
}

static void posehistory_step(ubx_block_t *c) {
	LOG(INFO) << "PoseHistory: executing: " << c->name;

}


/* put everything together */
ubx_block_t posehistory_comp = {
	.name = "posehistory/posehistory",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = posehistory_meta,
	.configs = posehistory_config,
	.ports = posehistory_ports,

	/* ops */
	.init = posehistory_init,
	.start = posehistory_start,
	.step = posehistory_step,
	.cleanup = posehistory_cleanup,
};

static int posehistory_init(ubx_node_info_t* ni)
{
	LOG(DEBUG) << "posehistory_init(ubx_node_info_t* ni)";
	return ubx_block_register(ni, &posehistory_comp);
}

static void posehistory_cleanup(ubx_node_info_t *ni)
{
	LOG(DEBUG) << "posehistory_cleanup(ubx_node_info_t *ni)";
	ubx_block_unregister(ni, "posehistory/posehistory");
}

UBX_MODULE_INIT(posehistory_init)
UBX_MODULE_CLEANUP(posehistory_cleanup)

