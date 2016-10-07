/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2016, KU Leuven
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
 * @file osgvisualizer
 *
 * A function block to visualize the RSG via Open Scene Graph
 *
 */

/* std includes */
#include <iostream>
#include <fstream>
#include <cstring>

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/util/JSONTypecaster.h>


/***********FBX***************/


class OsgVisualizer: public brics_3d::rsg::IFunctionBlock {
public:

	OsgVisualizer(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "osgvisualizer";
		inputMetaModelFile = "fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */
		visualizer = new brics_3d::rsg::OSGVisualizer();

		osgConfiguration.visualizeIds = true;
		osgConfiguration.visualizeAttributes = true;
		visualizer->setConfig(osgConfiguration);
		wm->scene.attachUpdateObserver(visualizer); //enable visualization
		wm->scene.advertiseRootNode(); // Don't forget this one! Otherwise the observers cannot correctly handle the updates.

	    /* get default location of model schemas */
//	    char defaultFilename[255] = { FBX_MODELS_DIR };
		char defaultFilename[255] = { "." };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;
	};

	~OsgVisualizer() {
		wm->scene.detachUpdateObserver(visualizer);
		delete visualizer;
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
				LOG(ERROR) << "OsgVisualizer: Model validation for input model failed: " << outputModel;
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


		/* prepare output */
		bool result = true;


		brics_3d::rsg::JSONTypecaster::addTransformCacheToJSON(history, outputModelAsJSON);

		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation && false) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "OsgVisualizer: Model validation for output model failed: " << outputModel;
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
	brics_3d::rsg::OSGVisualizer* visualizer;

	/* Parameters */
	brics_3d::rsg::VisualizationConfiguration osgConfiguration;
};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(OsgVisualizer)
FUNCTION_BLOCK_DESTROY

