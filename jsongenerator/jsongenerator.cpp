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
 * @file JsonGenerator
 *
 * A function block to request the generation of a json model file.
 * It has options to only grenerate subpraph(s)
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
#include <brics_3d/worldModel/sceneGraph/JSONGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/NodeHashTraverser.h>
#include <brics_3d/util/JSONTypecaster.h>


/***********FBX***************/


class JsonGenerator: public brics_3d::rsg::IFunctionBlock {
public:

	JsonGenerator(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "JsonGenerator";
		inputMetaModelFile = "fbx-subgraph-and-file-schema.json"; //"fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-file-schema.json"; //"fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */

	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;
	};

	~JsonGenerator(){
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
				LOG(ERROR) << "JsonGenerator: Model validation for input model failed: " << outputModel;
				return false;
			}
		}

		/* get data */
		brics_3d::rsg::Id subgraphId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "subgraphId");

		/* output data */
		libvariant::Variant poses(libvariant::VariantDefines::ListType);
		bool result = true;


		/* execute query
		 *
		 */
        std::string fileName;
        std::string jsonFileName;
        std::string path = "./";
		std::stringstream tmpFileName;
		std::ofstream output;

		/* default file name with stamp */
		time_t rawtime;
		struct tm* timeinfo;
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		tmpFileName << "20" // This will have to be adjusted in year 2100
				<< (timeinfo->tm_year)-100 << "-"
				<< std::setw(2) << std::setfill('0')
				<<	(timeinfo->tm_mon)+1 << "-"
				<< std::setw(2) << std::setfill('0')
				<<	timeinfo->tm_mday << "_"
				<< std::setw(2) << std::setfill('0')
				<<	timeinfo->tm_hour << "-"
				<< std::setw(2) << std::setfill('0')
				<<	timeinfo->tm_min << "-"
				<< std::setw(2) << std::setfill('0')
				<<	timeinfo->tm_sec;

		jsonFileName = "rsg_dump_" + tmpFileName.str();

		/* Check if input specifies fileName or path*/
		if(inputModelAsJSON.Contains("jsonFileName")) { // optional
			jsonFileName = inputModelAsJSON.Get("jsonFileName").AsString();
		}
		if(inputModelAsJSON.Contains("path")) { // optional
			path = inputModelAsJSON.Get("path").AsString();
		}


		fileName = path + jsonFileName + ".json";
		LOG(INFO) << "JsonGenerator: Printing graph to file " << fileName;

		/* Save a complete snapshopt relative to the root node */
		bool printRemoteRootNodes = true;
		if(!subgraphId.isNil()) {
			wm->scene.executeGraphTraverser(&jsonGraphGenerator, subgraphId);
			wm->scene.executeGraphTraverser(&hashTraverser, subgraphId);
			LOG(INFO) << "Hash for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(subgraphId);
			hashTraverser.reset(false);
			wm->scene.executeGraphTraverser(&hashTraverser, subgraphId);
			LOG(INFO) << "Hash without IDs for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(subgraphId);
			LOG(INFO) << "Hash list:" << std::endl << hashTraverser.getJSON();
			printRemoteRootNodes = false;
		} else { // NiL = not defined/specified => as default print the whole graph
			wm->scene.executeGraphTraverser(&jsonGraphGenerator, wm->scene.getRootId());
			wm->scene.executeGraphTraverser(&hashTraverser, wm->scene.getRootId());
			LOG(INFO) << "Hash for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(wm->scene.getRootId());
			hashTraverser.reset(false);
			wm->scene.executeGraphTraverser(&hashTraverser, subgraphId);
			LOG(INFO) << "Hash without IDs for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(subgraphId);
			LOG(INFO) << "Hash list:" << std::endl << hashTraverser.getJSON();
			printRemoteRootNodes = true;
		}

		if(printRemoteRootNodes) {
			vector<brics_3d::rsg::Id> remoteRootNodeIds;
			wm->scene.getRemoteRootNodes(remoteRootNodeIds);
			for(vector<brics_3d::rsg::Id>::const_iterator it = remoteRootNodeIds.begin(); it != remoteRootNodeIds.end(); ++it) {
				wm->scene.executeGraphTraverser(&jsonGraphGenerator, *it);
				wm->scene.executeGraphTraverser(&hashTraverser, *it);
				LOG(INFO) << "Hash for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(*it);
				hashTraverser.reset(false);
				wm->scene.executeGraphTraverser(&hashTraverser, *it);
				LOG(INFO) << "Hash without IDs for subgraph with ID = " << subgraphId << " == " << hashTraverser.getHashById(*it);
				LOG(INFO) << "Hash list:" << std::endl << hashTraverser.getJSON();
			}
		}

		output.open((fileName).c_str(), std::ios::trunc);
		if (!output.fail()) {
			output << jsonGraphGenerator.getJSON();
		} else {
			LOG(ERROR) << "JsonGenerator: Cannot write to file " << fileName;
		}

		output.flush();
		output.close();
		jsonGraphGenerator.reset();
		hashTraverser.reset();

		LOG(INFO) << "JsonGenerator: Done.";


		/* prepare output */
		outputModelAsJSON.Set("file", fileName);


		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation /*&& false*/) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "JsonGenerator: Model validation for output model failed: " << outputModel;
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
	brics_3d::rsg::JSONGraphGenerator jsonGraphGenerator;
	brics_3d::rsg::NodeHashTraverser hashTraverser;

	/* Parameters */

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(JsonGenerator)
FUNCTION_BLOCK_DESTROY
