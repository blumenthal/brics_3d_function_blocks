/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2017, KU Leuven
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
 * @file demloader
 *
 * A query function block to load a DEM GIS map.
 *
 *
 */


/* std includes */
#include <iostream>
#include <fstream>
#include <cstring>


/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h> // concrete type
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/PathCollector.h>
#include <brics_3d/worldModel/sceneGraph/INodeVisitor.h>
#include <brics_3d/util/JSONTypecaster.h>

/* GDAL includes */
#include "gdal_priv.h"

/***********FBX***************/


class DemLoader: public brics_3d::rsg::IFunctionBlock {
public:

	DemLoader(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "demloader";
		inputMetaModelFile = "fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */
		demDataset = 0;

	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;
	};

	~DemLoader(){
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
				LOG(ERROR) << "DemLoader: Model validation for input model failed: " << outputModel;
				return false;
			}
		}

		/* get data */
		string command = inputModelAsJSON.Get("command").AsString(); // "enum": [ "LOAD_MAP", "GET_ELEVATION", "GET_MIN_MAX_ELEVATION"],
//		double longitude = inputModelAsJSON.Get("longitude").AsDouble();
//		double latitude = inputModelAsJSON.Get("latitude").AsDouble();
		brics_3d::rsg::Id areaId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "areaId");
		string demFile = inputModelAsJSON.Get("file").AsString();


		/* output data */
		bool result = false;
//	   	double elevation = -1.0;
//	   	double minElevation = -1.0;
//	   	double maxElevation = -1.0;
		//libvariant::Variant poses(libvariant::VariantDefines::ListType);

		/* execute query
		 *
		 */
		if (command.compare("LOAD_MAP") == 0) {
			LOG(DEBUG) << "DemLoader: LOAD_MAP";

		    GDALAllRegister();
		    demDataset = (GDALDataset *) GDALOpen( demFile.c_str(), GA_ReadOnly );
		    if(demDataset){
		    	result = true;

		    	printf( "Driver: %s/%s\n",
		    			demDataset->GetDriver()->GetDescription(),
		    			demDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );
		    	printf( "Size is %dx%dx%d\n",
		    			demDataset->GetRasterXSize(), demDataset->GetRasterYSize(),
		    			demDataset->GetRasterCount() );
		    	if( demDataset->GetProjectionRef()  != NULL )
		    		printf( "Projection is `%s'\n", demDataset->GetProjectionRef() );
		    	if( demDataset->GetGeoTransform( geoTransform ) == CE_None )
		    	{
		    		printf( "Origin = (%.6f,%.6f)\n",
		    				geoTransform[0], geoTransform[3] );
		    		printf( "Pixel Size = (%.6f,%.6f)\n",
		    				geoTransform[1], geoTransform[5] );
		    	}
		    }





		} else if (command.compare("GET_ELEVATION") == 0) {
			LOG(DEBUG) << "DemLoader: GET_ELEVATION";
		} else if (command.compare("GET_MIN_MAX_ELEVATION") == 0) {
			LOG(DEBUG) << "DemLoader: GET_MIN_MAX_ELEVATION";
		} else {
			LOG(ERROR) << "DemLoader: Unknown command.";
		}


		/* prepare output */
		//outputModelAsJSON.Set("poses", poses);


		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation && false) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "DemLoader: Model validation for output model failed: " << outputModel;
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
	GDALDataset *demDataset;

	/* Parameters */
	double geoTransform[6];

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(DemLoader)
FUNCTION_BLOCK_DESTROY
