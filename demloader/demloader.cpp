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
		demMinElevation = -1.0;
		demMaxElevation = - 1.0;
		demMaxPixelSizeX = 0;
		demMaxPixelSizeY = 0;
		demBandIndex = 1;

		/*
		 * Values below -10,971 [m] Are not plausible since this is the absolute minimum on earth.
		 * Use to determine invalid points.
		 * E.g. ArcGIS uses -32767 to indicate a VOID type. (~ limit of 16bit int)
		 */
		globalMinElevation = -11000; // in [m]

	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;
	};

	~DemLoader() {
		if(demDataset) {
		    GDALClose((GDALDataset *) demDataset);
			demDataset = 0;
		}
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
			LOG(INFO) << "DemLoader: LOAD_MAP";

		    GDALAllRegister();
		    demDataset = (GDALDataset *) GDALOpen( demFile.c_str(), GA_ReadOnly );
		    if(demDataset) {

		    	/* print meta data */
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

		    	/* access band */
		    	GDALRasterBand  *poBand;
		    	int             nBlockXSize, nBlockYSize;
		    	int             bGotMin, bGotMax;
		    	double          adfMinMax[2];
		    	poBand = demDataset->GetRasterBand(demBandIndex);
		    	poBand->GetBlockSize(&nBlockXSize, &nBlockYSize);
		    	printf( "Block=%dx%d Type=%s, ColorInterp=%s\n",
		    	        nBlockXSize, nBlockYSize,
		    	        GDALGetDataTypeName(poBand->GetRasterDataType()),
		    	        GDALGetColorInterpretationName(
		    	            poBand->GetColorInterpretation()) );
		    	adfMinMax[0] = poBand->GetMinimum( &bGotMin );
		    	adfMinMax[1] = poBand->GetMaximum( &bGotMax );
		    	if( ! (bGotMin && bGotMax) )
		    	    GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
		    	printf( "Min=%.3fd, Max=%.3f\n", adfMinMax[0], adfMinMax[1] );
		    	demMinElevation =  adfMinMax[0];
		    	demMaxElevation =  adfMinMax[1];
		    	if( poBand->GetOverviewCount() > 0 )
		    	    printf( "Band has %d overviews.\n", poBand->GetOverviewCount() );
		    	if( poBand->GetColorTable() != NULL )
		    	    printf( "Band has a color table with %d entries.\n",
		    	             poBand->GetColorTable()->GetColorEntryCount() );

		    	 demMaxPixelSizeX = poBand->GetXSize();
		    	 demMaxPixelSizeY = poBand->GetYSize();

		    	if(poBand->GetRasterDataType() != GDT_Float32) {
				    GDALClose((GDALDataset *) demDataset);
					demDataset = 0;
		    		result = false;
					outputModelAsJSON.Set("result", "DEM_FILE_NOT_LOADED");
		    	} else {

					/* read it */
					float *pafScanline;
					int   nXSize = poBand->GetXSize();
					int line = 3000;
					pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize);
					poBand->RasterIO( GF_Read, 0, line, nXSize, 1,
									  pafScanline, nXSize, 1, poBand->GetRasterDataType(),
									  0, 0 );

					/* try affine projections */
					int xPixel = 0;
					int yPixel = 0;
					double xGeo = 0;
					double yGeo = 0;
					pixelToWorld(xPixel, yPixel, xGeo, yGeo);
					worldToPixel(xGeo, yGeo, xPixel, yPixel);


//					for (int x = 0; x < nXSize; ++x) {
//
//						/*
//						 * Note, pixels can be invalid!
//						 * E.g. ArcGIS uses -32767 to indicate a VOID type. (~ limit of 16bit int)
//						 * Values below -10,971 [m] Are not plausible since this is the absolute minimum on earth
//						 */
//						printf("(%i, %f),", x , pafScanline[x]);
//
//					}
		    	}

		    	result = true;
				outputModelAsJSON.Set("result", "DEM_FILE_LOADED");
		    }





		} else if (command.compare("GET_ELEVATION") == 0) {
			LOG(INFO) << "DemLoader: GET_ELEVATION";
			outputModelAsJSON.Set("result", "DEM_FILE_NOT_LOADED");

		    if(demDataset) {
		    	double xGeo = inputModelAsJSON.Get("latitude").AsDouble();
		    	double yGeo = inputModelAsJSON.Get("longitude").AsDouble();
		    	double elevation = globalMinElevation;
		    	string resultMessage = "";

		    	result = getElevationAt(xGeo, yGeo, elevation, resultMessage);

				if(result) { // Only valid values are returned, other wise this filed is skipped
					outputModelAsJSON.Set("elevation", elevation);
				}

		    	outputModelAsJSON.Set("result", resultMessage);
		    }

		} else if (command.compare("GET_MIN_MAX_ELEVATION") == 0) {
			LOG(INFO) << "DemLoader: GET_MIN_MAX_ELEVATION";
			outputModelAsJSON.Set("result", "DEM_FILE_NOT_LODED");

		    if(demDataset) {
		    	outputModelAsJSON.Set("minElevation", demMinElevation);
		    	outputModelAsJSON.Set("maxElevation", demMaxElevation);
		    	outputModelAsJSON.Set("result", "ELEVATION_VALUE_EXISTS");
		    	result = true;
		    }

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

	bool pixelToWorld (int xPixel, int yPixel, double& xGeo, double& yGeo) {
		if(!demDataset) {
			LOG(ERROR) << name << "pixelToWorld: DEM not loaded yet.";
			return false;
		}

		/* affine projection */
		xGeo = geoTransform[0] + xPixel*geoTransform[1] + yPixel*geoTransform[2];
    	yGeo = geoTransform[3] + xPixel*geoTransform[4] + yPixel*geoTransform[5];
    	LOG(DEBUG) << name << "pixelToWorld: pixel at (" << xPixel << ", " << yPixel << ") is geolocated at (" << xGeo << ", "<< yGeo << ")";

		return true;
	}

	bool worldToPixel (double xGeo, double yGeo, int& xPixel, int& yPixel) {
		if(!demDataset) {
			LOG(ERROR) << name << "worldToPixel: DEM not loaded yet.";
			return false;
		}

		/* affine (inverse) projection */
		/* NOTE: Due to rounding erroirs the location can be off by one pixel */
    	xPixel = int((xGeo - geoTransform[0]) / geoTransform[1]);
    	yPixel = int((yGeo - geoTransform[3]) / geoTransform[5]);
    	LOG(DEBUG) << name << "worldToPixel: geolocation at (" << xGeo << ", " << yGeo << ") is corresponds to elemet at (" << xPixel << ", "<< yPixel << ")";

    	/* check boundaries */
    	if ( xPixel < 0 || xPixel >= demMaxPixelSizeX ) {
    		LOG(ERROR) << name << "worldToPixel: X = " << xPixel << " is out of bounds [" << 0 << ", " << demMaxPixelSizeX << "].";
    		return false;
    	}
    	if ( yPixel < 0 || yPixel >= demMaxPixelSizeY ) {
    		LOG(ERROR) << name << "worldToPixel: Y = " << xPixel << " is out of bounds [" << 0 << ", " << demMaxPixelSizeY << "].";
    		return false;
    	}

    	return true;
	}

	bool getElevationAt(double xGeo, double yGeo, double& elevation, string& resultMessage) {
		resultMessage = "ELEVATION_ERROR"; // only internally used
		elevation = globalMinElevation;

		int xPixel = 0;
		int yPixel = 0;

		/* get pixel indices and check if they are within the boundaries of the raster */
		if(!worldToPixel(xGeo, yGeo, xPixel, yPixel)) {
			resultMessage = "ELEVATION_QUERY_OUT_OF_BOUNDS";
			return false;
		}

		/* open respective band*/
		if(demDataset) {
	    	GDALRasterBand  *poBand;
	    	poBand = demDataset->GetRasterBand(demBandIndex);

	    	/* read it */
	    	float *pafScanline;
	    	int   nXSize = poBand->GetXSize();
	    	int line = yPixel;
	    	pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize);
	    	poBand->RasterIO( GF_Read, 0, line, nXSize, 1,
	    			pafScanline, nXSize, 1, poBand->GetRasterDataType(),
	    			0, 0 );

	    	elevation = pafScanline[xPixel];
	    	LOG(DEBUG) << name << "getElevationAt: Found elevation value = " << elevation << " at geolocation (" << xGeo << ", " << yGeo << ")";
			resultMessage = "ELEVATION_VALUE_EXISTS";

			if(elevation <= globalMinElevation) {
				LOG(DEBUG) << name << "getElevationAt: Elevation value = " << elevation << " is invalid.";
				resultMessage = "ELEVATION_VALUE_INVALID";
				return false;
			}

	    	for (int x = 0; x < nXSize; ++x) { // DGB
	    		printf("(%i, %f),", x , pafScanline[x]);
	    	}


		} else {
			resultMessage = "DEM_FILE_NOT_LOADED";
			return false;
		}

		return true;
	}

	/* Meta data */
	std::string name;
	std::string modelsDefaultPath;
	std::string inputMetaModelFile;
	std::string outputMetaModelFile;

	/* Algorithm(s) */
	GDALDataset *demDataset;

	/* Parameters */
	double geoTransform[6];
	double demMaxElevation;
	double demMinElevation;
	double globalMinElevation;
	int demMaxPixelSizeX;
	int demMaxPixelSizeY;
	int demBandIndex;

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(DemLoader)
FUNCTION_BLOCK_DESTROY
