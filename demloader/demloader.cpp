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
 * A query function block to load and query a Digital Elevation Map (DEM).
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
#include "ogrsf_frmts.h"

/* types for UNITY maps */
typedef uint16_t unity_pixel_t;

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
		oSourceSRS.SetWellKnownGeogCS("WGS84"); // default value, might be overridden by meta data of dataset
		oTargetSRS.SetWellKnownGeogCS("WGS84");
		poCT = 0;

		type = "DEM";
		xRangeInMeters = 0;
		yRangeInMeters = 0;
		zRangeInMeters = 0;

		/*
		 * Values below -10,971 [m] Are not plausible since this is the absolute minimum on earth.
		 * Use to determine invalid points.
		 * E.g. ArcGIS uses -32767 to indicate a VOID type. (~ limit of 16bit int)
		 * NOTE: Some other reprojection tools insert 0 as VOID type. E.g. gdalwarp
		 * so we put 0 to be on the safe side.
		 */
    	globalMinElevation = 0;//-11000; // in [m]

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


		/* output data */
		bool result = false;

		/* execute query
		 *
		 */
		if (command.compare("LOAD_MAP") == 0) {
			LOG(INFO) << "DemLoader: LOAD_MAP";

			if(demDataset) {
				LOG(INFO) << "DemLoader: Overriding map file.";
			    GDALClose((GDALDataset *) demDataset);
				demDataset = 0;
			}

			string demFile = inputModelAsJSON.Get("file").AsString();

			if(inputModelAsJSON.Contains("type")) { // optional
				type = inputModelAsJSON.Get("type").AsString();
				if (type.compare("UNITY") == 0) {
					if(inputModelAsJSON.Contains("xRangeInMeters")) { // optional
						xRangeInMeters = inputModelAsJSON.Get("xRangeInMeters").AsDouble();
					}
					if(inputModelAsJSON.Contains("xRangeInMeters")) { // optional
						yRangeInMeters = inputModelAsJSON.Get("yRangeInMeters").AsDouble();
					}
					if(inputModelAsJSON.Contains("xRangeInMeters")) { // optional
						zRangeInMeters = inputModelAsJSON.Get("zRangeInMeters").AsDouble();
					}
				}
				LOG(INFO) << "DemLoader: Using UNITY type with dimensions ("
						<< xRangeInMeters << ", "
						<< yRangeInMeters << ", "
						<< zRangeInMeters << ") in [m].";
			}

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


		    	/*
		    	 * Get geo coordinate system for meta data of the file.
		    	 * E.g. the Davos map uses EPSG:21781 while the Chamoluc map has WGS84
		    	 * cf. http://www.gdal.org/osr_tutorial.htm
		    	 */
		    	char *pszWkt = const_cast<char*>(demDataset->GetProjectionRef());
		    	oTargetSRS.importFromWkt(&pszWkt); // strange that it is not a const char* here ...

				char    *pszWKT = NULL;
				oTargetSRS.exportToWkt( &pszWKT );
				printf( "%s\n", pszWKT );


				poCT = OGRCreateCoordinateTransformation(&oTargetSRS, &oSourceSRS);
				LOG(DEBUG) << "DemLoader: poCT = " << poCT;

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


//		    	 The map's dimensions as a bounding box: e.g. 5.2kmx5.2kmx1.1km. So the resolution of such maps can be calculated by simply dividing pixel numbers by map size:
//
//		    	 Resolution_x = x_pixelsize / 5.2km
//		    	 Resolution_y = y_pixelsize / 5.2km
//		    	 Resolution_z = 2^8(grayscale range)  / 1.1km
//
// 				 https://docs.unrealengine.com/udk/Three/TerrainHeightmaps.html
//		 		xGeo = geoTransform[0] + xPixel*geoTransform[1] + yPixel*geoTransform[2];
//		     	yGeo = geoTransform[3] + xPixel*geoTransform[4] + yPixel*geoTransform[5];


		    	 if(type.compare("UNITY") == 0) {
		    		 // translation
		    		 geoTransform[0] = 0; // upper left corner?
		    		 geoTransform[3] = 0;

		    		 //scale (Pixel Size)
//		    		 geoTransform[1] = geoTransform[4] = demMaxPixelSizeX / xRangeInMeters;
//		    		 geoTransform[5] = geoTransform[5] = demMaxPixelSizeY / yRangeInMeters;
		    		 geoTransform[1] = xRangeInMeters / demMaxPixelSizeX;
		    		 geoTransform[5] = yRangeInMeters / demMaxPixelSizeY;

//		    		 demMinElevation =  demMinElevation / std::numeric_limits<uint16_t>::max()  * zRangeInMeters;
//		    		 demMaxElevation =  demMaxElevation / std::numeric_limits<uint16_t>::max()  * zRangeInMeters;
		    		 demMinElevation = unityMapToElevation(demMinElevation);
		    		 demMaxElevation = unityMapToElevation(demMaxElevation);
		    	 }

		    	if((poBand->GetRasterDataType() != GDT_Float32) && (poBand->GetRasterDataType() != GDT_UInt16)) {
				    GDALClose((GDALDataset *) demDataset);
					demDataset = 0;
		    		result = false;
					outputModelAsJSON.Set("result", "DEM_FILE_NOT_LOADED");
		    	} else {

//					/* read it */
//					float *pafScanline;
//					int   nXSize = poBand->GetXSize();
//					int line = 0;
//					pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize);
//					poBand->RasterIO( GF_Read, 0, line, nXSize, 1,
//									  pafScanline, nXSize, 1, poBand->GetRasterDataType(),
//									  0, 0 );

					/* try affine projections - this is only for debugging purposes*/
					int xPixel = 0;
					int yPixel = 0;
					double xGeo = 0;
					double yGeo = 0;
					pixelToWorld(xPixel, yPixel, xGeo, yGeo);
					worldToPixel(xGeo, yGeo, xPixel, yPixel);


			    	double elevation = -1.0;
			    	string resultMessage = "";
			    	getElevationAt(xGeo, yGeo, elevation, resultMessage); //0, 0 -> 63
			    	LOG(INFO) << name << " Elevation at (" <<  xGeo << ", " << yGeo << ") = " << elevation;
			    	getElevationAt(66, 40, elevation, resultMessage); //51, 31 -> 63
			    	LOG(INFO) << name << " Elevation at (" <<  xGeo << ", " << yGeo << ") = " << elevation;
			    	getElevationAt(2600, 2600, elevation, resultMessage); //center (?) 2023, 2032 -> 16
			    	LOG(INFO) << name << " Elevation at (" <<  xGeo << ", " << yGeo << ") = " << elevation;

			    	result = true;
					outputModelAsJSON.Set("result", "DEM_FILE_LOADED");
		    	}


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

				if(result) { // Only valid values are returned, otherwise this file is skipped
					outputModelAsJSON.Set("elevation", elevation);
				}

		    	outputModelAsJSON.Set("result", resultMessage);
		    }

		} else if (command.compare("GET_MIN_MAX_ELEVATION") == 0) {
			LOG(INFO) << "DemLoader: GET_MIN_MAX_ELEVATION";
			outputModelAsJSON.Set("result", "DEM_FILE_NOT_LODED");

		    if(demDataset) {
		    	brics_3d::rsg::Id areaId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "areaId");

		    	if(!areaId.isNil()) {
		    		result = false;
		    		float minAreaElevation = std::numeric_limits<float>::max();
		    		float maxAreaElevation = std::numeric_limits<float>::min();

		    		/*
		    		 * prepare polygon
		    		 */
		    		std::vector<brics_3d::rsg::Id> polygonIs;
		    		if(!wm->scene.getConnectionTargetIds(areaId,  polygonIs)) {
		    			LOG(ERROR) << name << "No area connection found.";
		    			return false; // no connection found
		    		}

		    		/* Get origin */
		    		brics_3d::rsg::Id originId;
		    		vector<brics_3d::rsg::Attribute> attributes;
		    		vector<brics_3d::rsg::Id> ids;
		    		attributes.push_back(brics_3d::rsg::Attribute("gis:origin", "wgs84"));
		    		wm->scene.getNodes(attributes, ids);
		    		if (ids.size() > 0) {
		    			originId = ids[0];
		    		} else {
		    			LOG(WARNING) << name << "No gis origin found. Using the root node instead.";
		    			originId = wm->getRootNodeId(); // Fall back to root node
		    		}

		    		/* for each point of the area, get the pose (NOTE: this might be better cached) */
		    		vector<brics_3d::Point3D> polygonPoints;
		    		for(std::vector<brics_3d::rsg::Id>::const_iterator it = polygonIs.begin(); it != polygonIs.end(); ++it) {
		    			brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;
		    			wm->scene.getTransformForNode(*it, originId, wm->now(), transform);
		    			brics_3d::Point3D polygonPoint(transform->getRawData()[brics_3d::matrixEntry::x], transform->getRawData()[brics_3d::matrixEntry::y], 0);
		    			polygonPoints.push_back(polygonPoint);
		    			LOG(DEBUG) << name << "Polygon point: " << polygonPoint;
		    		}
		    		LOG(DEBUG) << name << "Polygon has: " << polygonPoints.size() << " points.";


			    	/* access band */
			    	GDALRasterBand  *poBand;
			    	float *pafScanline;
			    	poBand = demDataset->GetRasterBand(demBandIndex);


			    	/* read it */
			    	for (int yPixel = 0; yPixel < demMaxPixelSizeY; ++yPixel) { // line by line

						pafScanline = (float *) CPLMalloc(sizeof(float)*demMaxPixelSizeX); // no ownership
						poBand->RasterIO( GF_Read, 0, yPixel, demMaxPixelSizeX, 1,
								pafScanline, demMaxPixelSizeX, 1, poBand->GetRasterDataType(),
								0, 0 );

						for (int xPixel = 0; xPixel < demMaxPixelSizeX; ++xPixel) { // pixel by pixel
							//printf("(%i, %f),", xPixel , pafScanline[xPixel]);

							/* for each raster pixel check if it is within area */
							if(pixelInPolygon(polygonPoints, xPixel, yPixel)) {
								float elevation = pafScanline[xPixel];
								if(elevation <= minAreaElevation) {
									minAreaElevation = elevation;
									result = true; // at least once
								}
								if(elevation >= maxAreaElevation) {
									maxAreaElevation = elevation;
									result = true; // at least once
								}

							}
						}

						CPLFree(pafScanline);
					}

		    		outputModelAsJSON.Set("minElevation", minAreaElevation);
		    		outputModelAsJSON.Set("maxElevation", maxAreaElevation);

		    	} else {

		    		outputModelAsJSON.Set("minElevation", demMinElevation);
		    		outputModelAsJSON.Set("maxElevation", demMaxElevation);
		    		result = true;

		    		static bool convertToPointCloud = true;
		    		if(convertToPointCloud) {
		    			LOG(DEBUG) << name << " Converting DEM to a into a point cloud";

			    		brics_3d::rsg::Id originId;
			    		vector<brics_3d::rsg::Attribute> attributes;
			    		vector<brics_3d::rsg::Id> ids;
			    		attributes.push_back(brics_3d::rsg::Attribute("gis:origin", "wgs84"));
			    		wm->scene.getNodes(attributes, ids);
			    		if (ids.size() > 0) {
			    			originId = ids[0];
			    		} else {
			    			LOG(WARNING) << name << "No gis origin found. Using the root node instead.";
			    			originId = wm->getRootNodeId(); // Fall back to root node
			    		}

		    			/* prepare point cloud */
			    		brics_3d::rsg::Id pcId;
		    			brics_3d::PointCloud3D::PointCloud3DPtr pointCloud(new brics_3d::PointCloud3D());
		    			LOG(DEBUG) << name << " Size of point cloud : " << pointCloud->getSize();





		    	    	/* access band */
		    			GDALRasterBand  *poBand;
		    			unity_pixel_t *pafUnityScanline;
		    			float *pafScanline;
		    			poBand = demDataset->GetRasterBand(demBandIndex);

		    			/* read it */
		    			double x,y,z= 0;
		    			double elevation = 0;
		    			for (int yPixel = 0; yPixel < demMaxPixelSizeY; ++yPixel) { // line by line

		    				if(type.compare("UNITY") == 0) {
		    					pafUnityScanline = (unity_pixel_t *) CPLMalloc(sizeof(unity_pixel_t)*demMaxPixelSizeX); // no ownership
								poBand->RasterIO( GF_Read, 0, yPixel, demMaxPixelSizeX, 1,
										pafUnityScanline, demMaxPixelSizeX, 1, GDT_UInt16,
										0, 0 );
		    				} else {
			    				pafScanline = (float *) CPLMalloc(sizeof(float)*demMaxPixelSizeX); // no ownership
			    				poBand->RasterIO( GF_Read, 0, yPixel, demMaxPixelSizeX, 1,
			    						pafScanline, demMaxPixelSizeX, 1, poBand->GetRasterDataType(),
			    						0, 0 );
		    				}

		    				for (int xPixel = 0; xPixel < demMaxPixelSizeX; ++xPixel) { // pixel by pixel


		    			    	if(type.compare("UNITY") == 0) {
		    			    		float unityElevation = static_cast<float>(pafUnityScanline[xPixel]);
		    			    		unityElevation = unityElevation/256; // workaround for issue with  PNG driver
		    			    		elevation = unityMapToElevation(unityElevation);
//		    			    		LOG(DEBUG) << name << "\t UNITY elevation = " << unityElevation << " at pixel (" << xPixel << ", " << yPixel << ") scaled from " << static_cast<double>(pafUnityScanline[xPixel]);
		    			    	} else {
		    			    		elevation = static_cast<double>(pafScanline[xPixel]);
		    			    	}
		    					z = elevation;
		    					if(pixelToWorld(xPixel, yPixel, x, y)) {

		    						if ((yPixel % 50 == 0)) {// subsample
//		    							LOG(DEBUG) << name << "\t Point3D(" << x << "," << y << "," << z << ")";
		    							if (elevation >= 0 && elevation <= zRangeInMeters) {
		    								pointCloud->addPoint(brics_3d::Point3D(x/1000.0,y/1000.0,z/100.0));
		    							}
		    						}
		    					}

		    				}

		    				if(type.compare("UNITY") == 0) {
		    					CPLFree(pafUnityScanline);
		    				} else {
		    					CPLFree(pafScanline);
		    				}
		    			}


		    			LOG(INFO) << name << " Size of point cloud : " << pointCloud->getSize();
		    			brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		    			pcContainer->data = pointCloud;
		    			attributes.clear();
		    			attributes.push_back(brics_3d::rsg::Attribute("name","dem_point_cloud"));
		    			attributes.push_back(brics_3d::rsg::Attribute("producer","demloader"));
		    			wm->scene.addGeometricNode(originId, pcId, attributes, pcContainer, brics_3d::rsg::TimeStamp(0.0));

		    		    /* place  the camera for the visualizer, based on the last loaded node  */
		    		    brics_3d::rsg::Id camearaId;
		    		    vector<brics_3d::rsg::Attribute> cameraAttributes;
		    		    cameraAttributes.push_back(brics_3d::rsg::Attribute("osg:camera","home"));
//		    		    cameraAttributes.push_back(brics_3d::rsg::Attribute("tf:type","wgs84")); // we are in Cartesian space.
		    		    brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr poseOfCamera(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, x,y,z));
//		    		    wm->scene.addTransformNode(originId, camearaId, cameraAttributes, poseOfCamera, wm->now());

		    			convertToPointCloud = false; //at most once
		    		}
		    	}

		    	if(result) {
		    		outputModelAsJSON.Set("result", "ELEVATION_VALUE_EXISTS");
		    	} else {
		    		outputModelAsJSON.Set("result", "ELEVATION_QUERY_ARE_OUT_OF_BOUNDS");
		    	}


		    }

		} else {
			LOG(ERROR) << "DemLoader: Unknown command.";
		}


		/* prepare output */



		outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
		brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

		/* validate output */
		if (enableValidation && false) { //TODO update model
			if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
				LOG(ERROR) << "DemLoader: Model validation for output model failed: " << outputModel;
				return false;
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
    	//LOG(DEBUG) << name << "pixelToWorld: pixel at (" << xPixel << ", " << yPixel << ") is geolocated at (" << xGeo << ", "<< yGeo << ")";


		/* Coordinate to coordinate transform */
		//LOG(DEBUG) << name << "pixelToWorld: non-transformed geolocation at (" << xGeo << ", " << yGeo << ").";
		//OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(&oTargetSRS, &oSourceSRS);
		if( poCT != 0) {
			if(!poCT->Transform( 1, &xGeo, &yGeo )) { // NOTE: this is an in place modification
				LOG(ERROR) << name << "pixelToWorld: Transformation failed.";
				return false;
			}
		}
		//LOG(DEBUG) << name << "pixelToWorld: transformed geolocation at (" << xGeo << ", " << yGeo << ").";

		return true;
	}

	bool worldToPixel (double xGeo, double yGeo, int& xPixel, int& yPixel) {
		if(!demDataset) {
			LOG(ERROR) << name << "worldToPixel: DEM not loaded yet.";
			return false;
		}

		/* Coordinate to coordinate transform */
		LOG(DEBUG) << name << "worldToPixel: non-transformed geolocation at (" << xGeo << ", " << yGeo << ").";
		OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation( &oSourceSRS, &oTargetSRS );
		if(poCT != 0) {
				if(!poCT->Transform( 1, &xGeo, &yGeo )) { // NOTE: this is an in place modification
					LOG(ERROR) << name << "worldToPixel: Transformation failed.";
					return false;
				}
		}
		LOG(DEBUG) << name << "worldToPixel: transformed geolocation at (" << xGeo << ", " << yGeo << ").";

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
			int nXSize = poBand->GetXSize();
			int line = yPixel;

	    	if(type.compare("UNITY") == 0) {

				/* read it */
				unity_pixel_t *pafScanline;
				pafScanline = (unity_pixel_t *) CPLMalloc(sizeof(unity_pixel_t)*nXSize);
				poBand->RasterIO( GF_Read, 0, line, nXSize, 1,
						pafScanline, nXSize, 1, GDT_UInt16,
						0, 0 );
				elevation = static_cast<float>(pafScanline[xPixel]/256); // GDAL does not use the intended PNG driver; it deduces 16bit, which is not the case - it is 8bit
	    		elevation = unityMapToElevation(elevation);
	    		LOG(DEBUG) << name << "getElevationAt:\t Scaled to UNITY elevation = " << elevation << " at geolocation (" << xGeo << ", " << yGeo << ")";

				CPLFree(pafScanline);

	    	} else {
				float *pafScanline;
				pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize);
				poBand->RasterIO( GF_Read, 0, line, nXSize, 1,
						pafScanline, nXSize, 1, poBand->GetRasterDataType(),
						0, 0 );
		    	elevation = static_cast<float>(pafScanline[xPixel]);
		    	CPLFree(pafScanline);
	    	}

	    	LOG(DEBUG) << name << "getElevationAt: Found elevation value = " << elevation << " at geolocation (" << xGeo << ", " << yGeo << ")";
			resultMessage = "ELEVATION_VALUE_EXISTS";

			if(elevation <= globalMinElevation) {
				LOG(DEBUG) << name << "getElevationAt: Elevation value = " << elevation << " is invalid.";
				resultMessage = "ELEVATION_VALUE_INVALID";
				return false;
			}

		} else {
			resultMessage = "DEM_FILE_NOT_LOADED";
			return false;
		}

		return true;
	}

	bool isInArea(int xPixel, int yPixel, brics_3d::rsg::Id areaId) {
//	bool isInArea(int xPixel, int yPixel, vector<brics_3d::Point3D> polygonPoints) {
		bool isInArea =  false;


		double x = 0;
		double y = 0;
		pixelToWorld(xPixel, yPixel, x, y);

		std::vector<brics_3d::rsg::Id> polygonIs;
		if(!wm->scene.getConnectionTargetIds(areaId,  polygonIs)) {
			LOG(ERROR) << name << "::isInArea: no connection found.";
			return false; // no connection found
		}

		/* Get origin */
		brics_3d::rsg::Id originId;
		vector<brics_3d::rsg::Attribute> attributes;
		vector<brics_3d::rsg::Id> ids;
		attributes.push_back(brics_3d::rsg::Attribute("gis:origin", "wgs84"));
		wm->scene.getNodes(attributes, ids);
		if (ids.size() > 0) {
			originId = ids[0];
		} else {
			LOG(WARNING) << name << "No gis origin found. Using the root node instead.";
			originId = wm->getRootNodeId(); // Fall back to root node
		}

		/* for each point of the area, get the pose (NOTE: this might be better cached) */
		vector<brics_3d::Point3D> polygonPoints;
		for(std::vector<brics_3d::rsg::Id>::const_iterator it = polygonIs.begin(); it != polygonIs.end(); ++it) {
			brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;
			wm->scene.getTransformForNode(*it, originId, wm->now(), transform);
			brics_3d::Point3D polygonPoint(transform->getRawData()[brics_3d::matrixEntry::x], transform->getRawData()[brics_3d::matrixEntry::y], 0);
			polygonPoints.push_back(polygonPoint);
			LOG(DEBUG) << name << "::isInArea: polygon point: " << polygonPoint;
		}
		LOG(DEBUG) << name << "::isInArea: polygon has: " << polygonPoints.size() << " points.";

		LOG(DEBUG) << name << "::isInArea: testing node: (" << x << ", " << y << ")";
		isInArea = pointInPolygon(polygonPoints, x, y);
		if(isInArea) {
			LOG(DEBUG) << name << "::isInArea: node (" << x << ", " << y << ") is within polygon";
		}

		return isInArea;
	}

	bool pixelInPolygon(vector<brics_3d::Point3D>& polygonPoints, int  xPixel, int yPixel) {
		double xGeo = 0;
		double yGeo = 0;
		pixelToWorld(xPixel, yPixel, xGeo, yGeo);
		return pointInPolygon(polygonPoints, xGeo, yGeo);
	}

	bool pointInPolygon(vector<brics_3d::Point3D>& polygonPoints, double  x, double y) {

		  int   i;
		  int j=polygonPoints.size()-1;
		  bool  oddNodes=false;

		  for (i=0; i<polygonPoints.size(); i++) {
			  if (((polygonPoints[i].getY()<y) && (polygonPoints[j].getY())>=y)
					  ||  ((polygonPoints[j].getY()<y) && (polygonPoints[i].getY()>=y))) {
				  if (polygonPoints[i].getX()+(y-polygonPoints[i].getY())/(polygonPoints[j].getY()-polygonPoints[i].getY())*(polygonPoints[j].getX()-polygonPoints[i].getX())<x) {
					  oddNodes=!oddNodes;
				  }
			  }
			  j=i;
		  }

		  return oddNodes;
	}

	double unityMapToElevation(double unityElevation) {
//		return  unityElevation / std::numeric_limits<uint16_t>::max() * zRangeInMeters;
//		return  unityElevation * 256/*to*/ *  zRangeInMeters / 256;
		return unityElevation;
	}

	/* Meta data */
	std::string name;
	std::string modelsDefaultPath;
	std::string inputMetaModelFile;
	std::string outputMetaModelFile;

	/* Algorithm(s) */
	GDALDataset *demDataset;
	OGRSpatialReference oSourceSRS; // as derived from dataset
	OGRSpatialReference oTargetSRS; // Ours system here is WGS84
	OGRCoordinateTransformation* poCT; // source to target transformation

	/* Parameters */
	double geoTransform[6];
	double demMaxElevation;
	double demMinElevation;
	double globalMinElevation;
	int demMaxPixelSizeX;
	int demMaxPixelSizeY;
	int demBandIndex;
	string type;
	double xRangeInMeters;
	double yRangeInMeters;
	double zRangeInMeters;

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(DemLoader)
FUNCTION_BLOCK_DESTROY
