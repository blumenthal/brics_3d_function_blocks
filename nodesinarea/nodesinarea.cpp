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
 * @file nodesinarea
 *
 * A function block to all retrieve nodes that are within an
 * area as specified by an area Connection.
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

using namespace brics_3d;
using namespace brics_3d::rsg;

/***********FBX***************/

class NodesInArea: public brics_3d::rsg::IFunctionBlock {
public:

	NodesInArea(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "nodesinarea";
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

	~NodesInArea(){
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
				LOG(ERROR) << "NodesInArea: Model validation for input model failed: " << outputModel;
				return false;
			}
		}



		/* get data */
		brics_3d::rsg::Id areaId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "areaId");
		std::vector<brics_3d::rsg::Attribute> attributes = brics_3d::rsg::JSONTypecaster::getAttributesFromJSON(inputModelAsJSON);

		/* output data */
		std::vector<brics_3d::rsg::Id> nodeIdsInArea;

		/* execute query
		 *
		 */
		std::vector<brics_3d::rsg::Id> nodesWithSpecifiedAttributes;
		if(attributes.size() == 0) { // if no attributes are specified, then take all nodes
			attributes.push_back(brics_3d::rsg::Attribute("*","*"));
		}
		wm->scene.getNodes(attributes, nodesWithSpecifiedAttributes);

		/* for each node, check if it is within the area */
		for(std::vector<brics_3d::rsg::Id>::const_iterator it = nodesWithSpecifiedAttributes.begin(); it != nodesWithSpecifiedAttributes.end(); ++it) {
			if(isInArea(*it, areaId)) {
				nodeIdsInArea.push_back(*it);
			}
		}


		/* prepare output */
		bool result = true;

		brics_3d::rsg::JSONTypecaster::addIdsToJSON(nodeIdsInArea, outputModelAsJSON, "ids");

		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation /* && false */) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "NodesInArea: Model validation for output model failed: " << outputModel;
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
	bool isInArea(brics_3d::rsg::Id id, brics_3d::rsg::Id areaId) {
		bool isInArea =  false;

		/* */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;
		wm->scene.getTransformForNode(id, wm->getRootNodeId(), wm->now(), transform); //TODO reference ID! for now it is the root node

		double x = transform->getRawData()[matrixEntry::x];
		double y = transform->getRawData()[matrixEntry::y];

		std::vector<brics_3d::rsg::Id> polygonIs;
		if(!wm->scene.getConnectionTargetIds(areaId,  polygonIs)) {
			LOG(ERROR) << name << "::isInArea: no connection found.";
			return false; // no connection found
		}

		/* for each point of the area, get the pose (NOTE: this might be better cached) */
		vector<brics_3d::Point3D> polygonPoints;
		for(std::vector<brics_3d::rsg::Id>::const_iterator it = polygonIs.begin(); it != polygonIs.end(); ++it) {
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;
			wm->scene.getTransformForNode(*it, wm->getRootNodeId(), wm->now(), transform); //TODO reference ID! for now it is the root node
			Point3D polygonPoint(transform->getRawData()[matrixEntry::x], transform->getRawData()[matrixEntry::y], 0);
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

	// Take from: http://alienryderflex.com/polygon/
	//  Globals which should be set before calling this function:
	//
	//  int    polyCorners  =  how many corners the polygon has (no repeats)
	//  float  polyX[]      =  horizontal coordinates of corners
	//  float  polyY[]      =  vertical coordinates of corners
	//  float  x, y         =  point to be tested
	//
	//  (Globals are used in this example for purposes of speed.  Change as
	//  desired.)
	//
	//  The function will return YES if the point x,y is inside the polygon, or
	//  NO if it is not.  If the point is exactly on the edge of the polygon,
	//  then the function may return YES or NO.
	//
	//  Note that division by zero is avoided because the division is protected
	//  by the "if" clause which surrounds it.

	bool pointInPolygon(vector<brics_3d::Point3D> polygonPoints, double  x, double y) {

	  unsigned int   i;
	  int j=polygonPoints.size()-1;
	  bool  oddNodes=false;

//	  for (i=0; i<polyCorners; i++) {
//	    if (polyY[i]<y && polyY[j]>=y
//	    ||  polyY[j]<y && polyY[i]>=y) {
//	      if (polyX[i]+(y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x) {
//	        oddNodes=!oddNodes; }}
//	    j=i; }

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

	/* Parameters */

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(NodesInArea)
FUNCTION_BLOCK_DESTROY

