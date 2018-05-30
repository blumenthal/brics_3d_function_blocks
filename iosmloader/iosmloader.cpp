/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2018, Locomotec
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
 * @file iosmloader
 *
 * A function block to load an Indoor Open Street Map
 *
 */


/* std includes */
//#include <iostream>
#include <fstream>


/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/JSONGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/NodeHashTraverser.h>
#include <brics_3d/util/JSONTypecaster.h>

#include <brics_3d/core/HomogeneousMatrix44.h> // concrete type
#include <brics_3d/core/PointCloud3D.h>		// concrete type
#include <brics_3d/core/TriangleMeshImplicit.h>// concrete type
#include <brics_3d/core/TriangleMeshExplicit.h>// concrete type

/* Xerces includes for XML handling */
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

/* GPS utils*/
#include "../osmloader/GpsConversions.h"

using namespace std;
using brics_3d::Logger;
using namespace xercesc_3_1;

/***********FBX***************/


class IosmLoader : public brics_3d::rsg::IFunctionBlock {
public:

	IosmLoader(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "iosmloader";
		inputMetaModelFile = "fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */

	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;

	    parser = 0;
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


		/* Input data */
		string osmFile = inputModelAsJSON.Get("fileName").AsString();
		brics_3d::rsg::Id subgraphId = wm->getRootNodeId();
		bool convertToUtm = false;

		if(inputModelAsJSON.Contains("subgraphId")) { // optional
			subgraphId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "subgraphId");
		}
		if(inputModelAsJSON.Contains("convertToUtm")) { // optional
			convertToUtm = inputModelAsJSON.Get("convertToUtm").AsBool();
		}

		LOG(DEBUG) << name << ": [Input] fileName = " << osmFile;
		LOG(DEBUG) << name << ": [Input] subgraphId = " << subgraphId;
		LOG(DEBUG) << name << ": [Input] convertToUtm = " << convertToUtm;


		/* Execute loader */
		LoaderStatistics statistics;
		bool result = loadFromFile(osmFile, convertToUtm, wm->getRootNodeId(), statistics);
		LOG(DEBUG) << name << ": Map loaded.";



		/* prepare output */
		outputModelAsJSON.Set("numberOfNodes", statistics.numberOfNodes);
		outputModelAsJSON.Set("numberOfWays", statistics.numberOfWays);
		outputModelAsJSON.Set("numberOnumberOfWaysfNodes", statistics.numberOfRelations);


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

	~IosmLoader() {
		LOG(INFO) << name << ": Stopping block " << name;
	};


private:

	/* Meta data */
	std::string name;
	std::string modelsDefaultPath;
	std::string inputMetaModelFile;
	std::string outputMetaModelFile;

	XercesDOMParser* parser;

	struct LoaderStatistics {
		int numberOfNodes;
		int numberOfWays;
		int numberOfRelations;
	};

	bool loadFromFile(string fileName, bool convertToUtm, brics_3d::rsg::Id outputHookId, LoaderStatistics &statistics) {

		statistics.numberOfNodes = 0;
		statistics.numberOfWays = 0;
		statistics.numberOfRelations = 0;

		/* The origin */
		brics_3d::rsg::Id originId;
		std::vector<brics_3d::rsg::Attribute> originAttributes;
		if(convertToUtm) {
			originAttributes.push_back(brics_3d::rsg::Attribute("gis:origin","utm"));
		} else {
			originAttributes.push_back(brics_3d::rsg::Attribute("gis:origin","wgs84"));
		}

		/* check if it exists already */
		vector<brics_3d::rsg::Id> resultIds;
		wm->scene.getNodes(originAttributes, resultIds);
		if(resultIds.size() > 0) {
			if(resultIds.size() > 1) {
				LOG(INFO) << "iosmloader: Multiple origins found. Taking first one.";
			}
			originId = resultIds[0]; // We take the first one.
			LOG(INFO) << "iosmloader: Existing origin found with Id = " << originId;

		} else {
			LOG(INFO) << "iosmloader: Adding a new origin node";
			wm->scene.addGroup(outputHookId, originId, originAttributes);
		}




		/*
		 * get and set config data
		 */

//		std::string dataSetFolder = "/opt/src/sandbox/brics_3d_function_blocks/data";
		std::string dataSet = fileName;
	    LOG(INFO) << "iosmloader: Using file " << dataSet << " .";

	    /*
	     * define where to store results
	     */


		/*
		 * do computation
		 */
	    vector<brics_3d::rsg::Id> osmNodesIds;

	    /* Open xml file */
		try {
			XMLPlatformUtils::Initialize();
		}

		catch (const XMLException& e) {
			char *pMsg = XMLString::transcode(e.getMessage());
			cerr << "ERROR: An error occured during Xerces initialization.\n"
					<< "  Exception message:"
					<< pMsg;
			XMLString::release(&pMsg);
			return false;
		}

		//
		//  Create our parser, then attach an error handler to the parser.
		//  The parser will call back to methods of the ErrorHandler if it
		//  discovers errors during the course of parsing the XML document.
		//
		parser = new XercesDOMParser;
		parser->setValidationScheme(XercesDOMParser::Val_Auto);
		parser->setDoNamespaces(false);
		parser->setDoSchema(false);
		parser->setValidationSchemaFullChecking(false);
		parser->setCreateEntityReferenceNodes(false);

		//
		//  Parse the XML file, catching any XML exceptions that might propagate
		//  out of it.
		//
		bool errorsOccured = false;
		try {
			parser->parse(dataSet.c_str());
			int errorCount = parser->getErrorCount();
			if (errorCount > 0) {
				errorsOccured = true;
				LOG(ERROR) << "XML document has " << errorsOccured << " error(s).";
			}

		} catch (const XMLException& e) {
			LOG(ERROR) << "An error occured during parsing\n   Message: " << e.getMessage();
			errorsOccured = true;
		} catch (const DOMException& e) {
			LOG(ERROR) << "A DOM error occured during parsing\n   DOMException code: " << e.code;
			errorsOccured = true;
		} catch (...) {
			LOG(ERROR) << "An error occured during parsing\n" << endl;
			errorsOccured = true;
		}

	    LOG(INFO) << "iosmloader: Read file. Error: " << errorsOccured;
	    if(errorsOccured) {
	    	return false;
	    }

	    /* parse xml file */
	    DOMNode* current = 0;
	    DOMNode* currentChild = 0;
	    DOMNode* attributeNode = 0;
	    DOMNamedNodeMap* attributesList = 0;
	    DOMNamedNodeMap* childAttributesList = 0;
	    XMLCh* rootName = XMLString::transcode("osm");
	    XMLCh* nodeName = XMLString::transcode("node");
	    XMLCh* tagName = XMLString::transcode("tag");
	    XMLCh* kName = XMLString::transcode("k");
	    XMLCh* vName = XMLString::transcode("v");
	    XMLCh* idName = XMLString::transcode("id");
	    XMLCh* latName = XMLString::transcode("lat");
	    XMLCh* lonName = XMLString::transcode("lon");
	    XMLCh* wayName = XMLString::transcode("way");
	    XMLCh* refName = XMLString::transcode("ref");
	    XMLCh* versionName = XMLString::transcode("version");
	    string tmpResult;
	    string osmAttributePrefix = "osm:";
	    unsigned int nodeCounter = 0;
	    unsigned int wayCounter = 0;

	   /* <osm version="0.6" generator="Overpass API">
	    *  	<note>The data included in this document is from www.openstreetmap.org. The data is made available under ODbL.</note>
	    *  	<meta osm_base="2015-01-07T12:43:02Z"/>
	    *  	<node id="21101298" lat="50.7776577" lon="7.1853241"><tag k="crossing" v="traffic_signals"/>
	    * 		<tag k="highway" v="traffic_signals"/>
	    *  	</node>
	    * </osm>
	    */

	    DOMDocument* doc = parser->getDocument();

	    /* root node */
	    DOMNodeList* root = doc->getElementsByTagName(rootName);

	    if (root->getLength() > 1) {
	        LOG(WARNING) << "More than one osm elemnt found, taking the first one";
	    } else if(root->getLength() < 1) {
	    	LOG(ERROR) << "No osm elemnt found.";
			return false;
	    }

	    current = root->item(0);
	    attributesList =  current->getAttributes();
	    attributeNode = attributesList->getNamedItem(versionName);

	    if (attributeNode != 0) {
	    	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	    	LOG(INFO) << "osm version = " << tmpResult;
	    }

	    /* osm nodes */
	    DOMNodeList* osmNodes = doc->getElementsByTagName(nodeName);
		double x = -1.0;
		double y = -1.0;
		double z = 0.0;

	    for (unsigned int i = 0; i < osmNodes->getLength(); ++i) {
	    	unsigned int id = 0;
	    	double lon = -1.0;
	    	double lat = -1.0;
	    	vector<brics_3d::rsg::Attribute> tags;
	    	brics_3d::rsg::TimeStamp time = wm->now();

	        current = osmNodes->item(i);

	        attributesList =  current->getAttributes();

	        // Id
	        attributeNode = attributesList->getNamedItem(idName);
	        if (attributeNode != 0) {
	        	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        	LOG(DEBUG) << "node id = " << tmpResult;
	        	tags.push_back(brics_3d::rsg::Attribute(osmAttributePrefix+"node_id", tmpResult));
	        	id = atoi(tmpResult.c_str());
	        }

	        // lon
	        attributeNode = attributesList->getNamedItem(lonName);
	        if (attributeNode != 0) {
	        	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        	LOG(DEBUG) << "\t node lon = " << tmpResult;
	        	lon = atof(tmpResult.c_str());
	        }

	        // lat
	        attributeNode = attributesList->getNamedItem(latName);
	        if (attributeNode != 0) {
	        	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        	LOG(DEBUG) << "\t node lat = " << tmpResult;
	        	lat = atof(tmpResult.c_str());
	        }

	        //tags are in the child nodes
	        DOMNodeList* childs = current->getChildNodes();

	        for (unsigned int j = 0; j < childs->getLength(); ++j) {
	        	currentChild = childs->item(j);
	        	childAttributesList =  currentChild->getAttributes();

	        	tmpResult = XMLString::transcode(currentChild->getNodeName());
	//        	LOG(DEBUG) << "\t childName = " << tmpResult;
	        	if(tmpResult.compare("tag") == 0) {
	//        		LOG(DEBUG) << "\t found a tag: ";
	        		brics_3d::rsg::Attribute tag;

	        		// k
	        		attributeNode = childAttributesList->getNamedItem(kName);
	        		if (attributeNode != 0) {
	        			tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        			LOG(DEBUG) << "\t\t node k = " << tmpResult;
	        			tag.key = osmAttributePrefix+tmpResult;
	        		} else {
	        			continue;
	        		}

	        		// v
	        		attributeNode = childAttributesList->getNamedItem(vName);
	        		if (attributeNode != 0) {
	        			tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        			LOG(DEBUG) << "\t\t node v = " << tmpResult;
	        			tag.value = tmpResult;
	        		} else {
	        			continue;
	        		}

	        		tags.push_back(tag);
	        	}
	        }

	        string zone;
	        if(convertToUtm) {
	        	UTM::LLtoUTM(lon, lat, x, y, zone);
	        } else {
	        	x = lat;
	        	y = lon;
	        }
	        LOG(DEBUG) << "Pose = (" << x << ", " << y << ", " << z << ") in zone: " << zone;



	        /* Add to RSG */
	        brics_3d::rsg::Id poseId;
	        brics_3d::rsg::Id nodeId = id;

	    	vector<brics_3d::rsg::Attribute> poseAttributes;
	    	poseAttributes.push_back(brics_3d::rsg::Attribute(osmAttributePrefix + "tf","pose"));
	    	if(convertToUtm) {
	        	poseAttributes.push_back(brics_3d::rsg::Attribute("tf:type","utm"));
	        	poseAttributes.push_back(brics_3d::rsg::Attribute("tf:utm_zone", zone));
	    	} else {
	        	poseAttributes.push_back(brics_3d::rsg::Attribute("tf:type","wgs84"));
	    	}
	        brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr poseOfNode(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, x,y,z));
	        wm->scene.addTransformNode(originId, poseId, poseAttributes, poseOfNode, time);
//	        output.push_back(poseId);

	        wm->scene.addNode(poseId, nodeId, tags, true);
//	        output.push_back(nodeId);

	        nodeCounter++;
		}

	    LOG(INFO) << "iosmloader: " << nodeCounter <<" nodes loaded.";


	    /* place  the camera for the visualizer, based on the last loaded node  */
	    brics_3d::rsg::Id camearaId;
	    vector<brics_3d::rsg::Attribute> cameraAttributes;
	    cameraAttributes.push_back(brics_3d::rsg::Attribute("osg:camera","home"));
	    cameraAttributes.push_back(brics_3d::rsg::Attribute("tf:type","wgs84"));
	    brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr poseOfCamera(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, x,y,z));
	    wm->scene.addTransformNode(originId, camearaId, cameraAttributes, poseOfCamera, wm->now());
	    //output.push_back(camearaId);


	    /* osm ways */

	    /* e.g.
	     * <way id="139281027">
	     *		<nd ref="1526916568"/>
	     *		<nd ref="2280218902"/>
	     *		<nd ref="1526916486"/>
	     *		<tag k="highway" v="residential"/>
	     *		<tag k="maxspeed" v="30"/>
	     *  	<tag k="name" v="Südstraße"/>
	     * </way>
	     */
	    DOMNodeList* wayNodes = doc->getElementsByTagName(wayName);

	    for (unsigned int i = 0; i < wayNodes->getLength(); ++i) {
	    	unsigned int id = 0;

	        current = wayNodes->item(i);

	        attributesList =  current->getAttributes();
	        attributeNode = attributesList->getNamedItem(idName);
	    	vector<brics_3d::rsg::Attribute> tags;
	    	vector<brics_3d::rsg::Id> nodeReferences;

	        // id
	        if (attributeNode != 0) {
	        	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	        	LOG(DEBUG) << "way id = " << tmpResult;
	        	tags.push_back(brics_3d::rsg::Attribute(osmAttributePrefix+"way_id", tmpResult));
	        	id = atoi(tmpResult.c_str());
	        }

	        /*
	         * check children for tags node references
	         */
	        DOMNodeList* childs = current->getChildNodes();


	         for (unsigned int j = 0; j < childs->getLength(); ++j) {
	         	currentChild = childs->item(j);
	         	childAttributesList =  currentChild->getAttributes();


	         	tmpResult = XMLString::transcode(currentChild->getNodeName());
	         	LOG(DEBUG) << "\t childName = " << tmpResult;
	         	if(tmpResult.compare("tag") == 0) {
	         		LOG(DEBUG) << "\t found a tag: ";
	         		brics_3d::rsg::Attribute tag;

	         		// k
	         		attributeNode = childAttributesList->getNamedItem(kName);
	         		if (attributeNode != 0) {
	         			tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	         			LOG(DEBUG) << "\t\t node k = " << tmpResult;
	         			tag.key = osmAttributePrefix+tmpResult;
	         		} else {
	         			continue;
	         		}

	         		// v
	         		attributeNode = childAttributesList->getNamedItem(vName);
	         		if (attributeNode != 0) {
	         			tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	         			LOG(DEBUG) << "\t\t node v = " << tmpResult;
	         			tag.value = tmpResult;
	         		} else {
	         			continue;
	         		}

	         		tags.push_back(tag);
	         	} else if (tmpResult.compare("nd") == 0) {
	           		LOG(DEBUG) << "\t found a reference: ";
	         		unsigned int tmpId;

	         		// ref
	         		attributeNode = childAttributesList->getNamedItem(refName);
	         		if (attributeNode != 0) {
	         			tmpResult = XMLString::transcode(attributeNode->getNodeValue());
	         			LOG(DEBUG) << "\t\t node ref = " << tmpResult;
	         			tmpId = atoi(tmpResult.c_str());
	         		} else {
	         			continue;
	         		}

	         		brics_3d::rsg::Id refId = tmpId;
	         		nodeReferences.push_back(refId);
	         	}
	         }

	         /* Add to RSG as Connection */
	         brics_3d::rsg::Id wayId = id; //TODO add mask
	     	 vector<brics_3d::rsg::Id> emptyList;
	     	 LOG(DEBUG) << "Adding Connection with ID " << id << ", containing " << nodeReferences.size() << " references.";
	         wm->scene.addConnection(originId, wayId, tags, emptyList, nodeReferences , wm->now(), wm-> now(), true);
	         wayCounter++;

	        /* Add a mesh as visualization of the connection, NOTE: this is static */
	         if(nodeReferences.size() >= 2) {
	        	 brics_3d::rsg::Id currentNode = nodeReferences[1];
	        	 brics_3d::rsg::Id lastNode = nodeReferences[0];
	        	 brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTf(new brics_3d::HomogeneousMatrix44());

	        	 brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
	        	 brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	        	 newMeshContainer->data = newMesh;

	        	 for (unsigned int i = 1; i < nodeReferences.size(); ++i) {
	        		 currentNode = nodeReferences[i];
	        		 double x1, y1, x2, y2;
	        		 if( wm->scene.getTransformForNode(lastNode, originId, wm->now(), resultTf)) {
	        			 x1 = resultTf->getRawData()[brics_3d::matrixEntry::x];
	        			 y1 = resultTf->getRawData()[brics_3d::matrixEntry::y];
	        		 } else {
	        			 continue;
	        		 }

	        		 if( wm->scene.getTransformForNode(currentNode, originId, wm->now(), resultTf)) {
	        			 x2 = resultTf->getRawData()[brics_3d::matrixEntry::x];
	        			 y2 = resultTf->getRawData()[brics_3d::matrixEntry::y];
	        		 } else {
	        			 continue;
	        		 }
	        		 lastNode = currentNode;

	        		 LOG(DEBUG) << "Segment:" << x1 << ", " << y1 << ", " << x2 << ", " << y2;
	        		 double yOffset = -0.1; //m
	        		 if(!convertToUtm) {
	        			 yOffset *= 0.000014; // delta in WGS 84
	        		 }
	        		 newMesh->addTriangle(brics_3d::Point3D(x1,y1,0), brics_3d::Point3D(x1,y1+yOffset,0), brics_3d::Point3D(x2,y2,0) );
	        	 }

	        	 LOG(DEBUG) << "Adding a mesh with "  << newMesh->getSize() << " triangles to visualize a way.";
	        	 brics_3d::rsg::Id meshId;
	        	 vector<brics_3d::rsg::Attribute> meshAttributes;
	        	 meshAttributes.push_back(brics_3d::rsg::Attribute("geo:crs","wgs84"));
	        	 meshAttributes.push_back(brics_3d::rsg::Attribute("osm:dbg","way_mesh"));
	        	 wm->scene.addGeometricNode(originId, meshId, meshAttributes, newMeshContainer, wm->now());

	         }
	    }

	    LOG(INFO) << "iosmloader: " << wayCounter <<" ways loaded.";

	    /* clean up xml dom */
	    XMLString::release(&nodeName);
	    XMLString::release(&tagName);
	    XMLString::release(&wayName );
	    XMLString::release(&kName);
	    XMLString::release(&vName);
	    XMLString::release(&idName);
	    XMLString::release(&latName );
	    XMLString::release(&lonName);
	    XMLString::release(& wayName);
	    XMLString::release(&refName);
	    XMLString::release(&versionName);

	    delete parser;

		return true;
	}
};

/******************************************************************************************/


///* configuration
// * upon cloning the following happens:
// *   - value.type is resolved
// *   - value.data will point to a buffer of size value.len*value.type->size
// *
// * if an array is required, then .value = { .len=<LENGTH> } can be used.
// */
//ubx_config_t iosmloader_config[] = {
//	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
//    { .name="map_file", .type_name = "char" , .doc="OSM file name to be loaded to RSG." },
//    { .name="convert_to_utm", .type_name = "int", .doc="If true every pose will be converted into the UTM representation." },
//    { .name="log_level", .type_name = "int", .doc="Set the log level: LOGDEBUG = 0, INFO = 1, WARNING = 2, LOGERROR = 3, FATAL = 4" },
//	{ NULL },
//};
//
//ubx_port_t iosmloader_ports[] = {
//	{ .name="inputDataIds", .attrs=PORT_DIR_IN, .in_type_name="struct rsg_ids"},
//	{ .name="outputDataIds", .attrs=PORT_DIR_OUT, .out_type_name="struct rsg_ids"},
//	{ NULL },
//};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(IosmLoader)
FUNCTION_BLOCK_DESTROY




