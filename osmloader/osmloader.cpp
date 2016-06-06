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
 * @file osmloader
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
#include <brics_3d/core/TriangleMeshImplicit.h>// concrete type
#include <brics_3d/core/TriangleMeshExplicit.h>// concrete type

/* Xerces includes for XML handling */
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

/* GPS utils*/
#include "GpsConversions.h"


UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)

using namespace std;
using brics_3d::Logger;
using namespace xercesc_3_1;

brics_3d::WorldModel* wmHandle = 0;
brics_3d::rsg::DotGraphGenerator* wmPrinter = 0;
brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr center;

std::string* fileName;
bool convertToUtm;

/* XML DOM representation */
XercesDOMParser* parser;



/* function block meta-data
 * used by higher level functions.
 */
char osmloader_meta[] =
	"{ doc='A function block to load Open Street Map (OSM) data from a file.',"
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
ubx_config_t osmloader_config[] = {
	{ .name="wm_handle", .type_name = "struct rsg_wm_handle", .doc="Handle to the world wodel instance. This parameter is mandatory."},
    { .name="map_file", .type_name = "char" , .doc="OSM file name to be loaded to RSG." },
    { .name="convert_to_utm", .type_name = "int", .doc="If true every pose will be converted into the UTM representation." },
    { .name="log_level", .type_name = "int", .doc="Set the log level: LOGDEBUG = 0, INFO = 1, WARNING = 2, LOGERROR = 3, FATAL = 4" },
	{ NULL },
};

ubx_port_t osmloader_ports[] = {
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

static int osmloader_init(ubx_block_t *c)
{
	LOG(INFO) << "osmloader: initializing: " << c->name;
//	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::INFO);
	brics_3d::Logger::setLogfile("osmloader.log");

	unsigned int clen;
	rsg_wm_handle tmpWmHandle =  *((rsg_wm_handle*) ubx_config_get_data_ptr(c, "wm_handle", &clen));
	assert(clen != 0);
	wmHandle = reinterpret_cast<brics_3d::WorldModel*>(tmpWmHandle.wm); // We know that this pointer stores the world model type
	if(wmHandle == 0) {
		LOG(FATAL) << "osmloader: World model handle could not be initialized.";
		return -1;
	}

	/* retrive optional dot file prefix from config */
	fileName = new std::string("map.osm");
	char* chrptr = (char*) ubx_config_get_data_ptr(c, "map_file", &clen);
	if(clen == 0) {
		LOG(INFO) << "osmloader: No map_file configuation given. Selecting a default name.";
	} else {
		if(strcmp(chrptr, "")==0) {
			LOG(INFO) << "osmloader: map_file is empty. Selecting a default name.";
		} else {
			std::string map_file(chrptr);
			*fileName = map_file;
		}
	}
	LOG(DEBUG) << "osmloader: map_file = " << *fileName;

	convertToUtm = true;
	int* convert_to_utm =  ((int*) ubx_config_get_data_ptr(c, "convert_to_utm", &clen));
	if(clen == 0) {
		LOG(INFO) << "osmloader: No convert_to_utm configuation given. Turned on by default.";
	} else {
		if (*convert_to_utm == 1) {
			LOG(INFO) << "osmloader: convert_to_utm turned on.";
			convertToUtm = true;
		} else {
			LOG(INFO) << "osmloader: convert_to_utm turned off.";
			convertToUtm = false;
		}
	}
	LOG(DEBUG) << "osmloader: convertToUtm = " << convertToUtm;

	wmPrinter = new brics_3d::rsg::DotGraphGenerator();

	return 0;
}


static void osmloader_cleanup(ubx_block_t *c)
{
	if (wmPrinter != 0) {
		delete wmPrinter;
		wmPrinter = 0;
	}

	LOG(INFO) << "osmloader: cleaning up: " << c->name;
}

static int osmloader_start(ubx_block_t *c)
{
	LOG(INFO) << "osmloader: starting. " << c->name;

	/* Set logger level */
	unsigned int clen;
	int* log_level =  ((int*) ubx_config_get_data_ptr(c, "log_level", &clen));
	if(clen == 0) {
		LOG(INFO) << "osmloader: No log_level configuation given.";
	} else {
		if (*log_level == 0) {
			LOG(INFO) << "osmloader: log_level set to DEBUG level.";
			brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
		} else if (*log_level == 1) {
			LOG(INFO) << "osmloader: log_level set to INFO level.";
			brics_3d::Logger::setMinLoglevel(brics_3d::Logger::INFO);
		} else if (*log_level == 2) {
			LOG(INFO) << "osmloader: log_level set to WARNING level.";
			brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
		} else if (*log_level == 3) {
			LOG(INFO) << "osmloader: log_level set to LOGERROR level.";
			brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGERROR);
		} else if (*log_level == 4) {
			LOG(INFO) << "osmloader: log_level set to FATAL level.";
			brics_3d::Logger::setMinLoglevel(brics_3d::Logger::FATAL);
		} else {
			LOG(INFO) << "osmloader: unknown log_level = " << *log_level;		}
	}

	return 0; /* Ok */
}

static void osmloader_step(ubx_block_t *c) {
	LOG(INFO) << "osmloader: executing: " << c->name;

	/* Just print what the world model has to offer. */
	wmPrinter->reset();
	wmHandle->scene.executeGraphTraverser(wmPrinter, wmHandle->getRootNodeId());
	LOG(DEBUG) << "osmloader: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

	/* read Id(s) from input port */
	std::vector<brics_3d::rsg::Id> inputDataIds;
	inputDataIds.clear();
	ubx_port_t* inputPort = ubx_port_get(c, "inputDataIds");
	rsg_ids recievedInputDataIs;
	recievedInputDataIs.numberOfIds = 0u;

	int ret = read_rsg_ids(inputPort, &recievedInputDataIs);
	if (ret < 1) {
		LOG(WARNING) << "osmloader: No input IDs given.";
	}

	brics_3d::rsg::Id outputHookId;
	brics_3d::rsg::UbxTypecaster::convertIdsFromUbx(recievedInputDataIs, inputDataIds);
	if (inputDataIds.size() < 1) {
		LOG(WARNING) << "osmloader: Not enough IDs specified. Expected 1 but it is: " << inputDataIds.size() << std::endl << "Using root instead.";
		outputHookId = wmHandle->getRootNodeId();
	} else {
		outputHookId = inputDataIds[0]; // First ID is always the output hook.
	}

	std::vector<brics_3d::rsg::Id> output;
	output.clear();
	output.push_back(outputHookId); // First ID is always the output hook.


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
	wmHandle->scene.getNodes(originAttributes, resultIds);
	if(resultIds.size() > 0) {
		if(resultIds.size() > 1) {
			LOG(INFO) << "osmloader: Multiple origins found. Taking first one.";
		}
		originId = resultIds[0]; // We take the first one.
		LOG(INFO) << "osmloader: Existing origin found with Id = " << originId;

	} else {
		LOG(INFO) << "osmloader: Adding a new origin node";
		wmHandle->scene.addGroup(outputHookId, originId, originAttributes);
	}




	/*
	 * get and set config data
	 */

	std::string dataSetFolder = "/opt/src/sandbox/brics_3d_function_blocks/data";
	std::string dataSet = *fileName;
    LOG(INFO) << "osmloader: Using file " << dataSet << " .";

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
		return;
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

    LOG(INFO) << "osmloader: Read file. Error: " << errorsOccured;
    if(errorsOccured) {
    	return;
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
		return ;
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

    for (int i = 0; i < osmNodes->getLength(); ++i) {
    	unsigned int id = 0;
    	double lon = -1.0;
    	double lat = -1.0;
    	vector<brics_3d::rsg::Attribute> tags;
    	brics_3d::rsg::TimeStamp time = wmHandle->now();

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

        for (int j = 0; j < childs->getLength(); ++j) {
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
        wmHandle->scene.addTransformNode(originId, poseId, poseAttributes, poseOfNode, time);
        output.push_back(poseId);

        wmHandle->scene.addNode(poseId, nodeId, tags, true);
        output.push_back(nodeId);

        nodeCounter++;
	}

    LOG(INFO) << "osmloader: " << nodeCounter <<" nodes loaded.";


    /* place  the camera for the visualizer, based on the last loaded node  */
    brics_3d::rsg::Id camearaId;
    vector<brics_3d::rsg::Attribute> cameraAttributes;
    cameraAttributes.push_back(brics_3d::rsg::Attribute("osg:camera","home"));
    cameraAttributes.push_back(brics_3d::rsg::Attribute("tf:type","wgs84"));
    brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr poseOfCamera(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, x,y,z));
    wmHandle->scene.addTransformNode(originId, camearaId, cameraAttributes, poseOfCamera, wmHandle->now());
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

    for (int i = 0; i < wayNodes->getLength(); ++i) {
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


         for (int j = 0; j < childs->getLength(); ++j) {
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
         wmHandle->scene.addConnection(originId, wayId, tags, emptyList, nodeReferences , wmHandle->now(), wmHandle-> now(), true);
         wayCounter++;

        /* Add a mesh as visualization of the connection, NOTE: this is static */
         if(nodeReferences.size() >= 2) {
        	 brics_3d::rsg::Id currentNode = nodeReferences[1];
        	 brics_3d::rsg::Id lastNode = nodeReferences[0];
        	 brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTf(new brics_3d::HomogeneousMatrix44());

        	 brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
        	 brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
        	 newMeshContainer->data = newMesh;

        	 for (int i = 1; i < nodeReferences.size(); ++i) {
        		 currentNode = nodeReferences[i];
        		 double x1, y1, x2, y2;
        		 if( wmHandle->scene.getTransformForNode(lastNode, originId, wmHandle->now(), resultTf)) {
        			 x1 = resultTf->getRawData()[brics_3d::matrixEntry::x];
        			 y1 = resultTf->getRawData()[brics_3d::matrixEntry::y];
        		 } else {
        			 continue;
        		 }

        		 if( wmHandle->scene.getTransformForNode(currentNode, originId, wmHandle->now(), resultTf)) {
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
        	 wmHandle->scene.addGeometricNode(originId, meshId, meshAttributes, newMeshContainer, wmHandle->now());

         }
    }

    LOG(INFO) << "osmloader: " << wayCounter <<" ways loaded.";

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




	/* push output to microblx */
	ubx_port_t* outputPort = ubx_port_get(c, "outputDataIds");
	rsg_ids toBeSendOutputDataIs;
	toBeSendOutputDataIs.numberOfIds = 0u;
	brics_3d::rsg::UbxTypecaster::convertIdsToUbx(output, toBeSendOutputDataIs);
	write_rsg_ids(outputPort, &toBeSendOutputDataIs);

}


/* put everything together */
ubx_block_t osmloader_comp = {
	.name = "osmloader/osmloader",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = osmloader_meta,
	.configs = osmloader_config,
	.ports = osmloader_ports,

	/* ops */
	.init = osmloader_init,
	.start = osmloader_start,
	.step = osmloader_step,
	.cleanup = osmloader_cleanup,
};

static int osmloader_init(ubx_node_info_t* ni)
{
	LOG(DEBUG) << "osmloader_init(ubx_node_info_t* ni)";
	return ubx_block_register(ni, &osmloader_comp);
}

static void osmloader_cleanup(ubx_node_info_t *ni)
{
	LOG(DEBUG) << "osmloader_cleanup(ubx_node_info_t *ni)";
	ubx_block_unregister(ni, "osmloader/osmloader");
}

UBX_MODULE_INIT(osmloader_init)
UBX_MODULE_CLEANUP(osmloader_cleanup)
