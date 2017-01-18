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
 * @file oncreate
 *
 * A monitor function block that sends an event whenever a new Atom (Node, Group, Connection, ...)  with a particular set of attributes is created.
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
#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/IPort.h>
#include <brics_3d/util/JSONTypecaster.h>

using namespace brics_3d::rsg;
using namespace brics_3d;

/* Definition of a single listener */
struct ListenerSpec {
	Id nodeId;
	Id monitorId;
	bool isStarted;
	std::vector<Attribute> attributes;
};

/*
 * Custom observer
 */
class OnCreateObserver: public ISceneGraphUpdateObserver {

public:
	OnCreateObserver() {
		port = 0;
	}

	virtual ~OnCreateObserver() {}

	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		return handleCreation(attributes, assignedId, "Node");
	}

	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		return handleCreation(attributes, assignedId, "Group");
	}

	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forceId = false) {
		return handleCreation(attributes, assignedId, "Transform");
	}

	bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forceId = false) {
		return handleCreation(attributes, assignedId, "UncertainTransform");
	}

	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forceId = false) {
		return handleCreation(attributes, assignedId, "GeometricNode");
	}

    bool addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
    	return handleCreation(attributes, rootId, "RemoteRootNode");
    }

    bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false) {
    	return handleCreation(attributes, assignedId, "Connection");
    }

	bool setNodeAttributes(Id id, vector<Attribute> newAttributes, TimeStamp timeStamp = TimeStamp(0)) {
		return true;
	}

	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
		return true;
	}

	bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp) {
		return true;
	}

	bool deleteNode(Id id) {
		return true;
	}

	bool addParent(Id id, Id parentId) {
		return true;
	}

	bool removeParent(Id id, Id parentId) {
		return true;
	}

	bool handleCreation(vector<Attribute> attributes, Id id, string type) {

		/* Check for each registered listener */
		for(std::map<Id, ListenerSpec>::iterator monitorsIt = listeners.begin(); monitorsIt != listeners.end(); ++monitorsIt) {
			listener = monitorsIt->second;

			/* check if all attributes of that particular listener match */
			bool attributesMatch = false;
			for (unsigned int i = 0; i < static_cast<unsigned int>(listener.attributes.size()); ++i) {
				attributesMatch = false;
				if (attributeListContainsAttribute(attributes, listener.attributes[i]) == false) {
					break;
				}
				attributesMatch = true; // Returns true, only when all attributes of monitor match.
			}


			/* If it matches, send a monitor message */
			if (attributesMatch) {

				if(port) { // We are better careful, if a port exists.

					libvariant::Variant result;

					/* Header */
					result.Set("@worldmodeltype", libvariant::Variant("RSGMonitor"));
					brics_3d::rsg::JSONTypecaster::addIdToJSON(listener.monitorId, result, "monitorId");
					// time stamp?

					/* Specific payload */
					brics_3d::rsg::JSONTypecaster::addIdToJSON(id, result, "id"); 			// id of the freshly created node.
					brics_3d::rsg::JSONTypecaster::addAttributesToJSON(attributes, result);	// send full attributes list (for debugging)
					string message = libvariant::Serialize(result, libvariant::SERIALIZE_JSON);

					LOG(DEBUG) << "OnCreateObserver: Sending message: " << message;
					int transferredBytes = 0;
					port->write(message.c_str(), message.size(), transferredBytes);

				}  else {
					LOG(WARNING) << "OnCreateObserver: no monitor port set.";
				}

			}

		}
		return true;
	}

	ListenerSpec listener;

	/// Look up table for all registered listeners. Identified via monitorId
    std::map<Id, ListenerSpec> listeners;

    IOutputPort* port;
};

/***********FBX***************/


class OnCreate: public brics_3d::rsg::IFunctionBlock {
public:

	OnCreate(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "oncreate";
		inputMetaModelFile = "fbx-" + name + "-input-schema.json";
		outputMetaModelFile= "fbx-" + name + "-output-schema.json";

		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Initializing block " << name  << " Build: " << __DATE__ << " " __TIME__;


		/* init algorithm */


	    /* get default location of model schemas */
	    char defaultFilename[255] = { FBX_MODELS_DIR };
	    modelsDefaultPath = defaultFilename;
	    LOG(DEBUG) << name << ": models default path = " << modelsDefaultPath;

	    monitor = 0;
	};

	~OnCreate(){
		LOG(INFO) << name << ": Stopping block " << name;

		if(monitor) {
			if(wm->scene.detachUpdateObserver(monitor)) {
				delete monitor;
				monitor = 0;
			} else {
				LOG(ERROR) << name << ": Can not detach OnCreateObserver.";
			}
		}
	};

	bool configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";

		return true;
	}

	bool execute(std::vector<brics_3d::rsg::Id>& inputDataIds, std::vector<brics_3d::rsg::Id>& outputDataIds) {
		LOG(WARNING) << name << " Cannot execute Id based i/o as this is a monitor function block";
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
				LOG(ERROR) << "OnCreate: Model validation for input model failed: " << outputModel;
				return false;
			}
		}

		/* get data */
		ListenerSpec listener;
		listener.monitorId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "monitorId");
		string monitorOperation = inputModelAsJSON.Get("monitorOperation").AsString();
		listener.isStarted = false;


		/* output data */
		bool result = true;

		/* execute query
		 *
		 */
		if (monitorOperation.compare("REGISTER") == 0) {
			LOG(DEBUG) << name << ": REGISTER operation found.";

			if (monitor == 0) { // only attach on first creation
				monitor = new OnCreateObserver();
				monitor->port = wm->scene.getMonitorPort();
				wm->scene.attachUpdateObserver(monitor);
			}

			listener.nodeId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "id");
			listener.attributes =  brics_3d::rsg::JSONTypecaster::getAttributesFromJSON(inputModelAsJSON);

			if (monitor != 0 ) {

				/* If it exists, update spec */
				map<Id, ListenerSpec>::iterator monitorIt = monitor->listeners.find(listener.monitorId);
				if(monitorIt != monitor->listeners.end()) {
					monitorIt->second = listener;
					LOG(INFO) << name << " Updating specification of existing monitor with id = " << listener.monitorId;
				} else {
					monitor->listeners.insert(std::make_pair(listener.monitorId, listener));
					LOG(INFO)  << name << " Added new monitor with id = " << listener.monitorId;
					LOG(DEBUG) << name << " There are now "<< monitor->listeners.size() << " listening monitors registered";
				}

			}
		} else if (monitorOperation.compare("START") == 0) {
				LOG(DEBUG) << name << ": START operation found.";

				if (monitor != 0 ) {
					map<Id, ListenerSpec>::iterator monitorIt = monitor->listeners.find(listener.monitorId);
					if(monitorIt != monitor->listeners.end()) {
						monitorIt->second.isStarted = true;
						LOG(INFO) << name << " Starting  monitor with id = " << listener.monitorId;
					} else {
						LOG(ERROR) << name << " Can not start unknown monitor with id = " << listener.monitorId;
						result = false;
					}
				}
		} else if (monitorOperation.compare("STOP") == 0) {
				LOG(DEBUG) << name << ": STOP operation found.";

				if (monitor != 0 ) {
					map<Id, ListenerSpec>::iterator monitorIt = monitor->listeners.find(listener.monitorId);
					if(monitorIt != monitor->listeners.end()) {
						monitorIt->second.isStarted = false;
						LOG(INFO) << name << " Stopping  monitor with id = " << listener.monitorId;
					} else {
						LOG(ERROR) << name << " Can not stop unknown monitor with id = " << listener.monitorId;
						result = false;
					}
				}
		} else if (monitorOperation.compare("UNREGISTER") == 0) {
			if (monitor != 0 ) {

				/* If it exists, update spec */
				map<Id, ListenerSpec>::iterator monitorIt = monitor->listeners.find(listener.monitorId);
				if(monitorIt != monitor->listeners.end()) {
					monitor->listeners.erase(listener.monitorId);
					LOG(INFO)  << name << " Removed monitor with id = " << listener.monitorId;
					LOG(DEBUG) << name << " There are now "<< monitor->listeners.size() << " listening monitors registered";
				} else {
					LOG(ERROR)  << name << " Can not remove monitor with id = " << listener.monitorId;
					result = false;
				}

			}
		}



		/* prepare output */



		if(result) {
			outputModelAsJSON.Set("metamodel", libvariant::Variant(outputMetaModelFile));
			brics_3d::rsg::JSONTypecaster::JSONtoString(outputModelAsJSON, outputModel);

			/* validate output */
			if (enableValidation && false) { //TODO update model
				if(!brics_3d::rsg::JSONTypecaster::validateFunctionBlockModel(outputModelAsJSON, outputMetaModelFile, modelsDefaultPath, outputModel)) {
					LOG(ERROR) << "OnCreate: Model validation for output model failed: " << outputModel;
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
	OnCreateObserver* monitor;


	/* Parameters */

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(OnCreate)
FUNCTION_BLOCK_DESTROY
