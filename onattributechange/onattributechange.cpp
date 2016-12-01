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
 * @file onattributechange
 *
 * A query function block to aggregate multiple pose queries into a single one, w.r.t to a single common reference and time stamp.
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

struct ListenerSpec {
	Id nodeId;
	Id monitorId;
	std::string attributeKey;
	std::string lastAttributeValue;
};

/*
 * Custom observer
 */
class OnAttributeChangeObserver: public ISceneGraphUpdateObserver {

public:
	OnAttributeChangeObserver() {
		port = 0;
	}

	virtual ~OnAttributeChangeObserver() {}

	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		return true;
	}

	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		return true;
	}

	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forceId = false) {
		return true;
	}

	bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forceId = false) {
		return true;
	}

	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forceId = false) {
		return true;
	}

    bool addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
		return true;
    }

    bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false) {
    	return true;
    }

	bool setNodeAttributes(Id id, vector<Attribute> newAttributes, TimeStamp timeStamp = TimeStamp(0)) {

		/* Check for each registered listener */
		for(std::map<Id, ListenerSpec>::iterator monitorsIt = listeners.begin(); monitorsIt != listeners.end(); ++monitorsIt) {
			listener = monitorsIt->second;

			if (id == listener.nodeId) {
				vector<std::string> resultValues;
				getValuesFromAttributeList(newAttributes, listener.attributeKey,resultValues);

				/* For each found value check if it has changed */
				for(std::vector<string>::iterator it = resultValues.begin(); it != resultValues.end() ;++it) {
					if(it->compare(listener.lastAttributeValue) != 0) { // It had changed
						LOG(INFO) << "OnAttributeChangeObserver: Monitor with Id = " << listener.monitorId << " detected an attribute of node " << listener.nodeId << " has changed from " << listener.lastAttributeValue << " to " << *it;

						/* TODO send an event to the monitor port */
						if(port) {

							libvariant::Variant result;
							result.Set("@worldmodeltype", libvariant::Variant("RSGMonitor"));
							brics_3d::rsg::JSONTypecaster::addIdToJSON(listener.monitorId, result, "monitorId");
							result.Set("attributeKey", libvariant::Variant(listener.attributeKey));
							result.Set("oldValue", libvariant::Variant(listener.lastAttributeValue));
							result.Set("newValue", libvariant::Variant(*it));
							string message = libvariant::Serialize(result, libvariant::SERIALIZE_JSON);
							LOG(DEBUG) << "OnAttributeChangeObserver: Sending message: " << message;
							int transferredBytes = 0;
							port->write(message.c_str(), message.size(), transferredBytes);

						}  else {
							LOG(WARNING) << "OnAttributeChangeObserver: no monitir port set.";
						}

						monitorsIt->second.lastAttributeValue = *it;
					}
				}
			}
		}

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

	ListenerSpec listener;

	/// Look up table for all registered listeners. Identified via monitorId
    std::map<Id, ListenerSpec> listeners;

    IOutputPort* port;
};

/***********FBX***************/


class OnAttributeChange: public brics_3d::rsg::IFunctionBlock {
public:

	OnAttributeChange(brics_3d::WorldModel* wmHandle) : brics_3d::rsg::IFunctionBlock(wmHandle) {
		name = "onattributechange";
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

	~OnAttributeChange(){
		LOG(INFO) << name << ": Stopping block " << name;

		if(monitor) {
			if(wm->scene.detachUpdateObserver(monitor)) {
				delete monitor;
				monitor = 0;
			} else {
				LOG(ERROR) << name << ": Can not detach OnAttributeChangeObserver.";
			}
		}
	};

	bool configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";

		return true;
	}

	bool execute(std::vector<brics_3d::rsg::Id>& inputDataIds, std::vector<brics_3d::rsg::Id>& outputDataIds) {
		LOG(WARNING) << name << " Cannot execute Id based i/o as this is a monitor	 function block";
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
				LOG(ERROR) << "OnAttributeChange: Model validation for input model failed: " << outputModel;
				return false;
			}
		}

		/* get data */
		ListenerSpec listener;
		listener.monitorId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "monitorId");
		string monitorOperation = inputModelAsJSON.Get("monitorOperation").AsString();

		listener.nodeId = brics_3d::rsg::JSONTypecaster::getIdFromJSON(inputModelAsJSON, "id");
		listener.attributeKey =  inputModelAsJSON.Get("attributeKey").AsString();


		/* output data */
		bool result = true;

		/* execute query
		 *
		 */
		if (monitorOperation.compare("REGISTER") == 0) {
			LOG(DEBUG) << name << ": REGISTER operation found.";

			if (monitor == 0) { // only attach on first creation
				monitor = new OnAttributeChangeObserver();
				monitor->port = wm->scene.getMonitorPort();
				wm->scene.attachUpdateObserver(monitor);
			}

			if (monitor != 0 ) {

				/* Get current attribute as initial value */
				//TODO

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
					LOG(ERROR) << "OnAttributeChange: Model validation for output model failed: " << outputModel;
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
	OnAttributeChangeObserver* monitor;


	/* Parameters */

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(OnAttributeChange)
FUNCTION_BLOCK_DESTROY
