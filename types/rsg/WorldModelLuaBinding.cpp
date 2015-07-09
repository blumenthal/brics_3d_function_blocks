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

#include "WorldModelLuaBinding.h"
#include <brics_3d/worldModel/sceneGraph/UuidGenerator.h>
#include <assert.h>

extern "C" {

/* Wrapped constructor */
WorldModel* WorldModel_WorldModel() {

	/* Configure the logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	return new WorldModel();
}

WorldModel* WorldModel_WorldModelWithId(const char* idAsString) {

	/* Configure the logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	std::string id(idAsString);
	LOG(DEBUG) << "[Id] = " << id;
	brics_3d::rsg::Id rootId;
	if(rootId.fromString(id)) {
		LOG(INFO) << "Preconfigured rootId is: " << rootId;
		return new WorldModel(new brics_3d::rsg::UuidGenerator(rootId));
	} else {
		LOG(ERROR) << "Failed to parse root id. Given: " << id << " Falling back to default Id.";
	}


	return new WorldModel();
}

/* Wrapped destuctor */
void WorldModel__gc(WorldModel *this_) {
	delete this_;
}

/* Helper function to get a handle of to be used for the configure functions of
 * a micro block
 */
rsg_wm_handle* WorldModel_getHandle(WorldModel *this_) {
	rsg_wm_handle* tmpHandle = new rsg_wm_handle; //scope?
	tmpHandle->wm = reinterpret_cast<void*>(this_);
	return tmpHandle;
}

/**
 * @brief Get the local rootId of the WorldModel
 * @param this_ Pointer to insatnce of WorldModel class.
 * @return A c string with the root Id. Nil in case of in error.
 */
const char* WorldModel_getRootId(WorldModel *this_) {
	assert(this_ != 0);
	std::string rootIdAsString;

	brics_3d::rsg::Id rootId = this_->getRootNodeId();
	rootIdAsString = rootId.toString();
	return rootIdAsString.c_str();
}

//bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
//bool WorldModel_setNodeAttributes(WorldModel *this_, Id id, vector<Attribute> newAttributes);
bool WorldModel_addNodeAttribute(WorldModel *this_, const char* idAsString, const char* key, const char* value) {
	assert(this_ != 0);
	assert(idAsString != 0);
	assert(key != 0);
	assert(value != 0);

	std::string tmpId(idAsString);
	std::string tmpKey(key);
	std::string tmpValue(value);

	brics_3d::rsg::Id id;
	if(!id.fromString(tmpId)) { return false; }

	/* Get potentially existing attributes */
	std::vector<brics_3d::rsg::Attribute> attributes;
	if(!this_->scene.getNodeAttributes(id, attributes)) { return false; }

	/* Append new attribute */
	attributes.push_back(brics_3d::rsg::Attribute(tmpKey, tmpValue));

	/* Update node */
	if(!this_->scene.setNodeAttributes(id, attributes)) { return false; }

	return true;
}

}

/* EOF */
