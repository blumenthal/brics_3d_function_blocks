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
#include <stdio.h>

extern "C" {
WorldModel* WorldModel_WorldModel() {
	printf("WorldModel_WorldModel\n");

	/* Configure logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	return new WorldModel();
}

void WorldModel__gc(WorldModel *this_) {
	delete this_;
}

rsg_wm_handle WorldModel_getHandle(WorldModel *this_) {
	rsg_wm_handle tmpHandle; //scope?
	tmpHandle.wm = reinterpret_cast<void*>(this_);
	return tmpHandle;
}

}

/* EOF */
