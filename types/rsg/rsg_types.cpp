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

#include <stdint.h>

#include "ubx.h"

//#define MAX_NUMBER_OF_IDS 64 //Well this is really an arbitrary choice...

/* BRICS_3D includes */
//#include <brics_3d/worldModel/WorldModel.h>

/* include types */
#include "types/rsg_uuid.h"
#include "types/rsg_uuid.h.hexarr"

#include "types/rsg_ids.h"
#include "types/rsg_ids.h.hexarr"

#include "types/rsg_wm_handle.h"
#include "types/rsg_wm_handle.h.hexarr"

/* declare types */
ubx_type_t rsg_types[] = {

	/* declare rsg types */
	def_struct_type(struct rsg_uuid, &rsg_uuid_h),
	def_struct_type(struct rsg_ids, &rsg_ids_h),

	def_struct_type(struct rsg_wm_handle, &rsg_wm_handle_h),

	{ NULL },
};



static int rsgtypes_init(ubx_node_info_t* ni)
{
	DBG(" ");
	ubx_type_t *tptr;
	for(tptr=rsg_types; tptr->name!=NULL; tptr++) {
		/* TODO check for errors */
		ubx_type_register(ni, tptr);
	}

	return 0;
}

static void rsgtypes_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	const ubx_type_t *tptr;

	for(tptr=rsg_types; tptr->name!=NULL; tptr++)
		ubx_type_unregister(ni, tptr->name);
}

UBX_MODULE_INIT(rsgtypes_init)
UBX_MODULE_CLEANUP(rsgtypes_cleanup)
