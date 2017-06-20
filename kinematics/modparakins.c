/********************************************************************
* Description: trivkins.c
*   Trivial kinematics for 3 axis Cartesian machine
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include "kinematics.h"		/* these decls */
#include "rtapi_math.h"
#include "posemath.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

#define VTVERSION VTKINEMATICS_VERSION1

/* define position of base strut ends in base (world) coordinate system */
static PmCartesian b[6] = {
				{BASE_0_X, BASE_0_Y, BASE_0_Z},
				{BASE_1_X, BASE_1_Y, BASE_1_Z},
				{BASE_2_X, BASE_2_Y, BASE_2_Z},
				{BASE_3_X, BASE_3_Y, BASE_3_Z},
				{BASE_4_X, BASE_4_Y, BASE_4_Z},
				{BASE_5_X, BASE_5_Y, BASE_5_Z}
				};

static PmCartesian p[6] = {
				{PLATFORM_0_X, PLATFORM_0_Y, PLATFORM_0_Z},
				{PLATFORM_1_X, PLATFORM_1_Y, PLATFORM_1_Z},
				{PLATFORM_2_X, PLATFORM_2_Y, PLATFORM_2_Z},
				{PLATFORM_3_X, PLATFORM_3_Y, PLATFORM_3_Z},
				{PLATFORM_4_X, PLATFORM_4_Y, PLATFORM_4_Z},
				{PLATFORM_5_X, PLATFORM_5_Y, PLATFORM_5_Z}
				};

const char * kinematicsGetName(void)
{
  return "modpara";
}

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{

    return 0;
}

//This assumes that the yaw motion is decoupled from the modified paradex hexapod
//and is controlled by a separate joint
int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{

	PmCartesian A;
	PmRpy rpy;
	
	//convert pose angles from degrees to radians;
	rpy.r = pos->a * PM_PI / 180.0;	//roll phi
	rpy.p = pos->b * PM_PI / 180.0;	//pitch theta
	rpy.y = pos->c * PM_PI / 180.0;	//yaw
	
	//calculate active rod length for each joint
	for(int i = 0; i < 6; i++)
	{
		A.x = p[i]x*rtapi_cos(rpy.p) + p[i].y*rtapi_sin(rpy.p)*rtapi_sin(rpy.r) + pos->x - b[i].x;
		A.y = p[i].y*rtapi_cos(rpy.r) + pos->y - b[i].y;
		A.z = rtapi_sqrt(Amag*Amag - A.x*A.x - A.y*A.y);
		
		if(rtapi_isnan(A.z))
			return -1;
		
		joints[i] = pos->z - p[i].x*rtapi_sin(rpy.p) + p[i].y*rtapi_cos(rpy.p)*rtapi_sin(rpy.r) - A.z;
	}
	
	//write pos->c (yaw) to joint6
	
    return 0;
}

/*
  kinematicsHome() is implemented by taking the desired world coordinates,
  which are passed as an arg, and running the inverse kinematics to get
  the resulting joints. The flags are set to zero.
*/
int kinematicsHome(EmcPose * world,
                   double * joint,
                   KINEMATICS_FORWARD_FLAGS * fflags,
                   KINEMATICS_INVERSE_FLAGS * iflags)
{
  *fflags = 0;
  *iflags = 0;

  return kinematicsInverse(world, joint, iflags, fflags);
}

KINEMATICS_TYPE kinematicsType(void)
{
    return KINEMATICS_INVERSE_ONLY;
}

MODULE_LICENSE("GPL");

static vtkins_t vtk = {
    .kinematicsForward = kinematicsForward,
    .kinematicsInverse  = kinematicsInverse,
    // .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};

static int comp_id, vtable_id;
static const char *name = "modparakins";

int rtapi_app_main(void)
{
    comp_id = hal_init(name);
    if(comp_id > 0) {
	vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);

	if (vtable_id < 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
			    name, name, VTVERSION, &vtk, vtable_id );
	    return -ENOENT;
	}
	hal_ready(comp_id);
	return 0;
    }
    return comp_id;
}

void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}
