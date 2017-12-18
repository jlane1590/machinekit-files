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
//#include "modparakins.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

#define VTVERSION VTKINEMATICS_VERSION1

#define radians(x) (x * PM_PI) / 180.0

/* define the geometry of the RSP */

/* radius of base joints; 0 index for even joints, 1 index for odd joints */
static const double baseRadius[2] = {
					1.767,
					2.369
					};

/* radius of platform joints; 0 index for even joints, 1 index for odd joints */
static const double platformRadius[2] = {
					1.651,
					1.651
					};


/* angle from x axis of base joints */
static const double baseAngles[6] = {
					132.99,
					213.07,
					252.99,
					333.07,
					12.99,
					93.07
					};

/* angle from x axis of platform joints */
static const double platformAngles[6] = {
					165.86,
					194.14,
					285.86,
					314.14,
					45.86,
					74.14
					};

/* angle from x axis of servo horn planes */
static const double beta[6] = {
					180.0,
					0.0,
					300.0,
					120.0,
					60.0,
					240.0
					};

static const double initialHeight =		4.845;
static const double hornLength =		1.25;
static const double legLength =			5.0;

/* position of base strut ends in base (world) coordinate system */
static PmCartesian b[6] = {
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0}
					};

static PmCartesian p[6] = {
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0},
					{0, 0, 0}
					};

const char * kinematicsGetName(void)
{
  return "rsp";
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

	PmCartesian q, l;
	PmRpy rpy;
	int i;
	double lMag;
	
	//convert pose angles from degrees to radians;
	rpy.r = radians(pos->a);	//roll phi
	rpy.p = radians(pos->b);	//pitch theta
	rpy.y = radians(pos->c);	//yaw

	for(i = 0; i < 6; i++)
	{
		//calculate q vector, platform joints wrt base coord system
		q.x = p[i].x*rtapi_cos(rpy.p) + p[i].y*rtapi_sin(rpy.p)*rtapi_sin(rpy.r) + pos->tran.x;
		q.y = p[i].y*rtapi_cos(rpy.r) + pos->tran.y;
		q.z = -p[i].x*rtapi_sin(rpy.p) + p[i].y*rtapi_cos(rpy.p)*rtapi_sin(rpy.r) + pos->tran.z + initialHeight;

		//calculate virtual leg length
		pmCartCartSub(&q, &b[i], &l);
		pmCartMagSq(&l, &lMag);

		double L = lMag - (legLength*legLength) + (hornLength*hornLength);
		double M = 2*hornLength*(q.z - b[i].z);
		double N = 2*hornLength*(rtapi_cos(radians(beta[i]))*(q.x-b[i].x) + rtapi_sin(radians(beta[i]))*(q.y-b[i].y));

		joints[i] = rtapi_asin(L/rtapi_sqrt(M*M+N*N)) - rtapi_atan2(N,M);
			
	}
	
	//write pos->c (yaw) to joint6
	//joints[6] = rpy.y;
	
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
    .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};

static int comp_id, vtable_id;
static const char *name = "rspkins";

int rtapi_app_main(void)
{
	int i;

    /* calculate base and platform joint positions */
    for(i = 0; i < 6; i++) {
        b[i].x = baseRadius[i%2]*rtapi_cos(radians(baseAngles[i]));
        b[i].y = baseRadius[i%2]*rtapi_sin(radians(baseAngles[i]));
        b[i].z = 0;

        p[i].x = platformRadius[i%2]*rtapi_cos(radians(platformAngles[i]));
        p[i].y = platformRadius[i%2]*rtapi_sin(radians(platformAngles[i]));
        p[i].z = 0;
    }

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
