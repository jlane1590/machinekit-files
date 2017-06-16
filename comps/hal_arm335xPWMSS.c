//--------------------------------------------------------------------------
// Description: hal_arm335xPWMSS.h
// HAL module to implement quadrature decoding and pwm generation using the
// ARM335x PWM Subsystem.
//
// Author(s): Josh Lane
// License: GNU GPL Version 2.0 or (at your option) any later version.
//
// Major Changes:
// 06/2017    Josh Lane
//            Initial implementation
//--------------------------------------------------------------------------
// This file is part of LinuxCNC HAL
//
// 
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
// 02110-1301, USA.
//
// THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR
// ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
// TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
// harming persons must have provisions for completely removing power
// from all motors, etc, before persons enter any danger area.  All
// machinery must be designed to comply with local and national safety
// codes, and the authors of this software can not, and do not, take
// any responsibility for such compliance.
//
// This code was written as part of the LinuxCNC project.  For more
// information, go to www.linuxcnc.org.
//-------------------------------------------------------------------------

/* Use config_module.h instead of config.h so we can use RTAPI_INC_LIST_H */
#include "config_module.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "hal.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hal_arm335xPWMSS.h"

/* this probably should be an ARM335x define */
#if !defined(TARGET_PLATFORM_BEAGLEBONE)
#error "This driver is for the beaglebone platform only"
#endif

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

/* Module information */
#define MODNAME "hal_arm335xPWMSS"
MODULE_AUTHOR("Josh Lane");
MODULE_DESCRIPTION("PWMSS EMC HAL driver for ARM335x");
MODULE_LICENSE("GPL");

/* CONSTANTS AND MACROS */
#define MAX_CAP 2
#define MAX_QEP 3
#define MAX_PWM 3
#define MAX_PWMSS 3
//TBCTL Counter Modes
#define TBCTL_CTRMODE_UP        0x0
#define TBCTL_CTRMODE_DOWN      0x1
#define TBCTL_CTRMODE_UPDOWN    0x2
#define TBCTL_CTRMODE_FREEZE    0x3

#define CHAN_A	0
#define CHAN_B	1

/* HAL Module Parameters */
char *ecaps[MAX_CAP] = {0,};
RTAPI_MP_ARRAY_STRING(ecaps, MAX_CAP, "names of eCAP modules");
char *eqeps[MAX_QEP] = {0,};
RTAPI_MP_ARRAY_STRING(eqeps, MAX_QEP, "names of eQEP modules");
char *epwms[MAX_PWM] = {0,};
RTAPI_MP_ARRAY_STRING(epwms, MAX_PWM, "names of ePWM modules");
int frequency = 0;
RTAPI_MP_INT(frequency, "frequency of PWM modules (1Hz - 25kHz)");
int minDC = 0;
RTAPI_MP_INT(minDC, "PWM min allowable duty cycle (0% - 100%)");
int maxDC = 100;
RTAPI_MP_INT(maxDC, "PWM max allowable duty cycle (0% - 100%)");
int type = 0;
RTAPI_MP_INT(type, "output type. 0: unidirectional, 1: bidirectional");

/* Global Variables */
// const devices_t devices[] = {
    // {"PWMSS0", 0x48300000},
    // {"PWMSS1", 0x48302000},
    // {"PWMSS2", 0x48304000},
    // {NULL, -1}
// };

const devices_t available_ecaps[] = {
	{"eCAP0", 0},
	{"eCAP2", 2},
	{NULL, -1}
};

const devices_t available_eqeps[] = {
	{"eQEP0", 0},
	{"eQEP1", 1},
	{"eQEP2", 2},
	{NULL, -1}
};

const devices_t available_epwms[] = {
	{"ePWM0", 0},
	{"ePWM1", 1},
	{"ePWM2", 2},
	{NULL, -1}
};

//PWMSS instantiations on the BBB, index is also the instantiation number
static pwmss_devices_t pwmss_devices[] = {
	{0, 0, 0x48300000, NULL},
	{1, 0, 0x48302000, NULL},
	{2, 0, 0x48304000, NULL}
};

static const char *modname = MODNAME;
static int comp_id;

static eqep_t *eqep_array; /* pointer to array of eqep_t structs in
                                    shmem, 1 per encoder */
static epwm_t *epwm_array; /* pointer to array of epwm_t structs in
									shmem, 1 per pwmgen */
static ecap_t *ecap_array; /* pointer to array of ecap_t structs in
									shmem, 1 per pwmgen */

static int numcap = 0;
static int numqep = 0;
static int numpwm = 0;

static hal_u32_t timebase;

/*---------------------
 Function prototypes
---------------------*/
static void update(void *arg, long period);

static int export_ecap(ecap_t *ecap);
static int setup_ecap(ecap_t *ecap);
static void update_ecap(pwmss_t *pwmss, long period);

static int export_eqep(eqep_t *eqep);
static int setup_eqep(eqep_t *eqep);
static void update_eqep(pwmss_t *pwmss, long period);

static int export_epwm(epwm_t *epwm);
static int setup_epwm(epwm_t *epwm);
static void update_epwm(pwmss_t *pwmss, long period);
void disable_epwm(epwm_t *epwm);
void disable_channel(epwm_t *epwm, int channel);
void enable_channel(epwm_t *epwm, int channel);
void set_channel_dc(epwm_t *epwm, int channel);

/*---------------------
 INIT and EXIT CODE
---------------------*/

int rtapi_app_main(void)
{
    int n, retval, i, j;
    eqep_t *eqep;
	ecap_t *ecap;
	epwm_t *epwm;
	
	//Check for valid module parameters
	int param_error = 0;
	if (frequency < 0 )
	    param_error = 1;
	if (minDC < 0 || minDC > maxDC || maxDC > 100)
		param_error = 2;
	if(type < 0 || type > 1)
		param_error = 3;

    if (param_error) {
		rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: PWM parameters not valid\n",modname);
				hal_exit(comp_id);
		return -1;
	}
	
    /* test for number of channels of each device type */
    for (i=0; i < MAX_CAP && ecaps[i]; i++){
		for(j=0; available_ecaps[j].name; j++) {
            retval = strcmp(ecaps[i], available_ecaps[j].name);
            if (retval == 0 ) {	//input ecap name matches eCAP0 or eCAP2
                pwmss_devices[available_ecaps[j].addr].inUse = 1; //corresponding PWMSS device is in use
				numcap++;
				break;
			}
		}
		if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, ecaps[i]);
            return -1;
        }
	}
	
	for (i=0; i < MAX_QEP && eqeps[i]; i++){
		for(j=0; available_eqeps[j].name; j++) {
            retval = strcmp(eqeps[i], available_eqeps[j].name);
            if (retval == 0 ) {	//input eqep name matches eQEP0, eQEP1, or eQEP2
                pwmss_devices[available_eqeps[j].addr].inUse = 1; //corresponding PWMSS device is in use
				numqep++;
				break;
			}
		}
		if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, eqeps[i]);
            return -1;
        }
	}
	
	for (i=0; i < MAX_PWM && epwms[i]; i++){
		for(j=0; available_epwms[j].name; j++) {
            retval = strcmp(epwms[i], available_epwms[j].name);
            if (retval == 0 ) {	//input epwm name matches ePWM0, ePWM1, or ePWM2
                pwmss_devices[available_epwms[j].addr].inUse = 1; //corresponding PWMSS device is in use
				numpwm++;
				break;
			}
		}
		if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, epwms[i]);
            return -1;
        }
	}
	
	//Check for valid module parameters
    if(numcap < 0 || numcap > MAX_CAP)  {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: invalid number of eCAPs: %d\n", modname, numcap);
        return -1;
    }
	
	if(numqep < 0 || numqep > MAX_QEP)  {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: invalid number of eQEPs: %d\n", modname, numqep);
        return -1;
    }
	
	if(numpwm < 0 || numpwm > MAX_PWM)  {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: invalid number of ePWMs: %d\n", modname, numpwm);
        return -1;
    }
	
	int totalDevices = numcap + numqep + numpwm;
	
	if(totalDevices <= 0){
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: At least one device must be used: %d\n", modname, totalDevices);
        return -1;
    }

    /* have good config info, connect to the HAL */
    comp_id = hal_init(modname);
    if(comp_id < 0 ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n",modname);
        return -1;
    }

    /* allocate shared memory for ecap data */
	if(numcap > 0){
		ecap_array = hal_malloc(numcap * sizeof(ecap_t));
		if (ecap_array ==  0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: hal_malloc() failed\n",modname);
			hal_exit(comp_id);
			return -1;
		}
	}
	
	/* allocate shared memory for eqep data */
	if(numqep > 0){
		eqep_array = hal_malloc(numqep * sizeof(eqep_t));
		if (eqep_array ==  0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: hal_malloc() failed\n",modname);
			hal_exit(comp_id);
			return -1;
		}
	}
	
	/* allocate shared memory for epwm data */
	if(numpwm > 0){
		epwm_array = hal_malloc(numpwm * sizeof(epwm_t));
		if (epwm_array ==  0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: hal_malloc() failed\n",modname);
			hal_exit(comp_id);
			return -1;
		}
	}
	
	/* allocate shared memory for pwmss data */
	pwmss_t *pwmss = hal_malloc(sizeof(pwmss_t));
		if (pwmss ==  0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: hal_malloc() failed\n",modname);
			hal_exit(comp_id);
			return -1;
		}
	
	/* map memory for all PWMSS in use */	
	for(i=0; i<MAX_PWMSS;i++){
		if(pwmss_devices[i].inUse){ //this PWMSS instantiation is in use, so mmap the registers
			int fd = open("/dev/mem", O_RDWR);
			
			pwmss_devices[i].pwmss_reg = mmap(0, IOMEMLEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, pwmss_devices[i].deviceAddr);
			
			close(fd);
			
			if(pwmss_devices[i].pwmss_reg == MAP_FAILED) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "%s: ERROR: PWMSS mmap failed %d\n", modname, i);
                    return -1;
                }
		}
	}
	
    timebase = 0;
	
	/* setup and export all the variables for each eCAP device */
    for (n = 0; n < numcap; n++) {
        ecap = &(ecap_array[n]);
        /* make sure it's a valid device */
        for(i=0; available_ecaps[i].name; i++) {
            retval = strcmp(ecaps[n], available_ecaps[i].name);
            if (retval == 0 ) {	//input ecap name matches eCAP0 or eCAP2
                ecap->name = available_ecaps[i].name;

                ecap->ecap_reg = (void*) pwmss_devices[available_ecaps[i].addr].pwmss_reg + 0x100;

                rtapi_print("memmapped %s to %p\n",ecap->name,ecap->ecap_reg);
                setup_ecap(ecap);
                break;
            }
        }

        if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, ecaps[n]);
            return -1;
        }
    }

    /* setup and export all the variables for each eQEP device */
    for (n = 0; n < numqep; n++) {
        eqep = &(eqep_array[n]);
        /* make sure it's a valid device */
        for(i=0; available_eqeps[i].name; i++) {
            retval = strcmp(eqeps[n], available_eqeps[i].name);
            if (retval == 0 ) {	//input eqep name matches eQEP0, eQEP1, or eQEP2
                eqep->name = available_eqeps[i].name;

                eqep->eqep_reg = (void*) pwmss_devices[available_eqeps[i].addr].pwmss_reg + 0x180;

                rtapi_print("memmapped %s to %p\n",eqep->name,eqep->eqep_reg);
                setup_eqep(eqep);
                break;
            }
        }

        if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, eqeps[n]);
            return -1;
        }
    }
	
	/* setup and export all the variables for each ePWM device */
    for (n = 0; n < numpwm; n++) {
        epwm = &(epwm_array[n]);
        /* make sure it's a valid device */
        for(i=0; available_epwms[i].name; i++) {
            retval = strcmp(epwms[n], available_epwms[i].name);
            if (retval == 0 ) {	//input epwm name matches ePWM0, ePWM1, or ePWM2
                epwm->name = available_epwms[i].name;

                epwm->epwm_reg = (void*) pwmss_devices[available_epwms[i].addr].pwmss_reg + 0x200;

                rtapi_print("memmapped %s to %p\n",epwm->name,epwm->epwm_reg);
                setup_epwm(epwm);
                break;
            }
        }

        if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, epwms[n]);
            return -1;
        }
    }
	
	pwmss->numcap = numcap;
	pwmss->numqep = numqep;
	pwmss->numpwm = numpwm;
	
	pwmss->ecap = ecap_array;
	pwmss->eqep = eqep_array;
	pwmss->epwm = epwm_array;

    /* export functions */
    retval = hal_export_funct("pwmss.update", update,
        pwmss, 0, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: function export failed\n",modname);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: installed %d total devices\n", modname, totalDevices);
    retval = hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
	int n;
    epwm_t *epwm;
	
	for(n = 0; n < numpwm; n++){
		epwm = &(epwm_array[n]);
		disable_epwm(epwm);
	}
	
    hal_exit(comp_id);
}

/*---------------------
 Realtime functions
---------------------*/

static void update(void *arg, long period)
{
	pwmss_t *pwmss = arg;
	
	if(pwmss->numcap > 0)
		update_ecap(pwmss, period);
	if(pwmss->numqep > 0)
		update_eqep(pwmss, period);
	if(pwmss->numpwm > 0)
		update_epwm(pwmss, period);

}

static void update_ecap(pwmss_t *pwmss, long period)
{
	
}

static void update_eqep(pwmss_t *pwmss, long period)
{
    hal_s32_t     i;
    hal_s32_t   delta_counts;
    hal_u32_t   delta_time;
    hal_u32_t   iflg;
    hal_float_t  vel,interp;
    eqep_t *eqep;

    eqep = pwmss->eqep;

    for(i = 0; i < pwmss->numqep; i++){
        /* Read the hardware  */
        eqep->raw_count = eqep->eqep_reg->QPOSCNT;
        iflg = eqep->eqep_reg->QFLG & EQEP_INTERRUPT_MASK;

        /* check if an index event has occured */
        if( *(eqep->index_ena) && (iflg & IEL)) {
            eqep->index_count = eqep->eqep_reg->QPOSILAT;
            *(eqep->index_ena) = 0;
        }

        /* check for phase errors */
        if( iflg & PHE ) {
            *(eqep->phase_error_count)++;
        }
        /* clear interrupt flags */
        eqep->eqep_reg->QCLR = iflg;

        /* check for changes in scale */
        if ( *(eqep->pos_scale) != eqep->old_scale ){
            eqep->old_scale = *(eqep->pos_scale);
            /* sanity check new value */
            if ((*(eqep->pos_scale) < 1e-20) && (*(eqep->pos_scale) > -1e-20)) {
                /* value is too small */
                *(eqep->pos_scale) = 1.0;
            }
            /* we want the reciprocal */
            eqep->scale = 1.0 / *(eqep->pos_scale);
        }

        /* has counter_mode been changed? */
        if ( *(eqep->counter_mode) != eqep->old_counter_mode ) {
            eqep->eqep_reg->QDECCTL ^= QSRC0;
            eqep->old_counter_mode = *(eqep->counter_mode);
        }

        /* has x2_mode been changed? */
        if ( *(eqep->x2_mode) != eqep->old_x2_mode ) {
            eqep->eqep_reg->QDECCTL ^= XCR;
            eqep->old_x2_mode = *(eqep->x2_mode);
        }

        /* has invert_A been changed? */
        if ( *(eqep->invertA) != eqep->old_invertA ) {
            eqep->eqep_reg->QDECCTL ^= QAP;
            eqep->old_invertA = *(eqep->invertA);
        }

        /* has invert_B been changed? */
        if ( *(eqep->invertB) != eqep->old_invertB ) {
            eqep->eqep_reg->QDECCTL ^= QBP;
            eqep->old_invertB = *(eqep->invertB);
        }

        /* has invert_Z been changed? */
        if ( *(eqep->invertZ) != eqep->old_invertZ ) {
            eqep->eqep_reg->QDECCTL ^= QIP;
            eqep->old_invertZ = *(eqep->invertZ);
        }

        /* has the capture prescaler been changed? */
        if ( *(eqep->capture_pre) != eqep->old_capture_pre ) {
            hal_u32_t active_pre;

            eqep->eqep_reg->QCAPCTL &= ~(CEN); // disable to prevent prescaler problems
            eqep->eqep_reg->QCAPCTL &= ~(CCPS0 | CCPS1 | CCPS2); // clear prescaler
            if (*(eqep->capture_pre) < 8u) {
                active_pre = *(eqep->capture_pre);
            }
            else {
                active_pre = 7u;  // clamp prescaler
            }
            eqep->eqep_reg->QCAPCTL |= active_pre << CCPS;
            eqep->eqep_reg->QCAPCTL |= CEN; // enable eQEP capture timer

            eqep->old_capture_pre = *(eqep->capture_pre);
            // prescale the capture tick, bit shift = division with base 2
            eqep->capture_freq = SYSCLKOUT / (hal_float_t)(1 << active_pre);
        }

        /* check for valid min_speed */
        if ( *(eqep->min_speed) <= 0.0 ) {
            *(eqep->min_speed) = 1.0;
        }

        /* check reset input */
        if (*(eqep->reset)) {
            /* reset is active, reset the counter */
            /* note: we NEVER reset raw_counts, that is always a
            running count of the edges seen since startup. The
            public "count" is the difference between raw_count
            and index_count, so it will become zero. */
            eqep->index_count = eqep->raw_count;
        }

        /* check for movement */
        if ( eqep->raw_count != eqep->old_raw_count ) {
            *(eqep->raw_counts) = eqep->raw_count;

            delta_counts = eqep->raw_count - eqep->old_raw_count;
            if (delta_counts < 0) {
                eqep->reverse = true;
                delta_counts = -delta_counts;
            }
            else {
                eqep->reverse = false;
            }
            delta_time = timebase - eqep->timestamp;
            eqep->old_raw_count = eqep->raw_count;
            eqep->timestamp = timebase;

        } else { /* no counts */
            delta_counts = 0;
        }

        /* decide which velocity estimation method to use */
        if (delta_counts >= *(eqep->capture_threshold)) {
            vel = ((hal_float_t)delta_counts * eqep->scale) / ((hal_float_t)delta_time * 1e-9);
        }
        else { /* use capture timer for velocity */
            if (eqep->eqep_reg->QEPSTS & UPEVNT) { /* we had an up event */
                hal_s32_t period_count;
                eqep->eqep_reg->QEPSTS = UPEVNT;  /* clear UPEVNT */
                period_count = eqep->eqep_reg->QCPRDLAT;
                *(eqep->capture_period) = period_count;
                vel = eqep->capture_freq / ((hal_float_t)period_count * *(eqep->pos_scale));
            }
            else { /* guess the velocity */
                hal_s32_t timer_count;
                timer_count = eqep->eqep_reg->QCTMRLAT;
                vel =  eqep->capture_freq / ((hal_float_t)timer_count * *(eqep->pos_scale));
                if (vel < *(eqep->min_speed)) {  /* no guessing beyond min speed */
                    vel = 0.0;
                    *(eqep->capture_period) = 0.0;
                }
                else if (vel > eqep->old_vel) {  /* use only if slower */
                    vel = eqep->old_vel;
                }
                else {
                    *(eqep->capture_period) = timer_count;
                }
            }
            /* apply direction to velocity */
            if (eqep->eqep_reg->QEPSTS & COEF) { /* overflow event */
                eqep->eqep_reg->QEPSTS = COEF;
                *(eqep->capture_overflow_count) += 1;
                *(eqep->capture_period) = 0;
                vel = 0.0;
            }
            if (eqep->eqep_reg->QEPSTS & CDEF) {  /* dir change event */
                eqep->eqep_reg->QEPSTS = CDEF;
                *(eqep->capture_dir_change_count) += 1;
                *(eqep->capture_period) = 0;
                vel = 0.0;
            }
        }

        /* store and apply velocity */
        eqep->old_vel = vel;
        if (eqep->reverse) {
            vel = -vel;
        }
        *(eqep->vel) = vel;

        /* compute net counts */
        *(eqep->count) = eqep->raw_count - eqep->index_count;

        *(eqep->pos) = *(eqep->count) * eqep->scale;

        /* add interpolation value */
        delta_time = timebase - eqep->timestamp;
        interp = *(eqep->vel) * (delta_time * 1e-9);
        *(eqep->pos_interp) = *(eqep->pos) + interp;


        /* move on to the next channel */
        eqep++;
    }
    timebase += (hal_u32_t)period;
}

static void update_epwm(pwmss_t *pwmss, long period)
{
	hal_s32_t     i;
	epwm_t *epwm;
	
    epwm = pwmss->epwm;
	
	for(i = 0; i < pwmss->numpwm; i++)
	{
		epwm->scale = (*(epwm->scale_in));
		epwm->enA = (*(epwm->enA_in));
		epwm->enB = (*(epwm->enB_in));
		
		//check if scale parameter has changed
		if(epwm->scale != epwm->oldScale) //scale parameter has changed, validate new scale value
		{
			
			if((epwm->scale < 1e-20) && (epwm->scale > -1e-20)) // value too small, divide by zero is a bad thing
			{
				epwm->scale = 1.0;
			}
		}
		
		//calculate scaled duty cycle values
		epwm->scaled_dcA = (*(epwm->dcA)) / epwm->scale; //scale dcA
		epwm->scaled_dcB = (*(epwm->dcB)) / epwm->scale; //scale dcB
		
		//clamp dc values if output is unidirectional
		if(epwm->outputType == 0)
		{
			if(epwm->scaled_dcA < 0.0)
				epwm->scaled_dcA = 0.0;
				
			if(epwm->scaled_dcB < 0.0)
				epwm->scaled_dcB = 0.0;
		}
		
		//check if either duty cycle is below the resolution of the pwm
		if(epwm->scaled_dcA < epwm->resolution && epwm->scaled_dcA > -epwm->resolution) //dcA is smaller than the resolution of the pwm counters and will be treated as 0
		{
			epwm->scaled_dcA = 0.0;
			epwm->enA = 0;
		}
		
		if(epwm->scaled_dcB < epwm->resolution && epwm->scaled_dcB > -epwm->resolution) //dcB is smaller than the resolution of the pwm counters and will be treated as 0
		{
			epwm->scaled_dcB = 0.0;
			epwm->enB = 0;
		}
		
		//channel A output is to be disabled
		if(!epwm->enA)
		{
			epwm->scaled_dcA = 0.0;
			if(epwm->oldEnA) //channel A output is currently enabled; else the channel is already disabled so do nothing
				disable_channel(epwm, CHAN_A);
		}
		else //channel A output is to be enabled
		{
			//set pwm direction
			if(epwm->scaled_dcA < 0.0)
			{
				epwm->dirA = 1;
				epwm->scaled_dcA = -epwm->scaled_dcA;
			}
			else
			{
				epwm->dirA = 0;
			}
			
			/* limit the duty cycle */
			if(epwm->scaled_dcA > epwm->max_dc) 
			{
				epwm->scaled_dcA = epwm->max_dc;
			}
			else if(epwm->scaled_dcA < epwm->min_dc)
			{
				epwm->scaled_dcA = epwm->min_dc;
			}
			//check if scaled_dcA has changed and update registers accordingly
			if(epwm->scaled_dcA != epwm->old_scaled_dcA)
				set_channel_dc(epwm, CHAN_A);
			//check if the A direction output has changed and update accordingly
			if(epwm->outputType == 1)
				if(epwm->dirA != epwm->oldDirA)
					(*(epwm->dirA_out)) = epwm->dirA;
			//check if channel A output is currently disabled and update registers accordingly
			if(!epwm->oldEnA && epwm->enA)
				enable_channel(epwm, CHAN_A);
		}
		
		//channel B output is to be disabled
		if(!epwm->enB)
		{
			epwm->scaled_dcB = 0.0;
			if(epwm->oldEnB) //channel A output is currently enabled; else the channel is already disabled so do nothing
				disable_channel(epwm, CHAN_B);
		}
		else //channel B output is to be enabled
		{
			//set pwm direction
			if(epwm->scaled_dcB < 0.0)
			{
				epwm->dirB = 1;
				epwm->scaled_dcB = -epwm->scaled_dcB;
			}
			else
			{
				epwm->dirB = 0;
			}
			
			/* limit the duty cycle */
			if(epwm->scaled_dcB > epwm->max_dc) 
			{
				epwm->scaled_dcB = epwm->max_dc;
			}
			else if(epwm->scaled_dcB < epwm->min_dc)
			{
				epwm->scaled_dcB = epwm->min_dc;
			}
			//check if scaled_dcB has changed and update registers accordingly
			if(epwm->scaled_dcB != epwm->old_scaled_dcB)
				set_channel_dc(epwm, CHAN_B);
			//check if the B direction output has changed and update accordingly
			if(epwm->dirB != epwm->oldDirB)
				(*(epwm->dirB_out)) = epwm->dirB;
			//check if channel B output is currently disabled and update registers accordingly
			if(!epwm->oldEnB && epwm->enB)
				enable_channel(epwm, CHAN_B);
		}
		
		epwm->oldEnA = epwm->enA;
		epwm->old_scaled_dcA = epwm->scaled_dcA;
		epwm->oldDirA = epwm->dirA;
		epwm->oldEnB = epwm->enB;
		epwm->old_scaled_dcB = epwm->scaled_dcB;
		epwm->oldDirB = epwm->dirB;
		epwm->oldScale = epwm->scale;
		
		epwm++;
	}
}

/*---------------------
 Local functions
---------------------*/
static int setup_ecap(ecap_t *ecap)
{
	/* export pins to hal */
	export_ecap(ecap);
	
	/* initialize members of ecap struct */
	ecap->scale = 100.0f;
	ecap->scaled_dc = 0.0f;
	ecap->old_scaled_dc = 0.0f;
	ecap->en = false;
	ecap->oldEn = false;
	ecap->min_dc = (float)minDC / 100.0f;
	ecap->max_dc = (float)maxDC / 100.0f;
	ecap->dir = 0;
	ecap->oldDir = 0;
	ecap->period = 4000; //equates to 25kHz
	ecap->resolution = 0.00025; //equates to 25kHz
	ecap->outputType = type;

	return 0;
}

static int setup_eqep(eqep_t *eqep)
{
    export_eqep(eqep);
	
    *(eqep->raw_counts) = 0;
    *(eqep->count) = 0;
    *(eqep->min_speed) = 1.0;
    *(eqep->pos) = 0.0;
    *(eqep->pos_scale) = 1.0;
    *(eqep->vel) = 0.0;
    *(eqep->phase_error_count) = 0;
    *(eqep->counter_mode) = false;
    *(eqep->x2_mode) = false;
    *(eqep->invertA) = false;
    *(eqep->invertB) = false;
    *(eqep->invertZ) = false;
    *(eqep->capture_period) = 0;
    *(eqep->capture_threshold) = 100;
    *(eqep->capture_overflow_count) = 0;
    *(eqep->capture_dir_change_count) = 0;
    eqep->old_raw_count = 0;
    eqep->old_scale = 1.0;
    eqep->raw_count = 0;
    eqep->timestamp = 0;
    eqep->index_count = 0;
    eqep->counts_since_timeout = 0;
    eqep->scale = 1.0 / *(eqep->pos_scale);
    eqep->reverse = false;
    eqep->old_vel = 0.0;
    eqep->old_counter_mode = false;
    eqep->old_x2_mode = false;
    eqep->old_invertA = false;
    eqep->old_invertB = false;
    eqep->old_invertZ = false;
    eqep->old_capture_pre = 255u;
    eqep->capture_freq = 0.0;

    eqep->eqep_reg->QDECCTL = XCR; /* start in x1 resolution */
    eqep->eqep_reg->QPOSINIT = 0;
    eqep->eqep_reg->QPOSMAX = -1;
    eqep->eqep_reg->QEINT |= (IEL | PHE);
    eqep->eqep_reg->QEPCTL = PHEN | IEL0 | IEL1 | SWI |PCRM0;

    eqep->eqep_reg->QCAPCTL = 0u; // reset to prevent prescaler problems
    eqep->eqep_reg->QCAPCTL |= CEN; // enable eQEP capture

    rtapi_print("%s: REVID = %#x\n",modname, eqep->eqep_reg->QREVID);
    return 0;
}

static int setup_epwm(epwm_t *epwm)
{
	/* export pins to hal */
	export_epwm(epwm);
	
	/* initialize members of ePWM struct */
	epwm->scale = 100.0f;
	epwm->scaled_dcA = 0.0f;
	epwm->scaled_dcB = 0.0f;
	epwm->old_scaled_dcA = 0.0f;
	epwm->old_scaled_dcB = 0.0f;
	epwm->enA = false;
	epwm->enB = false;
	epwm->oldEnA = false;
	epwm->oldEnB = false;
	epwm->min_dc = (float)minDC / 100.0f;
	epwm->max_dc = (float)maxDC / 100.0f;
	epwm->dirA = 0;
	epwm->dirB = 0;
	epwm->oldDirA = 0;
	epwm->oldDirB = 0;
	epwm->period = 4000; //equates to 25kHz
	epwm->resolution = 0.00025; //equates to 25kHz
	epwm->outputType = type;
	

	/* compute TBPRD and Clock dividers for desired PWM frequency */
	float Cyclens =0.0f ;
	float Divisor =0.0f;
	int i , j ;
	const float CLKDIV_div[] = {1.0 ,2.0 ,4.0 ,8.0 ,16.0 ,32.0 , 64.0 , 128.0};
	const float HSPCLKDIV_div[] ={1.0 ,2.0 ,4.0 ,6.0 ,8.0 ,10.0 , 12.0 , 14.0};
	int NearCLKDIV =7;
	int NearHSPCLKDIV =7;
	//int NearTBPRD =0;

	Cyclens = 1000000000.0f / frequency ; /* 10^9 / HZ , comput time per cycle (ns) */

	Divisor =  (Cyclens / 655350.0f) ;	/* am335x provide (128*14) divider , and per TBPRD means 10 ns when divider /1 ,
						 * and max TBPRD is 65535 , so , the max cycle is 128*14* 65535 *10ns */

	if(Divisor > (128 * 14)) {
		rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: PWM frequency out of bounds\n",modname);
				hal_exit(comp_id);
		return -1;
	}
	else {
		/* using Exhaustive Attack method */
		for(i = 0 ; i < 8 ; i ++) {
			for(j = 0 ; j < 8 ; j ++) {
				if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]) &&
				  ((CLKDIV_div[i] * HSPCLKDIV_div[j]) > Divisor)) {
					NearCLKDIV = i ;
					NearHSPCLKDIV = j ;
				}
			}
		}

		//NearTBPRD = (Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;
		epwm->period = (hal_u32_t)(Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;
		epwm->resolution = 1.0f/(float)epwm->period;
	}

	/* setting clock driver and freeze time base */
	epwm->epwm_reg->TBCTL = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);
	
	epwm->epwm_reg->TBPRD = (unsigned short)epwm->period;
	
	/* reset time base counter */
	epwm->epwm_reg->TBCNT = 0;

	/*  setting duty A and duty B, PWM channel outputs initially disabled */
	epwm->epwm_reg->CMPB = (unsigned short)((float)epwm->period * epwm->scaled_dcB);

	epwm->epwm_reg->CMPA = (unsigned short)((float)epwm->period * epwm->scaled_dcA);
	
	/* make sure PWM A and B outputs are disabled */
	epwm->epwm_reg->AQCTLA = 0x1 | ( 0x0 << 4);
    		
    epwm->epwm_reg->AQCTLB = 0x1 | ( 0x0 << 8);

	/* start the period counter even though the outputs are disabled */
    epwm->epwm_reg->TBCTL &= ~0x3;
	
    return 0;
}

static int export_ecap(ecap_t *ecap)
{
    if (hal_pin_bit_newf(HAL_IN, &(ecap->en_in), comp_id, "%s.en", ecap->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting out-enable\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(ecap->dc), comp_id, "%s.dc", ecap->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting duty-cycle\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(ecap->scale_in), comp_id, "%s.dc_scale", ecap->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Duty-cycle-scale\n");
        return -1;
    }
	if (hal_pin_bit_newf(HAL_OUT, &(ecap->dir_out), comp_id, "%s.dir", ecap->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting dir-out\n");
        return -1;
    }

    return 0;
}

static int export_eqep(eqep_t *eqep)
{
    if (hal_pin_bit_newf(HAL_IO, &(eqep->index_ena), comp_id, "%s.index-enable", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting index-enable\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->reset), comp_id, "%s.reset", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting reset\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_IO, &(eqep->count), comp_id, "%s.counts", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting counts\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IO, &(eqep->pos_scale), comp_id, "%s.position-scale", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting position-scale\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(eqep->min_speed), comp_id, "%s.min-speed-estimate", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting min-speed-estimate\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_OUT, &(eqep->pos_interp), comp_id, "%s.position-interpolated", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting position-interpolated\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_OUT, &(eqep->vel), comp_id, "%s.velocity", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting velocity\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_OUT, &(eqep->phase_error_count), comp_id, "%s.phase-errors", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting phase_errors\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_OUT, &(eqep->pos), comp_id, "%s.position", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting position\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_OUT, &(eqep->raw_counts), comp_id, "%s.rawcounts", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting rawcounts\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->counter_mode), comp_id, "%s.counter-mode", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting counter mode\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->x2_mode), comp_id, "%s.x2-mode", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting x2 mode\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->invertA), comp_id, "%s.invert-A", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting invert_A mode\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->invertB), comp_id, "%s.invert-B", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting invert_B mode\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(eqep->invertZ), comp_id, "%s.invert-Z", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting invert_Z mode\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_OUT, &(eqep->capture_period), comp_id, "%s.capture-period", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting capture-period\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_OUT, &(eqep->capture_overflow_count), comp_id, "%s.capture-overflows", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting capture-overflows\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_OUT, &(eqep->capture_dir_change_count), comp_id, "%s.capture-dir-changes", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting capture-dir-changes\n");
        return -1;
    }
    if (hal_pin_s32_newf(HAL_IN, &(eqep->capture_threshold), comp_id, "%s.capture-threshold", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting capture-threshold\n");
        return -1;
    }
    if (hal_pin_u32_newf(HAL_IN, &(eqep->capture_pre), comp_id, "%s.capture-prescaler", eqep->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting capture-prescaler\n");
        return -1;
    }

    return 0;
}

static int export_epwm(epwm_t *epwm)
{
    if (hal_pin_bit_newf(HAL_IN, &(epwm->enA_in), comp_id, "%s.en_A", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting A-out-enable\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IN, &(epwm->enB_in), comp_id, "%s.en_B", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting B-out-enable\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(epwm->dcA), comp_id, "%s.dc_A", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-A-duty-cycle\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(epwm->dcB), comp_id, "%s.dc_B", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-B-duty-cycle\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(epwm->scale_in), comp_id, "%s.dc_scale", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Duty-cycle-scale\n");
        return -1;
    }
	if (hal_pin_bit_newf(HAL_OUT, &(epwm->dirA_out), comp_id, "%s.dir_A", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting A-dir-out\n");
        return -1;
    }
	if (hal_pin_bit_newf(HAL_OUT, &(epwm->dirB_out), comp_id, "%s.dir_B", epwm->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting B-dir-out\n");
        return -1;
    }

    return 0;
}

void disable_epwm(epwm_t *epwm)
{
    epwm->epwm_reg->TBCTL |= 0x3;
    
    epwm->epwm_reg->AQCTLA = 0x1 | ( 0x0 << 4);
    		
    epwm->epwm_reg->AQCTLB = 0x1 | ( 0x0 << 8);

    epwm->epwm_reg->TBCNT = 0;
}

void disable_channel(epwm_t *epwm, int channel)
{
    //ePWM->ePWM_reg->TBCTL |= 0x3;
    if(channel == CHAN_A)
	{
        epwm->epwm_reg->AQCTLA = 0x1 | ( 0x0 << 4);
		//*(ePWM->dutyA) = *(ePWM->minDC);
	}
    else if(channel == CHAN_B)	
	{
        epwm->epwm_reg->AQCTLB = 0x1 | ( 0x0 << 8);
		//*(ePWM->dutyB) = *(ePWM->minDC);
	}

    //ePWM->ePWM_reg->TBCNT = 0;
}

void enable_channel(epwm_t *epwm, int channel)
{
    //ePWM->ePWM_reg->TBCTL |= 0x3;
	//set_channel_dc(ePWM, channel);
	
    if(channel == CHAN_A){
        epwm->epwm_reg->AQCTLA = 0x2 | ( 0x3 << 4);
    }else if(channel == CHAN_B){
        epwm->epwm_reg->AQCTLB = 0x2 | ( 0x3 << 8);
    }
    //ePWM->ePWM_reg->TBCNT = 0;
}

void set_channel_dc(epwm_t *epwm, int channel)
{
	//int NearTBPRD = 4000;

    if(channel == CHAN_A){
		//ePWM->dcA_scaled = *(ePWM->dutyA) / (100.0f * *(ePWM->scale));
		unsigned short CMPA_val = (unsigned short)((float)epwm->period * epwm->scaled_dcA);
		if(CMPA_val < 1)
		{
			disable_channel(epwm, channel);
			epwm->enA = 0;
		}
        epwm->epwm_reg->CMPA = CMPA_val;
    }
	else if(channel == CHAN_B){
		//ePWM->dcB_scaled = *(ePWM->dutyB) / (100.0f * *(ePWM->scale));
		unsigned short CMPB_val = (unsigned short)((float)epwm->period * epwm->scaled_dcB);
		if(CMPB_val == 0)
		{
			disable_channel(epwm, channel);
			epwm->enA = 0;
		}
        epwm->epwm_reg->CMPB = CMPB_val;
    }
}












