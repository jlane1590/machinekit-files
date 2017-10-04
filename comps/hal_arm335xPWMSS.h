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

#ifndef _hal_arm335xPWMSS_H_
#define _hal_arm335xPWMSS_H_

#define IOMEMLEN (8*1024)


typedef struct {
    char *name;
    int addr;
} devices_t;

typedef struct {
    __u32   IDVER;
    __u32   SYSCONFIG;
    __u32   CLKCONFIG;
    __u32   CLKSTATUS;
	
} pwmss_reg_t;

typedef struct {
	__u32	TSCTR;				//0x0		Time-stamp counter register
	__u32	CTRPHS;				//0x4		Counter phase offset value register
	__u32	CAP1;				//0x8		Capture 1 register
	__u32	CAP2;				//0xC		Capture 2 register
	__u32	CAP3;				//0x10		Capture 3 register
	__u32	CAP4;				//0x14		Capture 4 register
	__u32	RESERVED[4];			//0x18		Reserved memory space
	__u16	ECCTL1;			//0x28		Capture control register 1
	__u16	ECCTL2;				//0x2A		Capture control register 2
	__u16	ECEINT;				//0x2C		Capture interrupt enable register
	__u16	ECFLG;				//0x2E		Capture interrupt flag register
	__u16	ECCLR;				//0x30		Capture interrupt clear register
	__u16	ECFRC;				//0x32		Capture interrupt force register
	__u32	RESERVED_2[10];		//0x34		Reserved memory space
	__u32	REVID;				//0x5C		Revision ID register
	
} ecap_reg_t;

typedef struct {
    __s32   QPOSCNT;
    __s32   QPOSINIT;
    __s32   QPOSMAX;
    __u32   QPOSCMP;
    __u32   QPOSILAT;
    __u32   QPOSSLA;
    __u32   QPOSLAT;
    __u32   QUTM;
    __u32   QUPRD;
    __u16   QWDTMR;
    __u16   QWDPRD;
    __u16   QDECCTL;
    __u16   QEPCTL;
    __u16   QCAPCTL;
    __u16   QPOSCTL;
    __u16   QEINT;
    __u16   QFLG;
    __u16   QCLR;
    __u16   QFRC;
    __u16   QEPSTS;
    __u16   QCTMR;
    __u16   QCPRD;
    __u16   QCTMRLAT;
    __u16   QCPRDLAT; // 40
    __u16   FILLER[13]; // 42,44,46,48,4a,4c,4e,50,52,54,56,58,5a
    __u32   QREVID; // 5c
	
} eqep_reg_t;

typedef struct {
	__u16	TBCTL;		//0x0		Time base control register
	__u16	TBSTS;		//0x2		TIme base status register
	__u16	TBPHSHR;	//0x4		HRPWM phase register
	__u16	TBPHS;		//0x6		Time base phase register
	__u16	TBCNT;		//0x8		Time base counter register
	__u16	TBPRD;		//0xA		Time base period register
	__u16	RESERVED;	//0xC		Reserved memory space, DO NOT MODIFY
	__u16	CMPCTL;		//0xE		Counter compare control register
	__u16	CMPAHR;		//0x10		HRPWM counter compare A register
	__u16	CMPA;		//0x12		Counter compare A register
	__u16	CMPB;		//0x14		Counter compare B register
	__u16	AQCTLA;		//0x16		Action qualifier control register for output A
	__u16	AQCTLB;		//0x18		Action qualifier control register for output B
	__u16	AQSFRC;		//0x1A		Action qualifier software force register
	__u16	AQCSFRC;	//0x1C		Action qualifier continuous software force register
	__u16	DBCTL;		//0x1E		Dead band generator control register
	__u16	DBRED;		//0x20		Dead band generator rising edge delay count register
	__u16	DBFED;		//0x22		Dead band generator falling edge delay count register
	
} epwm_reg_t;

typedef struct {
    char 					*name;
    //pwmss_reg_t volatile	*pwmss_reg;   /* Pointer to PWMSS hardware registers */
    ecap_reg_t volatile		*ecap_reg;  	/* Pointer to ePWM hardware registers */

	hal_float_t				*dc;
	hal_bit_t				*en_in;
	hal_float_t				*scale_in;
	hal_bit_t				*dir_out;
	hal_bit_t				*dir_out_inv;
	
	hal_float_t				scale;
	hal_float_t				oldScale;
	hal_float_t				scaled_dc;
	hal_float_t				old_scaled_dc;
	hal_bit_t				en;
	hal_bit_t				oldEn;
	hal_float_t				min_dc;
	hal_float_t				max_dc;
	hal_bit_t				dir;
	hal_bit_t				oldDir;
	hal_u32_t				period;
	hal_float_t				resolution;
	hal_bit_t				outputType;
	
} ecap_t;

typedef struct {
    char            *name;
    //pwm_reg_t volatile  *pwmss_reg;   /* Pointer to hardware PWMSS hardware registers */
    eqep_reg_t volatile *eqep_reg;  /* Pointer to hardware eQEP hardware registers */
    hal_s32_t           old_raw_count;    /* used by update to detect movement */
    hal_s32_t       raw_count;
    hal_u32_t       timestamp;
    hal_s32_t       index_count;
    hal_s32_t       counts_since_timeout;

    hal_bit_t       *index_ena;
    hal_bit_t       *reset;         /* counter reset input */
    hal_bit_t       *counter_mode;
    hal_bit_t       old_counter_mode; /* so we can tell if counter_mode has
                                       changed without bothering the hardware */
    hal_bit_t       *x2_mode;
    hal_bit_t       old_x2_mode;      /* so we can tell if x2_mode has
                                       changed without bothering the hardware */
    hal_bit_t       *invertA;
    hal_bit_t       old_invertA;      /* so we can tell if invertA has
                                       changed without bothering the hardware */
    hal_bit_t       *invertB;
    hal_bit_t       old_invertB;      /* so we can tell if invertB has
                                       changed without bothering the hardware */
    hal_bit_t       *invertZ;
    hal_bit_t       old_invertZ;      /* so we can tell if invertZ has
                                       changed without bothering the hardware */
    hal_s32_t       *raw_counts;
    hal_s32_t       *count;         /* = *(raw_counts) - index_counts */
    hal_float_t     *pos_scale;     /* scaling factor for pos */
    hal_float_t     *pos;
    hal_float_t     *pos_interp;
    hal_float_t     *min_speed;
    hal_s32_t       *phase_error_count;
    hal_float_t     old_scale;
    hal_float_t     scale;          /* reciprocal value used for scaling */
    hal_bit_t       reverse;        /* if the velocity should be inverted */
    hal_float_t     *vel;
    hal_float_t     old_vel;
    hal_float_t     *delta_counts;

    hal_s32_t       *capture_period;
    hal_s32_t       *capture_overflow_count;
    hal_s32_t       *capture_dir_change_count;
    hal_s32_t       *capture_threshold; /* ticks per period until we use tick difference for velocity calculation*/
    hal_u32_t       *capture_pre;   /* capture timer prescaler */
    hal_u32_t       old_capture_pre;
    hal_float_t     capture_freq;

} eqep_t;

typedef struct {
    char 					*name;
    //pwmss_reg_t volatile	*pwmss_reg;   /* Pointer to PWMSS hardware registers */
    epwm_reg_t volatile		*epwm_reg;  	/* Pointer to ePWM hardware registers */

	hal_float_t				*dcA;
	hal_float_t				*dcB;
	hal_bit_t				*enA_in;
	hal_bit_t				*enB_in;
	hal_float_t				*scale_in;
	hal_bit_t				*dirA_out;
	hal_bit_t				*dirA_out_inv;
	hal_bit_t				*dirB_out;
	hal_bit_t				*dirB_out_inv;
	
	hal_float_t				scale;
	hal_float_t				oldScale;
	hal_float_t				scaled_dcA;
	hal_float_t				scaled_dcB;
	hal_float_t				old_scaled_dcA;
	hal_float_t				old_scaled_dcB;
	hal_bit_t				enA;
	hal_bit_t				enB;
	hal_bit_t				oldEnA;
	hal_bit_t				oldEnB;
	hal_float_t				min_dc;
	hal_float_t				max_dc;
	hal_bit_t				dirA;
	hal_bit_t				dirB;
	hal_bit_t				oldDirA;
	hal_bit_t				oldDirB;
	hal_u32_t				period;
	hal_float_t				resolution;
	hal_bit_t				outputType;
	
} epwm_t;

typedef struct {
	int						numcap;
	int						numqep;
	int						numpwm;
	
	ecap_t					*ecap;
	eqep_t					*eqep;
	epwm_t					*epwm;
	
} pwmss_t;

typedef struct {
	hal_u8_t				deviceNum;
	hal_u8_t				inUse;
	int						deviceAddr;
	pwmss_reg_t volatile	*pwmss_reg;   /* Pointer to hardware PWMSS hardware registers */
	
} pwmss_devices_t;

// system tick 100MHz
#define SYSCLKOUT  1e8
#define SYSCLKOUT_INV 1e-8

// Bits for the QDECTL register
#define QSRC1      (0x0001 << 15)
#define QSRC0      (0x0001 << 14)
#define SOEN       (0x0001 << 13)
#define SPSEL      (0x0001 << 12)
#define XCR        (0x0001 << 11)
#define SWAP       (0x0001 << 10)
#define IGATE      (0x0001 << 9)
#define QAP        (0x0001 << 8)
#define QBP        (0x0001 << 7)
#define QIP        (0x0001 << 6)
#define QSP        (0x0001 << 5)

// Bits for the QEPCTL register
#define FREESOFT1  (0x0001 << 15)
#define FREESOFT0  (0x0001 << 14)
#define PCRM1      (0x0001 << 13)
#define PCRM0      (0x0001 << 12)
#define SEI1       (0x0001 << 11)
#define SEI0       (0x0001 << 10)
#define IEI1       (0x0001 << 9)
#define IEI0       (0x0001 << 8)
#define SWI        (0x0001 << 7)
#define SEL        (0x0001 << 6)
#define IEL1       (0x0001 << 5)
#define IEL0       (0x0001 << 4)
#define PHEN       (0x0001 << 3)
#define QCLM       (0x0001 << 2)
#define UTE        (0x0001 << 1)
#define WDE        (0x0001 << 0)

// Bits for the QCAPCTL register
#define CEN        (0x0001 << 15)
#define CCPS       4 // position of CCPS bits
#define CCPS2      (0x0001 << 6)
#define CCPS1      (0x0001 << 5)
#define CCPS0      (0x0001 << 4)
#define UPPS3      (0x0001 << 3)
#define UPPS2      (0x0001 << 2)
#define UPPS1      (0x0001 << 1)
#define UPPS0      (0x0001 << 0)

// Bits for the QPOSCTL register
#define PCSHDW     (0x0001 << 15)
#define PCLOAD     (0x0001 << 14)
#define PCPOL      (0x0001 << 13)
#define PCE        (0x0001 << 12)
#define PCSPW11    (0x0001 << 11)
#define PCSPW10    (0x0001 << 10)
#define PCSPW9    (0x0001 << 9)
#define PCSPW8    (0x0001 << 8)
#define PCSPW7    (0x0001 << 7)
#define PCSPW6    (0x0001 << 6)
#define PCSPW5    (0x0001 << 5)
#define PCSPW4    (0x0001 << 4)
#define PCSPW3    (0x0001 << 3)
#define PCSPW2    (0x0001 << 2)
#define PCSPW1    (0x0001 << 1)
#define PCSPW0    (0x0001 << 0)


// Bits for the QEPSTS register
#define UPEVNT  (0x0001 << 7)
#define FDF     (0x0001 << 6)
#define QDF     (0x0001 << 5)
#define QDLF    (0x0001 << 4)
#define COEF    (0x0001 << 3)
#define CDEF    (0x0001 << 2)
#define FIMF    (0x0001 << 1)
#define PCEF    (0x0001 << 0)

// Bits for the interrupt registers
#define EQEP_INTERRUPT_MASK (0x0FFF)
#define UTOF                (0x0001 << 11)
#define IEL                 (0x0001 << 10)
#define PHE                 (0x0001 << 2)

// Bits for PWM CLKCONFIG
#define EQEPCLK_EN          (0x0001 << 4)

#endif

