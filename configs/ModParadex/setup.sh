#!/bin/bash
# Copyright 2013
# Charles Steinkuehler <charles@steinkuehler.net>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

dtbo_err () {
	echo "Error loading device tree overlay file: $DTBO" >&2
	exit 1
}

pin_err () {
	echo "Error exporting pin:$PIN" >&2
	exit 1
}

dir_err () {
	echo "Error setting direction:$DIR on pin:$PIN" >&2
	exit 1
}

SLOTS=/sys/devices/bone_capemgr.*/slots

# Make sure required device tree overlay(s) are loaded
for DTBO in cape-universal cape-bone-iio ; do

	if grep -q $DTBO $SLOTS ; then
		echo $DTBO overlay found
	else
		echo Loading $DTBO overlay
		sudo -A su -c "echo $DTBO > $SLOTS" || dtbo_err
		sleep 1
	fi
done;

if [ ! -r /sys/devices/ocp.*/helper.*/AIN0 ] ; then
	echo Analog input files not found in /sys/devices/ocp.*/helper.* >&2
	exit 1;
fi

if [ ! -r /sys/class/uio/uio0 ] ; then
	echo PRU control files not found in /sys/class/uio/uio0 >&2
	exit 1;
fi

# Export GPIO pins:
# One pin needs to be exported to enable the low-level clocks for the GPIO
# modules (there is probably a better way to do this)
# 
# Any GPIO pins driven by the PRU need to have their direction set properly
# here.  The PRU does not do any setup of the GPIO, it just yanks on the
# pins and assumes you have the output enables configured already
# 
# Direct PRU inputs and outputs do not need to be configured here, the pin
# mux setup (which is handled by the device tree overlay) should be all
# the setup needed.
# 
# Any GPIO pins driven by the hal_bb_gpio driver do not need to be
# configured here.  The hal_bb_gpio module handles setting the output
# enable bits properly.  These pins _can_ however be set here without
# causing problems.  You may wish to do this for documentation or to make
# sure the pin starts with a known value as soon as possible.

sudo $(which config-pin) -f - <<- EOF

	# eQEP encoder channel input pins
	P8.11	qep		#eQEP2B_in
	P8.12	qep		#eQEP2A_in
	P8.33	qep		#eQEP1B_in
	P8.35	qep		#eQEP1A_in
	P9.27	qep		#eQEP0B_in
	P9.42	qep		#eQEP0A_in
	
	# ePWM/eCAP pwm output pins
	P8.13	pwm		#ePWM2B
	P8.19	pwm		#ePWM2A
	P9.14	pwm		#ePWM1A
	P9.16	pwm		#ePWM1B
	P9.21	pwm		#ePWM0B
	P9.22	pwm		#ePWM0A
#	P9.28	pwm2	#eCAP2
	
	# hpg encoder pru direct input pins
	P8.15	pruin		#hpg0A_in pru0_15
	P8.16	pruin		#hpg0B_in pru0_14
	P9.24	pruin		#hpg1A_in pru0_16
	P9.25	pruin		#hpg1B_in pru0_7
	P9.29	pruin		#hpg2A_in pru0_1
	P9.30	pruin		#hpg2B_in pru0_2
	P9.31	pruin		#hpg3A_in pru0_0
	P9.91	pruin		#hpg3B_in pru0_6, connected to P9.41
	
	# pwm direction output pins
	P8.14	gpio		#ePWM2B_dir
	P8.17	gpio		#ePWM2A_dir
	P9.13	gpio		#ePWM1A_dir
	P9.15	gpio		#ePWM1B_dir
	P9.19	gpio		#ePWM0B_dir
	P9.23	gpio		#ePWM0A_dir
#	P9.26	gpio		#eCAP2_dir

	# limit switch input pins
	P8.34	gpio		#limit-x
	P8.36	gpio		#limit-y
	P8.38	gpio		#limit-z
	P8.40	gpio		#limit-a
	P8.42	gpio		#limit-b
	P8.44	gpio		#limit-c
#	P8.46	gpio		#limit-u
EOF

