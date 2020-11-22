/*
PID library 01

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include <avr/io.h>
#include <stdlib.h>
#include <limits.h>

#include "pid.h"

//pid factors
volatile double pid_pidP = 0;
volatile double pid_pidI = 0;
volatile double pid_pidD = 0;

//pid terms
volatile int16_t pid_Perr = 0;
volatile int16_t pid_Ierr = 0;
volatile int16_t pid_Derr = 0;

//pid terms limits
volatile int16_t pid_Perrmin = INT_MIN;
volatile int16_t pid_Perrmax = INT_MAX;
volatile int16_t pid_Ierrmin = INT_MIN;
volatile int16_t pid_Ierrmax = INT_MAX;


/*
 * set pid terms
 */
void pid_setpid(double pidP, double pidI, double pidD) {
	pid_pidP = pidP;
	pid_pidI = pidI;
	pid_pidD = pidD;
}


/*
 * set P term limits
 */
void pid_setlimitsPerr(int16_t Perr_min, int16_t Perr_max) {
	pid_Perrmin = Perr_min;
	pid_Perrmax = Perr_max;
}


/*
 * set I term limits
 */
void pid_setlimitsIerr(int16_t Ierr_min, int16_t Ierr_max) {
	pid_Ierrmin = Ierr_min;
	pid_Ierrmax = Ierr_max;
}


/*
 * reset I term limit
 */
void pid_resetIerr() {
	pid_Ierr = 0;
}


/*
 * pid control algorithm
 */
int16_t pid_update(int16_t setpoint, int16_t input, double dt) {
	//if this function get called always at the same period, dt = 1 can be used
	//otherwise dt should be calculated
	static int16_t inputprev = 0;

	//compute P error
	pid_Perr = setpoint - input;
	if(pid_Perr < pid_Perrmin)
		pid_Perr = pid_Perrmin;
	else if(pid_Perr > pid_Perrmax)
		pid_Perr = pid_Perrmax;

	//compute I error
	pid_Ierr += pid_Perr;
	if(pid_Ierr < pid_Ierrmin)
		pid_Ierr = pid_Ierrmin;
	else if(pid_Ierr > pid_Ierrmax)
		pid_Ierr = pid_Ierrmax;

	//compute D error
	pid_Derr = (inputprev - input);

	//record last value
	inputprev = input;

	//compute output
	int16_t output = (pid_pidP*pid_Perr) + (pid_pidI*pid_Ierr) + (pid_pidD*pid_Derr);
	
	if (output > 255) output = 255;
	else if (output < -255) output = -255;

	return output;
}
