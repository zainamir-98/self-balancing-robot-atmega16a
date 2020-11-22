/*
PID library 01

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#ifndef PID_H_
#define PID_H_


//functions
extern void pid_setpid(double pidP, double pidI, double pidD) ;
extern void pid_setlimitsPerr(int16_t Perr_min, int16_t Perr_max);
extern void pid_setlimitsIerr(int16_t Ierr_min, int16_t Ierr_max);
extern void pid_resetIerr();
extern int16_t pid_update(int16_t setpoint, int16_t input, double dt);

#endif
