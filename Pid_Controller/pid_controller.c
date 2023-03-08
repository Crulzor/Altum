#include "pid_controller.h"

void PIDControllerInit(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDControllerUpdate(PIDController *pid) {
	//Accuracy of the position. Devides the 4096 int from adc to smaller value to have less noise
	pid->setPoint = truncf(pid->setPoint / pid->accuracy);
	pid->measurement = truncf(pid->measurement / pid->accuracy);

	/*
	 * Error signal
	 */
	float error = pid->setPoint - pid->measurement;

	/*
	 * Proportional
	 */
	float proportional = pid->Kp * error;

	/*
	 * Integral
	 */
	pid->integrator = pid->integrator
			+ 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid->integrator > pid->limMaxInt) {

		pid->integrator = pid->limMaxInt;

	} else if (pid->integrator < pid->limMinInt) {

		pid->integrator = pid->limMinInt;
	}

	/*
	 * Derivative (band-limited differentiator)
	 */

	pid->differentiator = -(2.0f * pid->Kd
			* (pid->measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
	+ (2.0f * pid->tau - pid->T) * pid->differentiator)
			/ (2.0f * pid->tau + pid->T);

	/*
	 * Compute output and apply limits
	 */
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax) {

		pid->out = pid->limMax;

	} else if (pid->out < pid->limMin) {

		pid->out = pid->limMin;

	}

	/* Store error and measurement for later use */
	pid->prevError = error;
	pid->prevMeasurement = pid->measurement;

	/* Return controller output */
	return pid->out;

}
