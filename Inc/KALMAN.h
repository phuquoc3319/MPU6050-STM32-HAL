#ifndef __KALMAN_H
#define __KALMAN_H


typedef struct
{
	float Q_angle;
	float Q_bias;
	float Q_measure;
	float Angle;
	float bias;
	float rate;
	
	float p[2][2];
	float k[2];
	float y;
	float s;
}kalman;


float getAngle(float newAngle,float newRate,float dt,kalman *k);


#endif


