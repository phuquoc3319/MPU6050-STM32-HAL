#include "KALMAN.h"

void kalman_init(kalman *k)
{
	k->Q_angle = 0.001;
	k->Q_bias = 0.003;
	k->Q_measure = 0.03;
	
	k->bias =0;
	k->p[0][0] = 0;
	k->p[0][1] = 0;
	k->p[1][0] = 0;
	k->p[1][1] = 0;
}

float getAngle(float newAngle,float newRate,float dt,kalman *k)
{
	k->rate = newRate - k->bias;
	k->Angle += dt* k->rate;
	
	k->p[0][0] += dt * (dt*k->p[1][1] - k->p[0][1] - k->p[1][0] + k->Q_angle);
	k->p[0][1] -= dt * k->p[1][1];
	k->p[1][0] -= dt * k->p[1][1];
	k->p[1][1] += k->Q_bias * dt;
	
	k->s = k->p[0][0] + k->Q_measure;
	
	k->k[0] = k->p[0][0]/k->s;
	k->k[1] = k->p[1][0]/k->s;
	
	k->y = newAngle - k->Angle;
	k->Angle += k->k[0] * k->y;
	k->bias += k->k[1] * k->y;
	
	k->p[0][0] -= k->k[0] * k->p[0][0];
	k->p[0][1] -= k->k[0] * k->p[0][1];
	k->p[1][0] -= k->k[1] * k->p[0][0];
	k->p[1][1] -= k->k[1] * k->p[0][1];
	
	return (k->Angle);
}

