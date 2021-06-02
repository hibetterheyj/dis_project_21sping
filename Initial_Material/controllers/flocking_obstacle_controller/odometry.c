#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <gsl/gsl_sf_bessel.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     			// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    			// Print odometry values computed with accelerometer
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc, _speed_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */

void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3])
{

  //printf("acc mean %g \n",acc_mean[1]);
  //printf("acc %g \n",acc[1]);
  if(!isnan(acc[1]) && !isnan(acc[2]) && !isnan(acc[3]))
  {
    double acc_wx = ( acc[1] - acc_mean[1]);

    double a = _odo_pose_enc.heading;

    _odo_speed_acc.x += acc_wx*_T;

    _odo_pose_acc.x += _odo_speed_acc.x*_T*cos(a);
    _odo_pose_acc.y += _odo_speed_acc.x*_T*sin(a);
    _odo_pose_acc.heading= a;
  
    //printf("ODO with acceleration : %g %g %g\n", _odo_pose_acc.x , _odo_pose_acc.y , _odo_pose_enc.heading);
    /*_odo_speed_acc.x += acc_wx*_T*cos(a);
    _odo_speed_acc.y += acc_wx*_T*sin(a);

    _odo_pose_acc.x += _odo_speed_acc.x*_T;
    _odo_pose_acc.y += _odo_speed_acc.y*_T;*/

    memcpy(odo, &_odo_pose_acc, sizeof(pose_t));

    if(VERBOSE_ODO_ACC)
    {
      printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
    }
  }
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo, pose_t* enc_speed, double Aleft_enc, double Aright_enc)
{
  if(!isnan(Aleft_enc) && !isnan(Aright_enc))
  {
    // Rad to meter
    Aleft_enc  *= WHEEL_RADIUS;

    Aright_enc *= WHEEL_RADIUS;
	
    //printf("aleft enc %g \n",Aleft_enc);
    //printf("aright enc %g \n",Aright_enc);

    // Compute forward speed and angular speed
    double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );

    double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );

	// Apply rotation (Body to World)

    double a = _odo_pose_enc.heading;

    double speed_wx = speed * cos(a);

    _speed_enc.x = speed_wx;

    double speed_wy = speed * sin(a);

    _speed_enc.y = speed_wy;

    memcpy(enc_speed, &_speed_enc, sizeof(pose_t));

    // Integration : Euler method
    _odo_pose_enc.x += speed_wx * _T;

    _odo_pose_enc.y += speed_wy * _T;

    _odo_pose_enc.heading += omega * _T;

    memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

    if(VERBOSE_ODO_ENC)
    {
      printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
    }
  }
}


/*
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step,pose_t* initpos)
{

 	memcpy(&_odo_pose_acc, initpos , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memcpy(&_odo_pose_enc, initpos , sizeof(pose_t));

	memset(&_speed_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
	
	//printf("x : %f, y : %f, h : %f",_odo_pose_acc.x,_odo_pose_acc.y,_odo_pose_acc.heading);
}