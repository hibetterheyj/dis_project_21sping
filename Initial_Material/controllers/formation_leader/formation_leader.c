/*****************************************************************************/
/* File: formation_leader.c 
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>


#define FORWARD_SPEED 400
#define TURN_RATE 40
#define NB_SENSOR 8
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};
WbDeviceTag ds[NB_SENSOR];
WbDeviceTag emitter;                  // Handle for the emitter node

void reset(void) {
	wb_robot_init();

	emitter = wb_robot_get_device("emitter");

	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	/*Webots 2018b*/  

	char text[4] = "ps0";
	int i;
	for (i=0;i<NB_SENSOR;i++) {
		ds[i] = wb_robot_get_device(text); /* distance sensors */
		text[2]++;
	}
}

void braiten(short* left,short* right) {
	unsigned short ds_value[NB_SENSOR];
	int i;

	ds_value[0] = wb_distance_sensor_get_value(ds[0]);
	ds_value[1] = wb_distance_sensor_get_value(ds[1]);
	ds_value[2] = wb_distance_sensor_get_value(ds[2]);
	ds_value[3] = wb_distance_sensor_get_value(ds[3]);
	ds_value[4] = wb_distance_sensor_get_value(ds[4]);
	ds_value[5] = wb_distance_sensor_get_value(ds[5]);
	ds_value[6] = wb_distance_sensor_get_value(ds[6]);
	ds_value[7] = wb_distance_sensor_get_value(ds[7]);

	*left = 0;
	*right = 0;
	for (i=0;i<NB_SENSOR;i++) {
		*right += ds_value[i] * Interconn[i];
		*left += ds_value[i] * Interconn[i+NB_SENSOR];
	}
	*left /= 400;
	*right /= 400;
}

void run(short left_forward, short right_forward) {
	int left_encoder,right_encoder;
	short left_speed=0,right_speed=0;
	/*Webots 2018b*/
	float left_speed_w, right_speed_w;
	/*Webots 2018b*/
	int i;

	for (i=0;i<50;i++) {
		braiten(&left_speed,&right_speed);
		left_speed += left_forward;
		right_speed += right_forward;

		left_encoder = wb_differential_wheels_get_left_encoder();
		right_encoder = wb_differential_wheels_get_right_encoder();
		if (left_encoder>9000) wb_differential_wheels_set_encoders(0,right_encoder);
		if (right_encoder>1000) wb_differential_wheels_set_encoders(left_encoder,0);

		/*Webots 2018b*/
		/* Set the motor speeds */
		left_speed_w = left_speed*MAX_SPEED_WEB/1000;
		right_speed_w = right_speed*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, left_speed_w);
		wb_motor_set_velocity(right_motor, right_speed_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
		wb_robot_step(64); /* run one step */
	}
}

int main() {
	int rnd;
	int i;

	reset();

	srand(time(NULL));


	for(i=0;i<NB_SENSOR;i++) {
		wb_distance_sensor_enable(ds[i],64);
	}
	wb_differential_wheels_enable_encoders(64);
	for(;;) { /* The robot never dies! */

	rnd = rand()/(RAND_MAX/4);
	if (rnd > 2) {
	  run(FORWARD_SPEED+TURN_RATE,FORWARD_SPEED-TURN_RATE);
	} else if (rnd < 1) {
	  run(FORWARD_SPEED-TURN_RATE,FORWARD_SPEED+TURN_RATE);
	} else {
	  run(FORWARD_SPEED,FORWARD_SPEED);
	}

	}
	return 0;
}
