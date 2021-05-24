/*****************************************************************************/
/* File: formation_follower.c 
/*****************************************************************************/

#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS           8

/* Formation flocking parameters */
#define D                    0.1      // Distance between robots
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};	
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag receiver;                 // Handle for the receiver node for range and bearing information


int robot_id;                       // Unique robot ID

float goal_range   = 0.0;
float goal_bearing = 0.0;

float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;

static void reset(void) {
	wb_robot_init();

	receiver    = wb_robot_get_device("receiver");

	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	/*Webots 2018b*/

	int i;

	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);      // the device name is specified in the world file
		s[2]++;                         // increases the device number
	}
	char* robot_name; robot_name=(char*) wb_robot_get_name(); 

	sscanf(robot_name,"epuck%d",&robot_id);    // read robot id from the robot's name
	printf("Reset: robot %d\n",robot_id);
}

void update_leader_measurement(float new_leader_range, float new_leader_bearing, float new_leader_orientation) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
	leader_orientation = new_leader_orientation;
}

void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr) {
	// Define constants
	float Ku = 2.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	float theta = leader_orientation;
	x += goal_range * cosf(- M_PI + goal_bearing + theta);
	y += goal_range * sinf(- M_PI + goal_bearing + theta);
	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotational control
	float w = Kw * range * sinf(bearing) + Kb * leader_orientation;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)

	// Convert to wheel speeds!
	*msl = (int)((u - AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}

int main(){
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int i,initialized;

	reset();                          // Initialization 
	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64); 

	//read the initial packets
	initialized = 0;
	while(!initialized){
		/* Wait until supervisor sent range and bearing information */
		while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(64); // Executing the simulation for 64ms
		}  
		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
			if((int)rbbuffer[0]==robot_id){
				initialized = 1;
				rbbuffer = (float*) wb_receiver_get_data(receiver);
				goal_range = sqrt(rbbuffer[1]*rbbuffer[1] + rbbuffer[2]*rbbuffer[2]);
				goal_bearing = -atan2(rbbuffer[1],rbbuffer[2]);
				printf("Goal of robot %d: range = %.2f, bearing = %.2f\n", robot_id, goal_range, goal_bearing);
				leader_range = goal_range;
				leader_bearing = goal_bearing;
				leader_orientation = rbbuffer[2];
				printf("  Initial relative position: range = %.2f, bearing = %.2f, heading = %.2f\n", leader_range, leader_bearing, leader_orientation);
			}
			wb_receiver_next_packet(receiver);
		}
	}
	msl=0; msr=0;  

	for(;;){
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds

		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
			if((int)rbbuffer[0]==robot_id){
				new_leader_range = sqrt(rbbuffer[1]*rbbuffer[1] + rbbuffer[2]*rbbuffer[2]);
				new_leader_bearing = -atan2(rbbuffer[1],rbbuffer[2]);
				new_leader_orientation = rbbuffer[3];
				update_leader_measurement(new_leader_range, new_leader_bearing, new_leader_orientation);
				//printf("%d: Leader: r:%f gr:%f, b:%f gb:%f, o:%f\n", new_leader_range, goal_range, new_leader_bearing, goal_bearing, new_leader_orientation);
			}
			wb_receiver_next_packet(receiver);
		}


		compute_wheel_speeds(0, 0, &msl, &msr);

		msl += bmsl;
		msr += bmsr;

		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		wb_robot_step(64);               // Executing the simulation for 64ms
	}
}  
  
