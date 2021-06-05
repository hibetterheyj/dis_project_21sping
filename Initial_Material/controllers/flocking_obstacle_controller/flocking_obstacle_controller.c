#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

/* GSL library required for localization */
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

/* custom library */
#include "../../../localization_library/odometry.h"
#include "../../../localization_library/kalman.h"
/****************************************************************************************************/


/********** Fixed parameters **********/
#define NB_SENSORS      8				// Number of distance sensors
#define MIN_SENS		350     		// Minimum sensibility value
#define MAX_SENS		4096    		// Maximum sensibility value
#define MAX_SPEED		800     		// Maximum speed
#define MAX_SPEED_WEB  	6.28    		// Maximum speed webots
#define FLOCK_SIZE	  	5	  			// Size of flock
#define TIME_STEP	  	64	  			// [ms] Length of time step

#define AXLE_LENGTH 		0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628		// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205		// Wheel radius (meters)
#define DELTA_T			0.064			// Timestep (seconds)

#define TIME_INIT_ACC 3         		// Time in second
#define ABS(x) ((x>=0)?(x):-(x))

float init_x[FLOCK_SIZE] = {0.0, 0.1, -0.1, 0.213892, -0.2}; 		// Initial global position aligned w/ robot coordinate system
float init_z[FLOCK_SIZE] = {-2.9, -2.9, -2.9, -2.9, -2.9};

// Positioning mode
#define POS_ENC_KF		0
#define POS_ACC_KF		1
#define POS_DUMMY_ODO	2
#define POS_GT			3
#define POSITIONING_MODE POS_ENC_KF

/********** Fixed parameters *********/


/********** Tunable parameters **********/
#define VERBOSE false				// Print diagnosis information
#define VERBOSE_GPS false        	// Print GPS values
#define VERBOSE_ACC false        	// Print accelerometer values
#define VERBOSE_ACC_MEAN false   	// Print accelerometer mean values
#define VERBOSE_POSE false       	// Print pose values
#define VERBOSE_ENC false      		// Print encoder values

#define RULE1_THRESHOLD     0.262817		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        0.160796		// Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.086089  		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        0.000928	   	// Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        0.002298   		// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEAKEN_THRESHOLD     1.0   	// Min. distance w/o migration weakening penalty
#define MIGRATION_FREEZE_THRESHOLD     0.2   	// Slow down the robot if it's within this distance to migration destination
#define MIGRATION_WEIGHT			0.019997   	// Weight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 						// Tells the robots if they should just go forward or move towards a specific migratory direction

// Please note that X & Z axes are aligned with the robots' longitudinal and lateral motion directions at initial position respectively.
#define MIGRATORY_DEST_X  0.0				// X-coordinate of migration destination
#define MIGRATORY_DEST_Z  6.0				// Z-coordinate of migration destination 
/********** Tunable parameters **********/


/********** Global varialbes **********/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node
// int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance
float e_puck_matrix[16] = {42.218327,59.493205,41.934509,49.085423,23.414172,57.789951,20.330942,42.331566,-19.902089,23.479039,39.421804,56.295590,39.940728,41.974669,20.274993,2.085171};
// float e_puck_matrix[16] = {56,41,72,37,58,-56,-8,-56,-65,-28,-32,22,54,7,23,46};
int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
float migr[2] = {MIGRATORY_DEST_X, MIGRATORY_DEST_Z};	        // Migration vector
char* robot_name;
float theta_robots[FLOCK_SIZE];
float true_position[FLOCK_SIZE][3];     		// X, Z, Theta of the current robot (true), for debug only
/********** Global varialbes **********/


/********** Localization utilities **********/
/* Definition */
typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;	// Measurements structure

/* Global variables */
static measurement_t  _meas;
// position by gps, accelerometer odometry, encoder odometry, and speed by encoder odometry
static pose_t         _pose, _odo_acc, _odo_enc, _speed_enc;
static pose_t         _pose_origin = {0, 0, 0}; // do not touch.
double last_gps_time_s = 0.0f;
double last_gps_send_time_s = 0.0f;

// Kalman filter matrices for covariances and state (for accelerometer and encoder based kalman)
static gsl_matrix*Cov_acc;
static gsl_matrix*X_acc;
static gsl_matrix*Cov_enc;
static gsl_matrix*X_enc;

// devices
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; 
WbDeviceTag dev_right_motor;

// functions
static void init_position(int time_step, double x_init, double z_init, double h_init);
static void compute_position(int time_step);
static void init_state();
static void init_devices(int ts);
static void controller_get_pose();
static void controller_get_gps();
static double controller_get_heading();
static void controller_get_acc();
static void controller_get_encoder();
static void controller_compute_mean_acc(int ts);
static void controller_compute_initial_mean_acc();
/********** Localization utilities **********/


/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	// Webots initialization
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64);

	// Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
	my_position[0] = -init_x[robot_id];
	my_position[1] = init_z[robot_id];
	migr[0] = my_position[0] + MIGRATORY_DEST_X;
	migr[1] = my_position[1] + MIGRATORY_DEST_Z;
	printf("Reset: robot %d\n",robot_id_u);

	// Positioning mode sanity check
	if (POSITIONING_MODE < POS_ENC_KF || POSITIONING_MODE > POS_GT){
		printf("The input POSITIONING_MODE %d is wrong! Reset to encoder+KF positioning!\n", POSITIONING_MODE);
	}
	init_position(TIME_STEP, my_position[1], my_position[0], my_position[2]); // initialize localization variables
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/* 
 * Limit the speed proportionally.
*/
void limit_vel(int* v1, int* v2, int limit) {
    float limit_f = (float) limit;
    float limit_v1 = (float) *v1;
    float limit_v2 = (float) *v2;
    float max_vel, ratio;
    if (abs(limit_v1) >= abs(limit_v2)){
        if (abs(limit_v1) > limit_f){
           max_vel = abs(limit_v1);
           ratio = max_vel / limit_f;
           limit_v1 = limit_v1 / ratio;
           limit_v2 = limit_v2 / ratio;
        }
    }
    else {// abs(v1) < abs(v2)
        if (abs(limit_v2) > limit_f){
           max_vel = abs(limit_v2);
           ratio = max_vel / limit_f;
           limit_v1 = limit_v1 / ratio;
           limit_v2 = limit_v2 / ratio;
        }
    }
    *v1 = (int)limit_v1; *v2 = (int)limit_v2;
}


/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {
	if (POSITIONING_MODE == POS_ENC_KF){
		my_position[0] = gsl_matrix_get(X_enc,1,0);
		my_position[1] = gsl_matrix_get(X_enc,0,0);
		_odo_enc.heading = fmod(_odo_enc.heading, 2.0*M_PI);
		my_position[2] = _odo_enc.heading;
	}
	else if (POSITIONING_MODE == POS_ACC_KF){
		my_position[0] = gsl_matrix_get(X_acc,1,0);
		my_position[1] = gsl_matrix_get(X_acc,0,0);
		_odo_acc.heading = fmod(_odo_acc.heading, 2.0*M_PI);
		my_position[2] = _odo_acc.heading;
	}
	else if (POSITIONING_MODE == POS_DUMMY_ODO){
		float theta = my_position[2];
		// Compute deltas of the robot
		float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
		float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
		float du = (dr + dl)/2.0;
		float dtheta = (dr - dl)/AXLE_LENGTH;
  
		// Compute deltas in the environment
		float dx = du * sinf(theta);  // lateral movement
		float dz = du * cosf(theta);  // longitudinal movement
  
		// Update position
		my_position[0] += dx;
		my_position[1] += dz;
		my_position[2] += dtheta;
	}
	else if (POSITIONING_MODE == POS_GT){
		// Careful, for debug only!!!
		my_position[0] = -true_position[robot_id][1];
		my_position[1] = true_position[robot_id][0];
		my_position[2] = true_position[robot_id][2];
	}
	else{
		printf("Not implemented positioning mode!\n");
	}
  
	// Keep orientation within 0, 2pi
	if (VERBOSE){
		printf("Robot: %d     ", robot_id);
		printf("Self-estimated X: %.2f, Z: %.2f, Theta: %.4f      ", my_position[0], my_position[1], my_position[2]);
		printf("KF results: X: %.2f, Z: %.2f, Theta: %.4f      ", gsl_matrix_get(X_enc,1,0), gsl_matrix_get(X_enc,0,0), _odo_enc.heading);
		printf("Ground-truth: X: %.2f, Z: %.2f, Theta: %.4f     \n", -true_position[robot_id][1], true_position[robot_id][0], true_position[robot_id][2]);	
	}
}


/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
}


/*
 *  Update speed according to Reynold's rules
 */
void reynolds_rules() {
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
  /* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id)
	    	continue; // don't consider yourself for the average
	    for (j=0;j<2;j++) {
	      rel_avg_speed[j] += relative_speed[i][j];
	      rel_avg_loc[j] += relative_pos[i][j];
	    }
	}
	for (j=0;j<2;j++) {
	      rel_avg_speed[j] /= FLOCK_SIZE-1;
	      rel_avg_loc[j] /= FLOCK_SIZE-1;
	}
        
  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  if (sqrt(pow(rel_avg_loc[0],2)+pow(rel_avg_loc[1],2)) > RULE1_THRESHOLD) {
  	for (j=0;j<2;j++)
  		cohesion[j] = rel_avg_loc[j];
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (k=0;k<FLOCK_SIZE;k++) {
  	if (k != robot_id){
   	// Loop on flockmates only
  	// If neighbor k is too close (Euclidean distance)
  		if(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2) < RULE2_THRESHOLD){
  			for (j=0;j<2;j++) {
  				dispersion[j] -= 1/relative_pos[k][j]; // Relative distance to k
  			}
  		}
  	}
  }

  /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
  for (j=0;j<2;j++)
  	consistency[j] = rel_avg_speed[j];

  //aggregation of all behaviors with relative influence determined by weights
	for (j=0;j<2;j++) {
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	speed[robot_id][1] *= -1; // y axis of webots is inverted
        
	//move the robot according to some migration rule
	if(MIGRATORY_URGE == 0){
	  speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
	  speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
	} else {
		float dist_to_migration;
		float migration_strength;
		dist_to_migration = sqrt(pow(migr[0]-my_position[0],2)+pow(migr[1]-my_position[1],2));

		if (dist_to_migration > MIGRATION_WEAKEN_THRESHOLD){
			// Normal migration
			migration_strength = 1.0;
		}
		else if (dist_to_migration > MIGRATION_FREEZE_THRESHOLD){
			// Weaken migration as the robot goes closer to the migration destination
			migration_strength = (dist_to_migration - MIGRATION_FREEZE_THRESHOLD) / (MIGRATION_WEAKEN_THRESHOLD - MIGRATION_FREEZE_THRESHOLD);
			migration_strength = pow(migration_strength, 2);
		}
		else
			migration_strength = -1.0;

		if (VERBOSE)
			printf("Dist to migration destination: %g, strength: %g, ", dist_to_migration, migration_strength);
		if (migration_strength > 0){
			float migr_x, migr_y;
			migr_x = -(migr[0]-my_position[0]) * MIGRATION_WEIGHT;
			migr_y = (migr[1]-my_position[1]) * MIGRATION_WEIGHT;

			if (VERBOSE){
				printf("migr[0]: %g, my_position[0]: %g      ", migr[0], my_position[0]);
				printf("migr[1]: %g, my_position[1]: %g      ", migr[1], my_position[1]);
				printf("migr_x: %g, migr_y: %g \n", migr_x, migr_y);
			}
			speed[robot_id][0] += migr_x;
			speed[robot_id][1] += migr_y;

		}
		else{
			printf("Robot %d is stopped! \n", robot_id);
		  speed[robot_id][0] *= 0.001;
		  speed[robot_id][1] *= 0.001;
		}
	}
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {

		// get true pos from supervisor, for debug only
		inbuffer = (char*) wb_receiver_get_data(receiver);
		if (inbuffer[0] != 'e'){
			// printf("Robot %d \n",inbuffer[0]- '0');
			int i = inbuffer[0]- '0';
			sscanf(inbuffer,"%1d#%f#%f#%f",&i,&true_position[i][0],&true_position[i][1],&true_position[i][2]);
			wb_receiver_next_packet(receiver);
			true_position[i][2] += M_PI/2;
			if (true_position[i][2] > 2*M_PI) true_position[i][2] -= 2.0*M_PI;
			if (true_position[i][2] < 0) true_position[i][2] += 2.0*M_PI;
			//   printf("Robot %d is in %f %f %f\n",i,true_position[i][0],true_position[i][1],true_position[i][2]);
			continue;
		}

    	// process relative measurements
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);

		double y = message_direction[2];
		double x = message_direction[0];
                      
		theta = -atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
	
		if (VERBOSE){
    		printf("Robot %s, from robot %d, range %g, direction_x: %g, direction_y: %g, x: %g, z: %g. \n",robot_name,other_robot_id,range,x,y,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1]);
			// float l2_error = sqrt(pow(relative_pos[other_robot_id][0] - (true_position[other_robot_id][0] - true_position[robot_id][0]), 2) 
			// + pow(relative_pos[other_robot_id][1] - (true_position[other_robot_id][1] - true_position[robot_id][1]), 2));
			// float l1_error = ABS(relative_pos[other_robot_id][0] - (true_position[other_robot_id][0] - true_position[robot_id][0])) 
			// + ABS(relative_pos[other_robot_id][1] - (true_position[other_robot_id][1] - true_position[robot_id][1]));
			// printf("This relative measurement error: L2: %.3f, L1: %.3f. \n", l2_error, l1_error);
		}

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);		
		 
		wb_receiver_next_packet(receiver);
	}
}


/* The main function */
int main(){ 
	// Initialization
	int msl, msr;					// Wheel speeds, # rotations per 1000 second.
	float msl_w, msr_w;				// Wheel speeds for Webots
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;							// Loop counter
	int distances[NB_SENSORS];		// Array for the distance sensor readings
	int max_sens;					// Store highest sensor value
	int counter;					// Global loop counter

 	reset();						// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	counter = 0;
	
	// Forever
	for(;;){
		// Initialization
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
                
		// Braitenberg obstacle avoidance
		for(i=0;i<NB_SENSORS;i++) {
			distances[i] = wb_distance_sensor_get_value(ds[i]); // Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens > distances[i] ? max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += (int) e_puck_matrix[i] * distances[i];
			bmsl += (int) e_puck_matrix[i+NB_SENSORS] * distances[i];
		}
		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		// Localization
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		// compute_position(TIME_STEP);
		update_self_motion(msl,msr);
		
		process_received_ping_messages();
                      
		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

		compute_position(TIME_STEP);
    
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;

		// Limit wheel speed in the last step
		limit(&msl,MAX_SPEED);
		limit(&msr,MAX_SPEED);
                  
		// Set speed in Webots world
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
    
		// Continue one step
		wb_robot_step(TIME_STEP);

		if (robot_id == 0 && VERBOSE)
			printf("----- Iteration no. %d is finished. -----\n \n", counter);
		counter++;
	}

	// End of the simulation
	wb_robot_cleanup();
	return 0;
}  


/********** Localization utilities **********/
void init_position(int time_step, double x_init, double z_init, double h_init)
{
  init_devices(time_step);
  
  _pose.x = x_init;
  _pose.y = z_init;
  _pose.heading = h_init;
  _pose_origin.x= _pose.x;
  _pose_origin.y= _pose.y;
  _odo_acc.x= _pose.x;
  _odo_acc.y= _pose.y;
  _odo_acc.heading= _pose.heading;
  _odo_enc.x= _pose.x;
  _odo_enc.y= _pose.y;
  _odo_enc.heading= _pose.heading;
  
  odo_reset(time_step,&_pose_origin);
  kalman_reset(time_step);
  init_state(); //initial state variables for kalman filter
  //initial mean acceleration (as calibration takes place when the robot moves at cst speed)
  //improves acceleration odometry in the beginning
  controller_compute_initial_mean_acc();
}

void compute_position(int time_step)
{
   controller_get_pose();
   controller_get_acc();
   controller_get_encoder();
   //get mean values when the robot moves at cst speed
   if((wb_robot_get_time()<TIME_INIT_ACC)&&(wb_robot_get_time()>1))
   {
     controller_compute_mean_acc(time_step);
   }
   //compute odometries and kalman filter based localization
   odo_compute_acc(&_odo_acc, _meas.acc, _meas.acc_mean);
   odo_compute_encoders(&_odo_enc, &_speed_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);

   kalman_compute_acc(X_acc,Cov_acc,(_meas.acc[1]-_meas.acc_mean[1])*cos(_odo_enc.heading),
     (_meas.acc[1]-_meas.acc_mean[1])*sin(_odo_enc.heading),_pose.x,_pose.y,wb_robot_get_time()-last_gps_time_s);
      
   kalman_compute_enc(X_enc,Cov_enc,_speed_enc.x,
     _speed_enc.y,_pose.x,_pose.y,wb_robot_get_time()-last_gps_time_s);
}
void init_state() //declaration of kalman filter input and output variables
{
  // states variables for acc based kalman
  Cov_acc = gsl_matrix_calloc(4, 4);
  gsl_matrix_set(Cov_acc,0,0,0.001);
  gsl_matrix_set(Cov_acc,1,1,0.001);
  gsl_matrix_set(Cov_acc,2,2,0.001);
  gsl_matrix_set(Cov_acc,3,3,0.001);
  X_acc= gsl_matrix_calloc(4, 1);
  gsl_matrix_set(X_acc,0,0,_pose.x);
  gsl_matrix_set(X_acc,1,0,_pose.y);
  
  // state variables for enc based kalman
  Cov_enc = gsl_matrix_calloc(2, 2);
  gsl_matrix_set(Cov_enc,0,0,0.001);
  gsl_matrix_set(Cov_enc,1,1,0.001);
  X_enc= gsl_matrix_calloc(2, 1);
  gsl_matrix_set(X_enc,0,0,_pose.x);
  gsl_matrix_set(X_enc,1,0,_pose.y);
}

void init_devices(int ts) {
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);
  
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);
  
  
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder,  ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
}

void controller_get_pose()
{
  // Call the function to get the gps measurements
  double time_now_s = wb_robot_get_time();
  if (time_now_s - last_gps_time_s > 1.0f) {
    last_gps_time_s = time_now_s;
    controller_get_gps();
  
    _pose.x = _meas.gps[0];
    _pose.y = -(_meas.gps[2]);
    _pose.heading = controller_get_heading();
  
    if(VERBOSE_POSE)
      printf("ROBOT pose : %g %g %g\n", _pose.x , _pose.y , RAD2DEG(_pose.heading));
  }
}

void controller_get_gps()
{
  // To Do : store the previous measurements of the gps (use memcpy)
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  // To Do : get the positions from webots for the gps. Uncomment and complete the following line Note : Use dev_gps
  const double * gps_position = wb_gps_get_values(dev_gps);
  // To Do : Copy the gps_position into the measurment structure (use memcpy)
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

  if(VERBOSE_GPS)
    printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
}

double controller_get_heading()
{
  // To Do : implement your function for the orientation of the robot. Be carefull with the sign of axis y !
  double delta_x = _meas.gps[0] - _meas.prev_gps[0];

  double delta_y = -(_meas.gps[2] - _meas.prev_gps[2]);

  // To Do : compute the heading of the robot ( use atan2 )
  
  double heading = atan2(delta_y, delta_x);
  
  return heading;
}

void controller_get_acc()
{
  // To Do : Call the function to get the accelerometer measurements. Uncomment and complete the following line. Note : Use dev_acc
  const double * acc_values = wb_accelerometer_get_values(dev_acc);

  // To Do : Copy the acc_values into the measurment structure (use memcpy)
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if(VERBOSE_ACC)
    printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
}

void controller_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

  if(VERBOSE_ENC)
    printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
}

void controller_compute_mean_acc(int ts)
{
  static int count = 0;
  
  count++;
  
  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 3; i++)  
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 1) + _meas.acc[i]) / (double) count;
  }
  else //reset mean values
  {
    for(int i = 0; i < 3; i++)
        _meas.acc_mean[i] = 0.0;
  }
  if( count == (int) (TIME_INIT_ACC / (double) ts * 1000) )
    printf("Accelerometer initialization Done ! \n");

  if(VERBOSE_ACC_MEAN)
        printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
}

void controller_compute_initial_mean_acc()
{
  _meas.acc_mean[0] = 6.56983e-05;
  _meas.acc_mean[1] = 0.00781197;
  _meas.acc_mean[2] = 9.81;
  //printf("bias set \n");
}
/********** Localization utilities **********/