#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


// ********** Fixed parameters **********
#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  10	  // Size of flock
#define SINGLE_FLOCK_SIZE 5
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)

#define ABS(x) ((x>=0)?(x):-(x))
// ********** Fixed parameters **********


// ********** Tunable parameters **********
#define VERBOSE true // Print diagnosis information

#define RULE1_THRESHOLD     0.01   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (5.0/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.01/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEAKEN_THRESHOLD     0.5   // Min. distance w/o migration weakening penalty
#define MIGRATION_FREEZE_THRESHOLD     0.1   // Slow down the robot if it's within this distance to migration destination
#define MIGRATION_WEIGHT    (0.6/10)   // Weight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

// Please note that X & Z axes are aligned with the robots' longitudinal and lateral motion directions at initial position respectively.
#define MIGRATORY_DEST_X  0.0  // X-coordinate of migration destination
#define MIGRATORY_DEST_Z  2.0  // Z-coordinate of migration destination 
// ********** Tunable parameters **********

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {MIGRATORY_DEST_X, MIGRATORY_DEST_Z};	        // Migration vector
char* robot_name;

float theta_robots[FLOCK_SIZE];

float true_position[FLOCK_SIZE][3];     		// X, Z, Theta of the current robot (true), for debug only

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	
	//get motors
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

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
  
	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
  
  	if (robot_id >= SINGLE_FLOCK_SIZE)
		my_position[1] = -1.9;
	else
		my_position[1] = -0.1;

	int robot_id_ingroup = robot_id % 5;
	switch(robot_id_ingroup){
		case 0:
			my_position[0] = 0.0;
			break;
		case 1:
			my_position[0] = -0.1;
			break;
		case 2:
			my_position[0] = 0.1;
			break;
		case 3:
			my_position[0] = -0.2;
			break;
		case 4:
			my_position[0] = 0.2;
			break;
	}
  printf("Reset: robot %d\n",robot_id_u);
  
  if (robot_id >= SINGLE_FLOCK_SIZE){
	migr[0] = my_position[0] - MIGRATORY_DEST_X;
	migr[1] = my_position[1] + MIGRATORY_DEST_Z;
  }
  else{
  	migr[0] = my_position[0] - MIGRATORY_DEST_X;
	migr[1] = my_position[1] - MIGRATORY_DEST_Z;
  }

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
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
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

	// Debug, use ground-truth position
	my_position[0] = -true_position[robot_id][1];
	my_position[1] = true_position[robot_id][0];
	my_position[2] = true_position[robot_id][2];
  
	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;

	if (VERBOSE){
		printf("Robot: %d     ", robot_id);
		printf("Self-estimated X: %g, Z: %g, Theta: %g      ", my_position[0], my_position[1], my_position[2]);
		printf("Ground-truth: X: %g, Z: %g, Theta: %g \n", -true_position[robot_id][1], true_position[robot_id][0], true_position[robot_id][2]);	
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
	// limit(msl,MAX_SPEED);
	// limit(msr,MAX_SPEED);
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
	int idx_start = robot_id >= SINGLE_FLOCK_SIZE ? SINGLE_FLOCK_SIZE : 0;
	int idx_end = robot_id >= SINGLE_FLOCK_SIZE ? SINGLE_FLOCK_SIZE*2 : SINGLE_FLOCK_SIZE;

	for(i=idx_start; i<idx_end; i++) {
		if (i == robot_id)
	    	continue; // don't consider yourself for the average

	    for (j=0;j<2;j++) {
	      rel_avg_speed[j] += relative_speed[i][j];
	      rel_avg_loc[j] += relative_pos[i][j];
	    }
	}
	for (j=0;j<2;j++) {
	      rel_avg_speed[j] /= SINGLE_FLOCK_SIZE-1;
	      rel_avg_loc[j] /= SINGLE_FLOCK_SIZE-1;
	}
        
  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  // for (j=0;j<2;j++)
  // 	cohesion[j] = rel_avg_loc[j];

  if (sqrt(pow(rel_avg_loc[0],2)+pow(rel_avg_loc[1],2)) > RULE1_THRESHOLD) {
  	for (j=0;j<2;j++)
  		cohesion[j] = rel_avg_loc[j];
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (k=idx_start;k<idx_end;k++) {
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
			migration_strength = pow(migration_strength, 3);
		}
		else
			migration_strength = -1.0;

		printf("Dist to migration destination: %g, strength: %g \n", dist_to_migration, migration_strength);
		if (migration_strength > 0){
			float migr_x, migr_y;
			migr_x = (migr[0]-my_position[0]) * MIGRATION_WEIGHT * migration_strength;
			migr_y = (migr[1]-my_position[1]) * MIGRATION_WEIGHT * migration_strength;

			// printf("migr[0]: %g, my_position[0]: %g \n", migr[0], my_position[0]);
			// printf("migr[1]: %g, my_position[1]: %g \n", migr[1], my_position[1]);
			printf("migr_x: %g, migr_y: %g \n", migr_x, migr_y);
			// speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
			// speed[robot_id][1] += (migr[1]-my_position[1]) * MIGRATION_WEIGHT;
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
      // printf("Robot %d is in %f %f %f\n",i,true_position[i][0],true_position[i][1],true_position[i][2]);
      continue;
    }

    // process relative measurements
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);

		double y = message_direction[2];
		double x = message_direction[1];
                      
		theta = -atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
	
		// if (VERBOSE)
    	// printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);		
		 
		wb_receiver_next_packet(receiver);
	}
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value

	int counter;
	
 	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	counter = 0;
	
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		// indispensible for flocking!
		for(i=0;i<NB_SENSORS;i++) {
			distances[i] = wb_distance_sensor_get_value(ds[i]); // Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens > distances[i] ? max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
		
		process_received_ping_messages();
                      
		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);
    
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
                  
		// Set speed
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
}  
  
