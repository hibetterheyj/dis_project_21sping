/*****************************************************************************/
/* File: formation_graph_controller.c
- [finished] set a goal position for thPIe controller with threshold
- [finished] add obstacle avoidance from the code
- [finished] add leader-based formation control
- [finished] extend to five robot!
- [finished] control with odometry information
- [finished] try to max the speed before the goal while limit the speed when near the goal using `adjust_vel`
- [used but with very bad performance] use relative pose instead of true pose
/*****************************************************************************/

/**************************************************/
/* LIBRARY */
/**************************************************/
/* standard library */
#include <stdio.h>
#include <math.h>
#include <string.h>

/* webots library */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

/* gsl library */
#include <gsl/gsl_vector.h> // vector
#include <gsl/gsl_matrix.h> // matrix
#include <gsl/gsl_blas.h> // matrix operation

/* custom library */
#include "../../../localization_library/odometry.h"
#include "../../../localization_library/kalman.h"

/**************************************************/
/* PARAMETERS */
/**************************************************/
#define NB_SENSORS           8
/* Formation parameters */
#define FLOCK_SIZE	  2 // Size of formation
#define EDGE_SIZE	  1 // Size of edges
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define INIT_TYPE_LOCAL true // if true, init with custom setup; else (false), init with supervisor
// sensor for localization
#define TIME_INIT_ACC 3         // Time in second
#define VERBOSE false       // Print verbosely
#define VERBOSE_POSE false      // Print pose values
#define VERBOSE_ACC false       // Print accelerometer values
#define VERBOSE_ACC_MEAN false  // Print accelerometer mean values
#define VERBOSE_ENC false       // Print encoder values
#define VERBOSE_GPS false       // Print GPS values
#define VERBOSE_SPEED true       // Print SPEED values

#define LEADER_MODE false // use leader mode
#define ODO_TYPE 2 // 1: ACC-based KF, 2: ENC-based KF, 3: ACC-based position+ENC-based heading
#define USE_TRUE false

/**************************************************/
/* WEBOTS INITIALIZATION */
/**************************************************/
WbDeviceTag dev_gps;
WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag ds[NB_SENSORS];
WbDeviceTag receiver;
WbDeviceTag emitter;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_acc;

// init and goal position
// from -1.9 -> 2.4 = 4
float goal_distance[2] = {5, -0.0};
float goal_pos[2];
// initial position of 5 robots
float init_x[5] = {-2.9, -2.9, -2.9, -2.9, -2.9};
float init_y[5] = {0.0, 0.1, -0.1, 0.2, -0.2};
int DELTA_T=64;	// [ms] Length of time step

// pi(d) control towards goal
float err[2];
float prev_err[2];
float integrator[2];
float k_p = 0.5; // 0.1
float k_i = 0.0; // 0.0

/* TUNING PARAMETERS */
float THRESHOLD=0.3;	// Convergence threshold
// int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};
// weight for generated obstacle speed
float ob_weight = 4;

// bias vector
float bias_x[5] = {0, 0, 0, 0, 0};
// float bias_y[5] = {0.0, 0.1, -0.1, 0.2, -0.2};
float bias_y[5] = {0.0, 0.1, -0.1, 0.2, -0.2};

double weight_coefficient = 4;

/**************************************************/
/* ROBOT VARIABLES INITIALIZATION */
/**************************************************/
int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float true_position[FLOCK_SIZE][3]; // true position for algorithm test
float global_position[FLOCK_SIZE][3]; // true position for algorithm test
float speed[FLOCK_SIZE][2];		// Speeds calculated with graph-based formation
float final_speed[FLOCK_SIZE][2];
float goal_speed[2];
// float relative_speed[FLOCK_SIZE][2];
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
char* robot_name;

/* localization */
static void controller_get_pose();
static void controller_get_gps();
static double controller_get_heading();
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
static measurement_t  _meas;
static pose_t _pose, _odo_acc, _odo_enc, _speed_enc;
static pose_t _pose_origin = {0, 0, 0}; // do not touch.
double last_gps_time_s = 0.0f;
double last_gps_send_time_s = 0.0f;
// kalam filter matrices for covariances and state (for accelerometer and encoder based kalman)
static gsl_matrix*Cov_acc, *X_acc, *Cov_enc, *X_enc;

/**************************************************/
/* UTILITY FUNCTIONS */
/**************************************************/
// https://www.gnu.org/software/gsl/doc/html/vectors.html#c.gsl_matrix
int num_rows(gsl_matrix * A){
    int r = A->size1;
    return r;
}

int num_cols(gsl_matrix * A){
    int r = A->size2;
    return r;
}

/* Compute weight matrix, W */
void compute_weight(gsl_matrix * weight, double weight_coefficient, int edge_size){
    // temp matrix (FLOCK_SIZE) xE
	for (int i = 0; i<edge_size; i++){
		gsl_matrix_set(weight, i, i, weight_coefficient);
	}
}

/* Compute laplacian matrix, I * W * I^T */
void compute_laplacian(gsl_matrix * incidence, gsl_matrix * weight, gsl_matrix * laplacian){
    int edge_size = num_rows(weight);
    int flock_size = num_rows(laplacian);
    gsl_matrix * temp  = gsl_matrix_alloc (flock_size, edge_size);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, incidence, weight,
                    0.0, temp);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans,
                    1.0, temp, incidence,
                    0.0, laplacian);
    gsl_matrix_free (temp);
}

void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter, out,strlen(out)+1);
}

/* Keep given int number within interval {-limit, limit} */
void limit_speed(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

void limit_vel(int* v1, int* v2, int limit) {
    // printf("(v1, v2) = (%d, %d) \n", *v1 , *v2);
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
    //printf("updated (v1, v2) = (%d, %d) \n", *v1 , *v2);
}

void adjust_vel(int* v1, int* v2, int limit) {
    // printf("(v1, v2) = (%d, %d) \n", *v1 , *v2);
    float limit_f = (float) limit;
    float limit_v1 = (float) *v1;
    float limit_v2 = (float) *v2;
    float max_vel, ratio;
    if (abs(limit_v1) >= abs(limit_v2)){
        if (abs(limit_v1) < limit_f){
           max_vel = abs(limit_v1);
           ratio = max_vel / limit_f;
           limit_v1 = limit_v1 / ratio;
           limit_v2 = limit_v2 / ratio;
        }
    }
    else {// abs(v1) < abs(v2)
        if (abs(limit_v2) < limit_f){
           max_vel = abs(limit_v2);
           ratio = max_vel / limit_f;
           limit_v1 = limit_v1 / ratio;
           limit_v2 = limit_v2 / ratio;
        }
    }
    *v1 = (int)limit_v1; *v2 = (int)limit_v2;
}

/* Keep given int number within interval {-limit, limit} */
float limit_angle(float angle) {
	while(angle > 180 || angle < -180){
		if (angle > 180){
			angle -= 360;
		}
		else if (angle < -180){
			angle += 360;
		}
	}
	return angle;
}

float limit_rad(float rad) {
	while(rad > 2*M_PI || rad < 0){
		if (rad > 2*M_PI){
			rad -= 2*M_PI;
		}
		else if (rad < 0){
			rad += 2*M_PI;
		}
	}
	return rad;
}

/**************************************************/
/* ROBOT-RELATED FUNCTIONS */
/**************************************************/
/* Localization */
/* initialization of kalman filter input and output variables */
void init_state_kf() {
    /* states variables for acc based kalman filter */
    Cov_acc = gsl_matrix_calloc(4, 4);
    gsl_matrix_set(Cov_acc,0,0,0.001);
    gsl_matrix_set(Cov_acc,1,1,0.001);
    gsl_matrix_set(Cov_acc,2,2,0.001);
    gsl_matrix_set(Cov_acc,3,3,0.001);
    X_acc= gsl_matrix_calloc(4, 1);
    gsl_matrix_set(X_acc, 0,0, _pose.x);
    gsl_matrix_set(X_acc, 1,0, _pose.y);

    /* state variables for enc based kalman */
    Cov_enc = gsl_matrix_calloc(2, 2);
    gsl_matrix_set(Cov_enc,0,0,0.001);
    gsl_matrix_set(Cov_enc,1,1,0.001);
    X_enc= gsl_matrix_calloc(2, 1);
    gsl_matrix_set(X_enc, 0,0, _pose.x);
    gsl_matrix_set(X_enc, 1,0, _pose.y);
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
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

  if(VERBOSE_GPS)
    printf("ROBOT %d gps is at position: %g %g %g\n", robot_id, _meas.gps[0], _meas.gps[1], _meas.gps[2]);
}

double controller_get_heading()
{
  double delta_x = _meas.gps[0] - _meas.prev_gps[0];
  double delta_y = -(_meas.gps[2] - _meas.prev_gps[2]);
  double heading = atan2(delta_y, delta_x);

  return heading;
}

void controller_get_acc()
{
  const double * acc_values = wb_accelerometer_get_values(dev_acc);
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

void compute_position_kf(int time_step)
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
/* Localization End here */

/* Initialization sensor, motor, receiver, etc. */
void init_device(int time_step){
	robot_name=(char*) wb_robot_get_name();

	// gps
	dev_gps = wb_robot_get_device("gps");
	wb_gps_enable(dev_gps, 1000);
	// motor
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);
	// encoder
	dev_left_encoder = wb_robot_get_device("left wheel sensor");
	dev_right_encoder = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(dev_left_encoder,  time_step);
	wb_position_sensor_enable(dev_right_encoder, time_step);
	// accelerometer
	dev_acc = wb_robot_get_device("accelerometer");
	wb_accelerometer_enable(dev_acc, time_step);
	// receiver & emitter
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");

	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}

	printf("Initialized robot %d\n",robot_id_u);
}

void init_position_kf(int time_step, double x_init, double z_init, double h_init)
{
	// initialize the device using init_device(ts) at first

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
	init_state_kf(); //initial state variables for kalman filter
	//initial mean acceleration (as calibration takes place when the robot moves at cst speed)
	//improves acceleration odometry in the beginning
	controller_compute_initial_mean_acc();
}

/* Initialize robot's position */
void initial_pos(){
	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	// sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	// robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	while (initialized[robot_id] == 0) {
		// Initialize self position with pre-defined array
		my_position[0] = init_x[robot_id];  // x-position
		my_position[1] = init_y[robot_id]; // z-position
		printf("Robot %d pos (%f, %f)\n", robot_id, init_x[robot_id], init_y[robot_id]);
		printf("Init robot pos (%f, %f)\n", my_position[0], my_position[1]);
		my_position[2] = 0; // init theta
		prev_my_position[0] = init_x[robot_id];
		prev_my_position[1] = init_y[robot_id];
		prev_my_position[2] = 0; // init theta
		// goal_distance[2] = {GOAL_X, GOAL_Y}; // Error dynamics
		goal_pos[0] = my_position[0] + goal_distance[0];
		goal_pos[1] = my_position[1] + goal_distance[1];
		err[0] = goal_pos[0] - my_position[0];
		err[1] = -(goal_pos[1] - my_position[1]);
		prev_err[0] = err[0]; prev_err[1] = err[1];
		integrator[0] = 0; integrator[1] = 0;
		initialized[robot_id] = 1;  // initialized = true
	}
}

/* Compute relative ranger and bearing */
void compute_relative_pos() {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[1];

		// jianhao: get true pos from supervisor
		//printf("message received: %s\n", inbuffer[0] != 'e' ? "true":"false");
		//printf("%s\n", inbuffer);
		if (inbuffer[0] != 'e'){
			// printf("Robot %d \n",inbuffer[0]- '0');

			int i = inbuffer[0]- '0';
			sscanf(inbuffer,"%1d#%f#%f#%f",&i,&true_position[i][0],&true_position[i][1],&true_position[i][2]);
			wb_receiver_next_packet(receiver);
			true_position[i][2] += M_PI/2;
			if (true_position[i][2] > 2*M_PI) true_position[i][2] -= 2.0*M_PI;
			if (true_position[i][2] < 0) true_position[i][2] += 2.0*M_PI;
			//printf("Robot %d is in %f %f %f\n",i,true_position[i][0],true_position[i][1],true_position[i][2]);
			continue;
		}

		theta =	-atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		printf("robot %d relative to robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_id, other_robot_id, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1], -atan2(y,x)*180.0/3.141592, limit_angle(my_position[2]*180.0/3.141592));

		// relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		// relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

		wb_receiver_next_packet(receiver);
	}
}

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
			limit_rad(true_position[i][2]);
			// printf("Robot %d is in %f %f %f\n",i,true_position[i][0],true_position[i][1],true_position[i][2]);
			continue;
		}

		// process relative measurements
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);

		double x = message_direction[0];
		double y = message_direction[2];

		printf("message_direction (%f, %f %f)\n",
		message_direction[0], message_direction[1], message_direction[2]);

		// limit_rad(my_position[2]);
		// theta = -atan2(y, x);
		// theta = theta + my_position[2]; // find the relative theta;
		// limit_rad(theta);
		// range = sqrt((1/message_rssi));

		// other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		// prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		// prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		// //relative_pos[other_robot_id][0] = -range*sin(theta);   // relative y pos
		// relative_pos[other_robot_id][0] = range*cos(theta);
		// relative_pos[other_robot_id][1] = -range*sin(theta);  // relative x pos

		limit_rad(my_position[2]);
		theta = atan2(y, x);
		theta = M_PI/2.0 - theta + my_position[2] - M_PI/2.0; // find the relative theta;
		// limit_rad(theta);
		range = sqrt((1/message_rssi));

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][1] = -range*sinf(theta);
		relative_pos[other_robot_id][0] = range*cosf(theta);  // relative x pos

		relative_pos[other_robot_id][0] *= -1.0;
		relative_pos[other_robot_id][1] *= -1.0;

		printf("RobotTheta %f\t", theta);
		global_position[other_robot_id][0] = my_position[0] + relative_pos[other_robot_id][0];
		global_position[other_robot_id][1] = my_position[1] + relative_pos[other_robot_id][1];
		// if (VERBOSE){
		// printf("(relative) Current Robot %s, from robot %d, range %g, direction_x: %g, direction_y: %g, x: %g, y: %g, theta %g, my theta %g\n",
		// robot_name, other_robot_id, range, x, y, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1], limit_angle(-atan2(y,x)*180.0/M_PI), limit_angle(my_position[2]*180.0/M_PI));
    		  // printf("(Relative) position of robot %d: (%f, %f)\n",
    		  // other_robot_id, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1]);
                                    // printf("(Global) position of current robot %d: (%f, %f)\n",
                                    // robot_id, global_position[robot_id][0], global_position[robot_id][1]);
                                    //printf("(Global) estimated position of robot %d: (%f, %f)\t",
    		  //other_robot_id, global_position[other_robot_id][0], global_position[other_robot_id][1]);
                                    //printf("global true position of robot %d: (%f, %f)\n",
    		  //other_robot_id, true_position[other_robot_id][0], true_position[other_robot_id][1]);
    		  //printf("L2 error of robot %d: %f\n", other_robot_id,
    		      //sqrt(pow(global_position[other_robot_id][0]-true_position[other_robot_id][0], 2) + pow(global_position[other_robot_id][1]-true_position[other_robot_id][1], 2)));

                                  // }

		// relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		// relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

		wb_receiver_next_packet(receiver);
	}
}

/* Compute laplacian functions */
void update_laplacian(void){
  printf("TODO");
}

/* Advanced method to updates robot position with Kalman filter */
void update_self_motion(int msl, int msr) {

	limit_rad(_odo_enc.heading);
	// use true position
	if (USE_TRUE){
	    my_position[0] = true_position[robot_id][0];
	    my_position[1] = true_position[robot_id][1];
	    my_position[2] = true_position[robot_id][2] + M_PI/2.0;
	}
	else {
	    // Use KF results
	    if (ODO_TYPE == 1){
		// /* use ACC-based KF */
		my_position[0] = gsl_matrix_get(X_acc,0,0);
		my_position[1] = -gsl_matrix_get(X_acc,1,0);
		my_position[2] = limit_rad(_odo_acc.heading);
	    }
	    else if (ODO_TYPE == 2){
		// /* use ENC-based KF */
		my_position[0] = gsl_matrix_get(X_enc,0,0);
		my_position[1] = -gsl_matrix_get(X_enc,1,0);
  		my_position[2] = limit_rad(_odo_enc.heading) + M_PI/2.0;
	    }
	    else if (ODO_TYPE == 3){
		// /*use ACC-based KF in position and ENC-based in heading */
		my_position[0] = gsl_matrix_get(X_acc,0,0);
		my_position[1] = -gsl_matrix_get(X_acc,1,0);
		my_position[2] = limit_rad(_odo_enc.heading);
	    }
	}

	printf("ACC estimated X: %.2f, Y: %.2f, Theta: %.2f\n", gsl_matrix_get(X_acc, 0, 0), -gsl_matrix_get(X_acc, 1, 0), limit_rad(_odo_acc.heading));
	printf("ENC estimated X: %.2f, Y: %.2f, Theta: %.2f\n", gsl_matrix_get(X_enc, 0, 0), -gsl_matrix_get(X_enc, 1, 0), limit_rad(_odo_enc.heading));
	global_position[robot_id][0] = my_position[0];
	global_position[robot_id][1] = my_position[1];
	printf("(Global) Current robot %d: (%f, %f)\n", robot_id, global_position[robot_id][0], global_position[robot_id][1]);

	// new: update errors
	prev_err[0] = err[0]; prev_err[1] = err[1];
	// err[0] = goal_pos[0] - true_position[robot_id][0];
	// err[1] = (goal_pos[1] - true_position[robot_id][1]);
	err[0] = goal_pos[0] - my_position[0];
	err[1] = (goal_pos[1] - my_position[1]);
	// printf("x_err = x_goal - x (%f = %f - %f)\n", err[0], goal_pos[0], true_position[robot_id][0]);
	// printf("y_err = y_goal - y (%f = %f - %f)\n", err[1], goal_pos[1], true_position[robot_id][1]);
	printf("x_err = x_goal - x (%f = %f - %f)\n", err[0], goal_pos[0], my_position[0]);
	printf("y_err = y_goal - y (%f = %f - %f)\n", err[1], goal_pos[1], my_position[1]);
}

/* Compute wheel speeds arcording to graph */
void compute_wheel_speeds(int *msl, int *msr, gsl_matrix * laplacian){
	// for(int j = 0; j < FLOCK_SIZE; j++){
  	    // for(int i = 0; i < FLOCK_SIZE; i++){
  	        // printf("GSL matrix laplacian(%d. %d) = %f\n", i, j, gsl_matrix_get(laplacian, i, j));
  	    // }
	// }

	// x'= -L(x-b), Same for y
	//laplacian[robot_id:] * (x-b)
	// init with speed every time with zero
	speed[robot_id][0] = 0; speed[robot_id][1] = 0;
	for(int j = 0; j < FLOCK_SIZE; j++){
		// only need to show available connection
		if (j == robot_id) {
			printf("true_position: (%f, %f)\n", true_position[robot_id][0], true_position[robot_id][1]);
			// true position
			if (USE_TRUE){
			    speed[robot_id][0] -= gsl_matrix_get (laplacian, robot_id, j) * (true_position[j][0] - bias_x[j]);
			    speed[robot_id][1] -= gsl_matrix_get (laplacian, robot_id, j) * (true_position[j][1] - bias_y[j]);
                                                   }
                                                   else {
                                                       // global position
                                                       speed[robot_id][0] -= gsl_matrix_get (laplacian, robot_id, j) * (global_position[j][0] - bias_x[j]);
			    speed[robot_id][1] -= gsl_matrix_get (laplacian, robot_id, j) * (global_position[j][1] - bias_y[j]);
			}
		}
		else {
			if (gsl_matrix_get (laplacian, robot_id, j) == 0){
				continue;
			}
			else {// connect with current vertex
			    if (USE_TRUE){
			        // true position
			        speed[robot_id][0] -= gsl_matrix_get (laplacian, robot_id, j) * (true_position[j][0] - bias_x[j]);
			        speed[robot_id][1] -= gsl_matrix_get (laplacian, robot_id, j) * (true_position[j][1] - bias_y[j]);
			    }
			    else {
			        // global position
			        speed[robot_id][0] -= gsl_matrix_get (laplacian, robot_id, j) * (global_position[j][0] - bias_x[j]);
			        speed[robot_id][1] -= gsl_matrix_get (laplacian, robot_id, j) * (global_position[j][1] - bias_y[j]);
			    }
			}
		}
	}

	// goal_speed
	float Ts = DELTA_T/1000; // (s)
	// if (err[0] > 0.05){
	    integrator[0] = integrator[0] + (Ts/2) * (err[0] + prev_err[0]);
	    goal_speed[0] = k_p * err[0] + k_i * integrator[0];
	// } else{
	    // goal_speed[0] = 0;
	// }
	// if (err[1] > 0.05){
	    integrator[1] = -(integrator[1] + (Ts/2) * (err[1] + prev_err[1]));
	    goal_speed[1] = k_p * err[1] + k_i * integrator[1];
	    // goal_speed[1] = -goal_speed[1];
	// } else{
	    // goal_speed[1] = 0;
	// }
	// goal_speed[0] = k_p * err[0];
	// goal_speed[1] = k_p * err[1]; // TODO: different from y axis
	// goal_speed[0] = 0;
	// goal_speed[1] = 0;
	printf("Error (x, y) =  (%f, %f)\n", err[0], err[1]);
        	if (LEADER_MODE == true){
				if (robot_id % 5 == 0){
				final_speed[robot_id][0] = goal_speed[0];
				final_speed[robot_id][1] = goal_speed[1];
				}
				else{
					final_speed[robot_id][0] = speed[robot_id][0];
					final_speed[robot_id][1] = speed[robot_id][1];
				}
        	}
        	else {
				final_speed[robot_id][0] = speed[robot_id][0] + goal_speed[0];
				final_speed[robot_id][1] = speed[robot_id][1] + goal_speed[1];
			}
	if (VERBOSE_SPEED){
  	    printf ("X-formation, goal, final: (%f, %f, %f) \n", speed[robot_id][0], goal_speed[0], final_speed[robot_id][0] );
  	    printf ("Y-formation, goal, final: (%f, %f, %f) \n", speed[robot_id][1], goal_speed[1], final_speed[robot_id][1] );
	}
	// printf ("speed and angle is: (%f, %f) %f\n", speed[robot_id][0], speed[robot_id][1],true_position[robot_id][2]);
	//printf ("Current speed of agent %d (x, y): (%f, %f) \n", robot_id, speed[robot_id][0], speed[robot_id][1]);

	// Compute speed in global coordinate
	// use relative positioning
	// float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	// float y = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // y in robot coordinates

                 float x, y;
	if (USE_TRUE){
		printf("robot %d true heading %f\n", robot_id, true_position[robot_id][2]);
		x = final_speed[robot_id][0]*cosf(true_position[robot_id][2] - 1.57) + final_speed[robot_id][1]*sinf(true_position[robot_id][2] - 1.57); // x in robot coordinates
		y = -final_speed[robot_id][0]*sinf(true_position[robot_id][2] - 1.57) + final_speed[robot_id][1]*cosf(true_position[robot_id][2] - 1.57); // y in robot coordinates
	} else {
		printf("robot %d true heading %f\n", robot_id, true_position[robot_id][2]);
		// x = final_speed[robot_id][0]*cosf(global_position[robot_id][2] - 1.57) + final_speed[robot_id][1]*sinf(global_position[robot_id][2] - 1.57); // x in robot coordinates
		// y = -final_speed[robot_id][0]*sinf(global_position[robot_id][2] - 1.57) + final_speed[robot_id][1]*cosf(global_position[robot_id][2] - 1.57); // y in robot coordinates

		float tmp_x = final_speed[robot_id][0];
		float tmp_y = final_speed[robot_id][1];
		final_speed[robot_id][0] = -tmp_y;
		final_speed[robot_id][1] = tmp_x;
		x = final_speed[robot_id][0]*cosf(global_position[robot_id][2])  + final_speed[robot_id][1]*sinf(global_position[robot_id][2]); // x in robot coordinates
		y = -final_speed[robot_id][0]*sinf(global_position[robot_id][2]) + final_speed[robot_id][1]*cosf(global_position[robot_id][2]); // y in robot coordinates

	}
	
                 printf ("Current speed (msl, msr) of agent %d: (%d, %d) \n", robot_id, *msl, *msr);

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = -atan2(x, y);	  // Orientation of the wanted position

	// relative localization system to compute forward and rotational speed
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit_vel(msl, msr, MAX_SPEED);
	// printf ("range and bearing is: (%f, %f) \n",range, bearing);
	//printf ("Current speed (msl, msr) of agent %d: (%d, %d) \n", robot_id, msl, msr);
}

/*************************/
/* MAIN */
/*************************/
int main(){
	/* Variables setup */
	// graph-based controller matrix
	gsl_matrix_view incidence_view;
	gsl_matrix * weight  = gsl_matrix_alloc (EDGE_SIZE, EDGE_SIZE);
	gsl_matrix * laplacian  = gsl_matrix_alloc (FLOCK_SIZE, FLOCK_SIZE);
	// Incidence Matrix V (FLOCK_SIZE) xE
	double incidence21[] = {-1, 1};
	incidence_view  = gsl_matrix_view_array(incidence21, 2, 1);

	// Weight Matrix E x E
	compute_weight(weight, weight_coefficient, EDGE_SIZE);

	// Laplacian Matrix VxV I * W * I^T
	compute_laplacian(&incidence_view.matrix, weight, laplacian);
	// motor speed left and right
	int msl=0, msr=0;
	float msl_w, msr_w;
	//float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	// float *rbbuffer;                  // buffer for the range and bearing
	// int i,initialized;

	/* Webots init, device, and odom setup */
	wb_robot_init();
	int time_step = wb_robot_get_basic_time_step();
	//odo_reset(time_step);

	// initialize device
	init_device(time_step);

	// Initializing the robot's position from local settings
	initial_pos();

	// initialize localization variables
	// TODO: test!!! y axis!!!
	printf("Input init_position_kf robot pos (%f, %f, %f)\n", my_position[0], my_position[1], my_position[2]);
	init_position_kf(time_step, my_position[0], -my_position[1], my_position[2]);

	//improves acceleration odometry in the beginning
	controller_compute_initial_mean_acc();

	//for(;;){
	int cnt = 0;
	while (true) {
		printf("##### %d-step for robot %d\n", cnt, robot_id);

		/* I. Obstacle avoidance */
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		  //printf("current bmsr (%d), current sensor %d, current dist (%d), weight  (%d)\n", bmsr, sensor_nb, distances[sensor_nb], Interconn[sensor_nb]) ;
		}

		// /* Adapt Braitenberg values (empirical tests) */
		//printf("(bmsr, bmsl) =  (%d, %d)\n", bmsl, bmsr);
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		printf("(bmsr, bmsl) =  (%d, %d)\n", bmsl, bmsr);

		/* II. Udpate position from receiver */
		send_ping();  // sending a ping to other robot

		// Compute self position & speed
		// 1. update self's position from kalman filter/ encoder info
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];

		// update_self_motion_naive(msl, msr);
		update_self_motion(msl,msr);

		compute_position_kf(time_step); // in the localization.c file

		// 2. update others position from receiver
		// compute_relative_pos();
		process_received_ping_messages();

		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

		// III. Update with graph-based control
		// update_laplacian(); // update with range and bearing position
		// not use the first iteration (discarded)
		compute_wheel_speeds(&msl, &msr, laplacian);

		// printf ("left speed %d + %d  \n",msl, bmsl);
		// printf ("right speed %d + %d  \n",msr, bmsr);
		msl += ob_weight * bmsl;
		msr += ob_weight * bmsr;
		if (VERBOSE_SPEED){
		    printf ("left speed (final) %d += %d\n",msl, bmsl);
		    printf ("right speed (final) %d += %d\n",msr, bmsr);
		}
		// try to maxmize speed as fast as possible before reaching
		// limit speed when close to the target!!!
		float overall_err = sqrt(err[0] * err[0] + err[1] * err[1]);
		if (overall_err<THRESHOLD){
		    adjust_vel(&msl, &msr, MAX_SPEED*overall_err);
		} else{
  		    adjust_vel(&msl, &msr, MAX_SPEED);
		}

		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		if (VERBOSE_SPEED){
		     printf ("Current wheel speed (msl_w, msr_w) of agent %d: (%f, %f) \n", robot_id, msl_w, msr_w);
		}
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		wb_robot_step(DELTA_T);               // Executing the simulation for 64ms
		cnt++;
	}
}
