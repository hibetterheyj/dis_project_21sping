// generic libraries
#include <stdio.h>
#include <string.h>
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

// webots libraries
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>

// custom files
#include "trajectories.h"
#include "odometry.h"
#include "kalman.h"

// constants
#define MAX_SPEED 1000          // Maximum speed 
#define INC_SPEED 5             // Increment not expressed in webots 
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_INIT_ACC 3         // Time in second

// verbose flags
#define VERBOSE_GPS false        // Print GPS values
#define VERBOSE_ACC false       // Print accelerometer values
#define VERBOSE_ACC_MEAN false  // Print accelerometer mean values
#define VERBOSE_POSE false      // Print pose values
#define VERBOSE_ENC false       // Print encoder values

// structure
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
} measurement_t;

// variables
static measurement_t  _meas;
static pose_t         _pose, _odo_acc, _odo_enc;
static pose_t         _pose_origin = {-2.9, 0, 0};
double last_gps_time_s = 0.0f;
double last_gps_send_time_s = 0.0f;
double message[8] ;
static FILE *fp;
static gsl_matrix*Cov;
static gsl_matrix*X;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; 
WbDeviceTag dev_right_motor; 
WbDeviceTag emitter;		// Handle for the emitter node

// devices
static void controller_get_pose();
static void controller_get_gps();
static double controller_get_heading();
static void controller_get_acc();
static void controller_get_encoder();
static void controller_compute_mean_acc(int ts);
static void controller_compute_initial_mean_acc();
static void init_devices(int ts);
static void send_mea();

// logs
static void controller_print_log(double time);
static void controller_init_log(const char* filename);


//static void test_gsl();
static void init_state();

char* robot_name;


int main() 
{
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);
  odo_reset(time_step);
  kalman_reset(time_step);
  controller_init_log("logs.csv"); //logs file
  init_state(); //initial state variables for kalman filter
  
  //initial mean acceleration (as calibration takes place when the robot moves at cst speed)
  //improves acceleration odometry in the beginning
  controller_compute_initial_mean_acc();
  
  while (wb_robot_step(time_step) != -1)  {
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
    odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);

    kalman_compute_acc(X,Cov,(_meas.acc[1]-_meas.acc_mean[1])*cos(_odo_enc.heading),
      (_meas.acc[1]-_meas.acc_mean[1])*sin(_odo_enc.heading),_pose.x,_pose.y,wb_robot_get_time()-last_gps_time_s);
    //printf(" %g \n",gsl_matrix_get(X, 0, 0));


    // send measurements to supervisor
    send_mea();

    // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor);
    //trajectory_2(dev_left_motor, dev_right_motor);      
    controller_print_log(wb_robot_get_time());
  }
  if(fp != NULL)
    fclose(fp);
    // End of the simulation
  wb_robot_cleanup();
  return 0;
}

void init_state() //declaration of kalman filter input and output variables
{
  Cov = gsl_matrix_calloc(4, 4);
  gsl_matrix_set(Cov,0,0,0.001);
  gsl_matrix_set(Cov,1,1,0.001);
  gsl_matrix_set(Cov,2,2,0.001);
  gsl_matrix_set(Cov,3,3,0.001);
  X= gsl_matrix_calloc(4, 1);
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
  
  
  emitter = wb_robot_get_device("emitter");
  if (emitter==0) printf("miss emitter\n");
  if (emitter!=0) printf("emitter works\n");
}

void controller_get_pose()
{
  // Call the function to get the gps measurements
  double time_now_s = wb_robot_get_time();
  if (time_now_s - last_gps_time_s > 1.0f) {
    last_gps_time_s = time_now_s;
    controller_get_gps();

    // To Do : Fill the structure pose_t {x, y, heading}. Use the _pose_origin.
    _pose.x = _meas.gps[0] - _pose_origin.x;
    _pose.y = -(_meas.gps[2] - _pose_origin.y);
    _pose.heading = controller_get_heading() + _pose_origin.heading;
  
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

void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            time, _pose.x, _pose.y , _pose.heading, _meas.gps[0], _meas.gps[1], 
      _meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
      _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading,
      gsl_matrix_get(X,0,0), gsl_matrix_get(X,1,0), gsl_matrix_get(X,2,0), gsl_matrix_get(X,3,0));
  }

}


void controller_init_log(const char* filename)
{
  fp = fopen(filename,"w");

  fprintf(fp, "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kal_x; kal_y; kal_vx; kal_vy\n");
}

void send_mea() {
  double time_now_s = wb_robot_get_time();
  if (time_now_s - last_gps_send_time_s > 1.0f) {
    last_gps_send_time_s = time_now_s;
    message[0] = _meas.gps[0];
    message[1] = _meas.gps[2];
    
  }
  message[2] = _odo_acc.x;
  message[3] = _odo_acc.y;
  message[4] = _odo_enc.x;
  message[5] = _odo_enc.y;
  message[6] = gsl_matrix_get(X,0,0);
  message[7] = gsl_matrix_get(X,1,0);
  wb_emitter_send(emitter,message,8*sizeof(double)); 
}