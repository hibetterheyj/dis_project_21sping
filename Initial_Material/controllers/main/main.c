#include "localization.h"
#include "trajectories.h"

int main()
{
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  AAAAAA init_devices(time_step);
  AAAAAA init_position();
  AAAAAA odo_reset(time_step,&_pose_origin);
  AAAAAA kalman_reset(time_step);
  controller_init_log("logs.csv"); //logs file
  AAAAAA init_state(); //initial state variables for kalman filter
  
  //initial mean acceleration (as calibration takes place when the robot moves at cst speed)
  //improves acceleration odometry in the beginning
  
  while ((wb_robot_step(time_step) != -1)&& wb_robot_get_time()<130)  {
    BBBBB controller_get_pose();
    BBBBB controller_get_acc();
    BBBBB controller_get_encoder();
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
    //printf(" %g \n",gsl_matrix_get(X_enc, 0, 0));
    //printf("%g\n", wb_robot_get_time());

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