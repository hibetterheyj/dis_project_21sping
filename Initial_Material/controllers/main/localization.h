#ifndef LOCALIZATION_H
#define LOCALIZATION_H

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

void init_position();
void init_state();
void init_devices(int ts);
void controller_get_pose();
void controller_get_gps();
double controller_get_heading();
void controller_get_acc();
void controller_get_encoder();
void controller_compute_mean_acc(int ts);
void controller_compute_initial_mean_acc();
void controller_print_log(double time);
void controller_init_log(const char* filename);
void send_mea();

int run();

#endif