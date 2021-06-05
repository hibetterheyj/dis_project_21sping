#ifndef KALMAN_H
#define KALMAN_H

typedef struct 
{
  gsl_matrix*Cov;
  gsl_matrix*X;
} state_t;


void kalman_compute_acc(gsl_matrix*mu_old, gsl_matrix*lambda_old, const double acc_x, const double acc_y, const double gps_x, const double gps_y,const double delay);
void kalman_compute_enc(gsl_matrix*mu_old, gsl_matrix*lambda_old, const double speed_x, const double speed_y, const double gps_x, const double gps_y,const double delay);
void kalman_reset(int time_step);

#endif