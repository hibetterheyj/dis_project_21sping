#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define VERBOSE_err false       // Print accumulated error(required metric)
#define VERBOSE_avg_err false       // Print average error

WbNodeRef rob_node; // Robot nodes	
WbFieldRef rob_trans; // Robots translation fields
WbDeviceTag receiver;	// Handle for the receiver node

float pos_origin[3] = {-2.9, 0, 0};
float pos_true[3];
float pos_est[8] ; //0-1 is pos by gps, 2-3 is by acc, 4-5 is by encoder
float err_loc[4] = {0,0,0}; //metrics for gps, acc and encoder
float avg_err_loc[4] = {0,0,0}; //average localization error
float pos_true_prev[2] = {-2.9, 0}; //previous position of rob (the estimate we receive is one step later)
static FILE *fp;

/*
 * Initialize supervisor
 */
void supervisor_init() {
  wb_robot_init();
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver,16);
  
  rob_node = wb_supervisor_node_get_from_def("ROBOT1");
  rob_trans = wb_supervisor_node_get_field(rob_node, "translation");
  

}

/*
 * Receive estimator information
 */
void receive_inf(){
  if(wb_receiver_get_queue_length(receiver)>0){
    const char *message = wb_receiver_get_data(receiver);
    float *p = (float* )message;
    int i;
    for (i=0;i<8;i++){
      pos_est[i] = p[i];
    }
    wb_receiver_next_packet(receiver);
  }
}

/*
 * Main function.
 */
int main(int argc, char *args[]) {
  supervisor_init();
  
  fp = fopen("errors.csv","w");
  fprintf(fp, "time; err_gps; err_acc; err_enc; err_kf; err_gps_avg; err_acc_avg; err_enc_avg; err_kf_avg\n");
  
  int time_step = wb_robot_get_basic_time_step();
  while (wb_robot_step(time_step) != -1)  {
    // Receive the estimated position
    receive_inf();
    // Add offset
    pos_est[2]+=pos_origin[0];
    pos_est[4]+=pos_origin[0];
    pos_est[6]+=pos_origin[0];

    float time_now_s = wb_robot_get_time();
    if(time_now_s>0){
      err_loc[0] += sqrt(pow(pos_est[0]-pos_true_prev[0],2)+pow(pos_est[1]-pos_true_prev[1],2));
      err_loc[1] += sqrt(pow(pos_est[2]-pos_true_prev[0],2)+pow(pos_est[3]+pos_true_prev[1],2));
      err_loc[2] += sqrt(pow(pos_est[4]-pos_true_prev[0],2)+pow(pos_est[5]+pos_true_prev[1],2));
      err_loc[3] += sqrt(pow(pos_est[6]-pos_true_prev[0],2)+pow(pos_est[7]+pos_true_prev[1],2));
      
      if (time_now_s < 1.01){
        err_loc[0] -= sqrt(pow(pos_est[0]-pos_true_prev[0],2)+pow(pos_est[1]-pos_true_prev[1],2));
      }
      // printf("sup time: %g \n",time_now_s);
      // printf("time step %g \n",time_now_s/0.016);

      avg_err_loc[0] = err_loc[0]/(time_now_s/0.016);
      avg_err_loc[1] = err_loc[1]/(time_now_s/0.016);
      avg_err_loc[2] = err_loc[2]/(time_now_s/0.016);
      avg_err_loc[3] = err_loc[3]/(time_now_s/0.016);
      
      if( fp != NULL){
        fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            time_now_s, err_loc[0],err_loc[1],err_loc[2],err_loc[3],avg_err_loc[0],avg_err_loc[1],avg_err_loc[2],avg_err_loc[3]);
      }
    }
    
    if (VERBOSE_err){
      printf("err_gps %g \n",err_loc[0]);
      printf("err_acc %g \n",err_loc[1]);
      printf("err_enc %g \n",err_loc[2]);
      printf("err_kf %g \n",err_loc[3]);
    }
    if (VERBOSE_avg_err){
      printf("err_gps_avg %g \n",avg_err_loc[0]);
      printf("err_acc_avg %g \n",avg_err_loc[1]);
      printf("err_enc_avg %g \n",avg_err_loc[2]);
      printf("err_kl_avg %g \n",avg_err_loc[3]);
    }
    
    // printf("est_gps %g %g \n",pos_est[0],pos_est[1]);
    // printf("est_acc %g %g \n",pos_est[2],pos_est[3]);
    // printf("est_en %g %g \n",pos_est[4],pos_est[5]);
    // printf("est_kf %g %g \n",pos_est[6],pos_est[7]);
    
    // printf("true_prev %g %g \n",pos_true_prev[0],pos_true_prev[1]);
        
    // const double *pos_true = wb_supervisor_field_get_sf_vec3f(rob_trans);
    pos_true[0] = wb_supervisor_field_get_sf_vec3f(rob_trans)[0];
    pos_true[2] = wb_supervisor_field_get_sf_vec3f(rob_trans)[2];
    // printf("distance %g %g \n",pos_true_prev[0]-pos_true[0],pos_true_prev[1]-pos_true[2]);
    pos_true_prev[0] = pos_true[0];
    pos_true_prev[1] = pos_true[2];
    // printf("true %g %g \n",pos_true[0],pos_true[2]);
  }
  if(fp != NULL){
    fclose(fp);}
    // End of the simulation
  wb_robot_cleanup();
  return 0;
}
