#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pso.h"

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define VERBOSE_flocking_metric false       // Print metrics of flocking
#define VERBOSE_formation_metric false       // Print metrics of formation

#define FLOCK_SIZE	5		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

//Robot speed parameters
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

// Flocking parameters
#define FLOCKING_DIST     0.14   // Flcoking pairwise robot distance

// Formation parameter
// Following value should be in Webots coordination!!!
#define target_x     0   // x of target position
#define target_z     0   // z of target position

//PSO parameters
// #define SWARMSIZE 12                    // Number of particles in swarm 
#define NB 2                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 40                       // Maximum velocity particle can attain
#define MININIT 0.0                  // Lower bound on initialization value
#define MAXINIT 80.0                    // Upper bound on initialization value
#define ITS 5                         // Number of iterations to run
// #define DATASIZE 6      // Number of elements in particle

#define ROBOTS 1
#define MAX_ROB 1
#define ROB_RAD 0.035
#define ARENA_SIZE 0.94

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 180                     // Number of fitness steps to run during optimization

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8

static FILE *fp_flocking;
static FILE *fp_formation;

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;			// Single emitter
WbDeviceTag emitter2;			// Single emitter2
WbDeviceTag receiver;		           // Single receiver 

double int_position[FLOCK_SIZE][3];
double int_head[3] = {0.0,0.0,-1.57};
float loc[FLOCK_SIZE][3];		// Location of everybody in the flock
float loc_prev[FLOCK_SIZE][3];		// Location of everybody in the flock at last time step
float max_dis = DELTA_T*(MAX_SPEED_WEB*MAX_SPEED*WHEEL_RADIUS/1000); //maximal distance possible per timestep
int super_id;  //Supervisor's ID
char* super_name;
/*
 * Initialize supervisor
 */
void supervisor_init() {
  wb_robot_init();
  
  receiver = wb_robot_get_device("receiver");
  if (receiver==0) printf("miss receiver\n");
  if (receiver!=0) printf("receiver works\n");
  wb_receiver_enable(receiver,TIME_STEP);
  
  emitter = wb_robot_get_device("emitter");
  if (emitter==0) printf("miss emitter\n");
  if (emitter!=0) printf("emitter works\n");
  
  emitter2 = wb_robot_get_device("emitter2");
  if (emitter2==0) printf("miss emitter2\n");
  if (emitter2!=0) printf("emitter2 works\n");
  
  
  super_name=(char*) wb_robot_get_name(); 
  printf("my name is %s \n",super_name);
  if (true){
    printf("obstacle scenario \n");
    super_id = 0;
  }
  else{
    printf("crossing scenario \n");
    sscanf(super_name,"super%d",&super_id);
    printf("my id is %d \n",super_id);
  }
  

  char rob[7] = "epuck0";
    int i;
    for (i=0;i<FLOCK_SIZE;i++) {
      sprintf(rob,"epuck%d",i+super_id*5);
      robs[i] = wb_supervisor_node_get_from_def(rob);
      robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
      robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
      loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
      loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
      loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
      int_position[i][0] = -2.9;
      int_position[i][1] = 0.0;
      switch(i){
		case 0:
			int_position[i][2] = 0.0;
			break;
		case 1:
			int_position[i][2] = 0.1;
			break;
		case 2:
			int_position[i][2] = -0.1;
			break;
		case 3:
			int_position[i][2] = 0.2;
			break;
		case 4:
			int_position[i][2] = -0.2;
			break;
	}
      }
}

/*
 * Update the true localization of robots
 */
void update_loc(){
  int i;
       
  for (i=0;i<FLOCK_SIZE;i++) {
    // Get data
    loc_prev[i][0] = loc[i][0]; // X_prev
    loc_prev[i][1] = loc[i][1]; // Z_prev
    loc_prev[i][2] = loc[i][2]; // THETA_prev
    // printf("robot %d is in %g %g \n",i,loc[i][0],loc[i][1]);
    
    loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
    loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
    loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
    // printf("robot %d is now in %g %g \n",i,loc[i][0],loc[i][1]);
    
    
  }
}

/*
 * Send true position of robots (For test only)
 */
void send_true(){
  char buffer[255];	// Buffer for sending data
  int i;
       
  for (i=0;i<FLOCK_SIZE;i++) {
    
    // Send it out
    sprintf(buffer,"%1d#%f#%f#%f",i+super_id*5,loc[i][0],loc[i][1],loc[i][2]);
    // printf("Robot %s \n",buffer);
    wb_emitter_send(emitter,buffer,strlen(buffer));
  }
}

/*
 * Compute flocking metrics
 */
float compute_metric_flocking(){
  int i; int j;
  float orientation = 0;
  float distance;
  float d_1 = 0; //Denominator of distance metric
  float d_2 = 0; //Molecular of distance metric
  float velocity;
  
  float H_diff;  //the difference of heading 
  
  float avg_loc[2] = {0,0}; //center of the flock
  float avg_loc_prev[2] = {0,0}; //center of the flock (previous)
  
  
  for (i=0;i<FLOCK_SIZE;i++) {
    avg_loc[0] += loc[i][0];
    avg_loc[1] += loc[i][1];
    avg_loc_prev[0] += loc_prev[i][0];
    avg_loc_prev[1] += loc_prev[i][1];
  }
  avg_loc[0] /= FLOCK_SIZE;
  avg_loc[1] /= FLOCK_SIZE;
  avg_loc_prev[0] /= FLOCK_SIZE;
  avg_loc_prev[1] /= FLOCK_SIZE;
  /* orientation between robots */
  for (i=0;i<FLOCK_SIZE;i++) {
    for (j=i+1;j<FLOCK_SIZE;j++) {
      H_diff = fabsf(loc[i][2]-loc[j][2]);
      orientation += H_diff > M_PI ? 2-H_diff/M_PI : H_diff/M_PI;
      
      float delta_pos = sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2));
      float c1 = delta_pos/FLOCKING_DIST;
      float c2 = 1/powf(1 - FLOCKING_DIST + delta_pos, 2);
      d_2 += c1 < c2 ? c1 : c2;
    }
    d_1 += sqrtf(powf(loc[i][0]-avg_loc[0],2)+powf(loc[i][1]-avg_loc[1],2));
  }
  
  // Orientation between robots
  orientation = 1 - orientation/(FLOCK_SIZE*(FLOCK_SIZE-1)/2);
  
  
  // Distance between robots
  distance = (d_2/(FLOCK_SIZE*(FLOCK_SIZE-1)/2))/(1 + d_1/FLOCK_SIZE);
  
  
  // Velocity of the team towards the goal direction
  velocity = sqrtf(powf(avg_loc[0]-avg_loc_prev[0],2)+powf(avg_loc[1]-avg_loc_prev[1],2));
  velocity = sqrtf(powf(avg_loc[0]-avg_loc_prev[0],2));
  velocity /= max_dis;
  
  
  // Overall metric
  float metric = orientation*distance*velocity;
  metric = velocity;
  
  if (VERBOSE_flocking_metric){
    printf("orientation metric is %g      ",orientation);
    // printf("Denominator of distance metric is %g, Numerator of distance metric is %g \n",1 + d_1/FLOCK_SIZE,d_2/(FLOCK_SIZE*(FLOCK_SIZE-1)/2));
    printf("distance metric is %g     ",distance);
    printf("velocity metric is %g \n",velocity);
    printf("overall metric is %g \n",metric);
  }
  if( fp_flocking != NULL){
    float time_now_s = wb_robot_get_time();
    fprintf(fp_flocking, "%g; %g; %g; %g; %g\n",
      time_now_s, orientation, distance, velocity, metric);
  }
  return metric;
}

/*
 * Compute formation metrics
 */   
float compute_metric_formation(){
  int i;
  float distance = 0;
  float velocity;
  
  float avg_loc[2] = {0,0}; //center of the flock
  float avg_loc_prev[2] = {0,0}; //center of the flock (previous)
  
  
  for (i=0;i<FLOCK_SIZE;i++) {
    avg_loc[0] += loc[i][0];
    avg_loc[1] += loc[i][1];
    avg_loc_prev[0] += loc_prev[i][0];
    avg_loc_prev[1] += loc_prev[i][1];
    distance += sqrtf(powf(loc[i][0]-target_x,2)+powf(loc[i][1]-target_z,2));
  }
  avg_loc[0] /= FLOCK_SIZE;
  avg_loc[1] /= FLOCK_SIZE;
  avg_loc_prev[0] /= FLOCK_SIZE;
  avg_loc_prev[1] /= FLOCK_SIZE;
  
  // Distance between robots and their target positions
  distance = 1/(1+distance/FLOCK_SIZE);
  
  
  // Velocity of the team towards the goal direction
  velocity = sqrtf(powf(avg_loc[0]-avg_loc_prev[0],2)+powf(avg_loc[1]-avg_loc_prev[1],2));
  velocity /= max_dis;
  
  
  // Overall metric
  float metric = distance*velocity;
  
  if (VERBOSE_formation_metric){
    printf("distance metric is %g \n",distance);
    printf("velocity metric is %g \n",velocity);
    printf("overall metric is %g \n",metric);
  }
  if( fp_formation != NULL){
    float time_now_s = wb_robot_get_time();
    fprintf(fp_formation, "%g; %g; %g; %g\n",
      time_now_s, distance, velocity, metric);
  }
  
  return metric;
}
 

void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
  float overall_metric=0.0;
  float final_metric=0.0;
  int step = 0;
  bool run = true;
  double buffer[255];
  // sprintf(buffer,"%1s","p");
  for (int iter=0;iter<2;iter++) {
    run = true;
    overall_metric = 0.0;
    for (int j=0;j<DATASIZE;j++) {
      buffer[j] = weights[0][j];
    }
    wb_emitter_send(emitter2,(void *)buffer,DATASIZE*sizeof(double));
    // printf("send\n");
    for(int i=0;i<FLOCK_SIZE;i++){
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), int_head);
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"), int_position[i]);
    }
    while(run){
    double ran = (double)rand()/RAND_MAX;
      // printf("%f\n",ran);
      update_loc();
      
      send_true();
      
      float metric = compute_metric_flocking();
      // float metric = compute_metric_formation();
      
      overall_metric += metric;
      while (wb_receiver_get_queue_length(receiver) > 0) {
        // printf("receive inf\n");
        char *inbuffer = (char*) wb_receiver_get_data(receiver);
        if (inbuffer[0] == 'p'){
          run = false;
          // printf("receive finish\n");
          // printf("receive finish\n");
        }
        wb_receiver_next_packet(receiver);
      }
      wb_robot_step(TIME_STEP);
      step ++;
      // printf("current step is: %d \n",step);
    }
    overall_metric /= step;
    final_metric += overall_metric;
  }
  // fit[0] = overall_metric;
  fit[0] = final_metric/2;
  // printf("perf is: %f \n",final_metric/2);
}

/*
 * Main function.
 */
int main(int argc, char *args[]) {
  double *weights;                         // Optimized result
  int i,k;                               // Counter variables
  double fit;                        // Fitness of the current FINALRUN
  double endfit;                     // Best fitness over 10 runs
  double w[MAX_ROB][DATASIZE];       // Weights to be send to robots (determined by pso() )
  double f[MAX_ROB];                 // Evaluated fitness (modified by calc_fitness() )
  double bestfit, bestw[DATASIZE];
  
  supervisor_init();
  
  fp_flocking = fopen("flocking_metrics.csv","w");
  fprintf(fp_flocking, "time; orientation; distance; velocity towards goal direction; overall\n");
  
  fp_formation = fopen("formation_metrics.csv","w");
  fprintf(fp_formation, "time; distance; velocity towards goal direction; overall\n");
  
  for(int p=0;p<2;p++){
    weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);
    
    // Set robot weights to optimization results
    printf("best para:");
    fit = 0.0;
    for (i=0;i<MAX_ROB;i++) {
      for (k=0;k<DATASIZE;k++){
	       w[i][k] = weights[k];
	       printf("%f,",weights[k]);}
    }
    printf("\n");
    // Run FINALRUN tests and calculate average
    for (i=0;i<FINALRUNS;i+=MAX_ROB) {
      int nh[SWARMSIZE][SWARMSIZE];
      fitness(w,f,nh);
      for (k=0;k<MAX_ROB && i+k<FINALRUNS;k++) {
      	fit += f[k];
      }
    }
    fit /= FINALRUNS;
    
    // Check for new best fitness
    if (fit > bestfit) {
      bestfit = fit;
      for (i = 0; i < DATASIZE; i++){
	       bestw[i] = weights[i];
      }
    }
    printf("Performance of the best solution: %.3f\n",fit);
    
    wb_robot_step(TIME_STEP*2);
  }
  
  if(fp_flocking != NULL){
    fclose(fp_flocking);}
  if(fp_formation != NULL){
    fclose(fp_formation);}
  // End of the simulation
  wb_robot_cleanup();
  return 0;
}
