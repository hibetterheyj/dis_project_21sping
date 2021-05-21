#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15

// Formation parameter
// Following value should be in Webots coordination!!!
#define target_x     0   // x of target position
#define target_z     0   // z of target position

static FILE *fp_flocking;
static FILE *fp_formation;

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;			// Single emitter

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
  
  emitter = wb_robot_get_device("emitter");
  if (emitter==0) printf("miss emitter\n");
  if (emitter!=0) printf("emitter works\n");
  
  super_name=(char*) wb_robot_get_name(); 
  if (super_name[0] == 'r'){
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
void compute_metric_flocking(){
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
      
      float delta_pos = sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][0],2));
      float c1 = delta_pos/RULE2_THRESHOLD;
      float c2 = 1/powf(1 - RULE2_THRESHOLD + delta_pos, 2);
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
  velocity /= max_dis;
  
  
  // Overall metric
  float metric = orientation*distance*velocity;
  
  if (VERBOSE_flocking_metric){
    printf("orientation metric is %g \n",orientation);
    printf("Denominator of distance metric is %g, Molecular of distance metric is %g \n",1 + d_1/FLOCK_SIZE,d_2/(FLOCK_SIZE*(FLOCK_SIZE-1)/2));
    printf("distance metric is %g \n",distance);
    printf("velocity metric is %g \n",velocity);
    printf("overall metric is %g \n",metric);
  }
  if( fp_flocking != NULL){
    float time_now_s = wb_robot_get_time();
    fprintf(fp_flocking, "%g; %g; %g; %g; %g\n",
      time_now_s, orientation, distance, velocity, metric);
  }
}

/*
 * Compute formation metrics
 */   
void compute_metric_formation(){
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
}
 
 
/*
 * Main function.
 */
int main(int argc, char *args[]) {
  supervisor_init();
  
  fp_flocking = fopen("flocking_metrics.csv","w");
  fprintf(fp_flocking, "time; orientation; distance; velocity towards goal direction; overall\n");
  
  fp_formation = fopen("formation_metrics.csv","w");
  fprintf(fp_formation, "time; distance; velocity towards goal direction; overall\n");
  
  for(;;) {
    update_loc();
    
    send_true();
    
    compute_metric_flocking();
    compute_metric_formation();
    
    wb_robot_step(TIME_STEP);
  }
  
  if(fp_flocking != NULL){
    fclose(fp_flocking);}
  if(fp_formation != NULL){
    fclose(fp_formation);}
  // End of the simulation
  wb_robot_cleanup();
  return 0;
}
