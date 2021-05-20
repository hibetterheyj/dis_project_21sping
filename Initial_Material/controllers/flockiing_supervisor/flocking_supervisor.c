#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define VERBOSE_err false       // Print accumulated error(required metric)
#define VERBOSE_avg_err false       // Print average error

#define FLOCK_SIZE	5		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;			// Single emitter

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

/*
 * Initialize supervisor
 */
void supervisor_init() {
  wb_robot_init();
  
  emitter = wb_robot_get_device("emitter");
  if (emitter==0) printf("miss emitter\n");
  if (emitter!=0) printf("emitter works\n");
  
  
  char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}

  

}

/*
 * Receive estimator information
 */
void send_true(){
  char buffer[255];	// Buffer for sending data
  int i;
       
  for (i=0;i<FLOCK_SIZE;i++) {
    // Get data
    loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
    loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
    loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
    
    // Send it out
    sprintf(buffer,"%1d#%f#%f#%f",i,loc[i][0],loc[i][1],loc[i][2]);
    // printf("Robot %s \n",buffer);
    wb_emitter_send(emitter,buffer,strlen(buffer));
  }
}


/*
 * Main function.
 */
int main(int argc, char *args[]) {
  supervisor_init();
  for(;;) {
    send_true();
    wb_robot_step(TIME_STEP);
  }
  
  return 0;
}
