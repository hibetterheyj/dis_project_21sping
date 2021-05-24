/*****************************************************************************/
/* File:  formation_super.c
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define ROBOTS 5

static WbNodeRef robs[ROBOTS];
static WbFieldRef robs_translation[ROBOTS];
static WbFieldRef robs_rotation[ROBOTS];
WbDeviceTag emitter_device;

double loc[ROBOTS][4];

/* Good relative positions for each robot */
double good_rp[ROBOTS][2] = { {0.0,0.0}, {0.0,0.1}, {0.0,-0.1}, {0.0,0.2}, {0.0,-0.2}};
// double good_rp[ROBOTS][2] = { {0.0,0.0}, {0.0,0.30}, {0,-0.30}, {0.0,0.60} };


void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck1";
	char emitter0[8] = "emitter";
	int i;
	robs[0] = wb_supervisor_node_get_from_def(rob);
	robs_translation[0] = wb_supervisor_node_get_field(robs[0],"translation");
	robs_rotation[0] = wb_supervisor_node_get_field(robs[0],"rotation");

	rob[5]++;
	for (i=1;i<ROBOTS;i++) {
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");    
		rob[5]++;
	}
	emitter_device = wb_robot_get_device(emitter0);
}

int main(int argc, char *args[]) {
	float buffer[255];
	float global_x,global_z,rel_x,rel_z;
	double temp_err, err, avg_err;
	int cnt,i;
	int print_enabled = 0;
	int send_interval = 5;

	if (argc > 1) {
		print_enabled = atoi(args[1]);
		printf("Print: %d\n", print_enabled);
	}
	if (argc > 2) {
		send_interval = atoi(args[2]);
		if (send_interval < 1) send_interval = 1;
		if (send_interval > 1000) send_interval = 1000;
		printf("Sending at intervals of %d\n", send_interval);
	}


	reset();

	avg_err = 0.0;
	for(cnt = 0; ; cnt++) { /* The robot never dies! */

		/* Send relative positions to followers */
		err = 0.0;
		for (i=1;i<ROBOTS;i++) {
			/* Get data */
			loc[0][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[0];
			loc[0][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[1];
			loc[0][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[2];
			loc[0][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3];
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[1];
			loc[i][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
			loc[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];

			/* Find global relative coordinates */
			global_x = loc[i][0] - loc[0][0];
			global_z = loc[i][2] - loc[0][2];
			/* Calculate relative coordinates */
			rel_x = -global_x*cos(loc[i][3]) + global_z*sin(loc[i][3]);
			rel_z = global_x*sin(loc[i][3]) + global_z*cos(loc[i][3]);
			buffer[0]= i;
			buffer[1] = rel_x;
			buffer[2] = rel_z;
			buffer[3] = loc[0][3] - loc[i][3];
			while (buffer[2] > M_PI) buffer[2] -= 2.0*M_PI;
			while (buffer[2] < -M_PI) buffer[2] += 2.0*M_PI;
			if (cnt % send_interval == 0){
				wb_emitter_send(emitter_device,(char *)buffer,4*sizeof(float));        
			}

			/* Check error in position of robot */
			rel_x = global_x*cos(loc[0][3]) - global_z*sin(loc[0][3]);
			rel_z = -global_x*sin(loc[0][3]) - global_z*cos(loc[0][3]);
			temp_err = sqrt(pow(rel_x-good_rp[i][0],2) + pow(rel_z-good_rp[i][1],2));
			if (print_enabled)
				printf("Err %d: %.3f, ",i,temp_err);
			err += temp_err/ROBOTS;
		}
		if (print_enabled)
			printf("\n");
		err = exp(-err/0.2);
		avg_err += err;
		if (print_enabled)
			printf("Performance: %.2f, Average Performance: %.2f\n",err,avg_err/cnt);

		wb_robot_step(64); /* run one step */
	}
	return 0;
}
