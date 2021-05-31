This folder contains the two libraries needed to compute positioning of the epuck robot. 

# makefile

In order to use the kalman.c and odometry.c libraries, the makefile has to be modified to include them.
At the C_SOURCES line of the makefile, add the name of the libraries.
For example, if you want to use odometry and kalman in the code controller.c, add this line :

C_SOURCES = localization_controller.c odometry.c kalman.c

# add to code

In the "add_to_controller" text file, you can find all the code that has to be in your controller code c file.
If some part is already included in the code, skip it.

the file is segmented in 4 parts

Libraries : all the libraries used for the localization

Declarations : all the variables, structures, functions used for localization. 
This needs to be declared before the main function

Main contents : schematic main function including the important parts for localization

All used functions : all the functions used to get measurements and compute them.

# usage

To compute position, the function "init_position" has to be called in the initialization process.
Note that this function takes a time_step as argument (see how it is define in the main() example).
The initial position is also provided using the 3 next arguments that are x, z (or y depending on the frame) and heading

To periodically compute position, the functon "compute_position" has to be called in the simulation loop.

It is possible to access different positioning variables :

_pose contains gps position
_odo_acc contains odometry using accelerometer positions
_odo_enc contains odometry using encoders positions
those variable are structures called pose_t (declared in the odometry header file).
variable.x gives you the x value of the position
variable.y gives you the y value
variable.heading gives you the theta

X_acc is a matrix 4x1 containing the x-position, y-position, x-speed, y-speed values computed using
the accelerometer based kalman filter.
X_enc is a matrix 2x1 containing the x-position, y-position values computed using
the encoder based kalman filter.
matrix data can be accessed using the gsl function gsl_matrix_get(matrixname,row,column)

