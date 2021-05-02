#include <webots/robot.h>
#include <webots/motor.h>

void trajectory_1(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor) {
// ## DO NOT MODIFY THIS
   double t = wb_robot_get_time();
   if (t > 0.0 && t < 3.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 3.0 && t < 5.0) { // right turn
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 4.0);
   }
   
   else if (t >= 5.0 && t < 14.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 14.0 && t < 16.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 16.0 && t < 50.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 50.0 && t < 52.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 52.0 && t < 70.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 70.0 && t < 71.9) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 72.0 && t < 105.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 105.0 && t < 106.9) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 107.0 && t < 115.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
    
   else {
     wb_motor_set_velocity(dev_left_motor, 0.0);
     wb_motor_set_velocity(dev_right_motor, 0.0);
   }
}

void trajectory_2(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor) {
// ## DO NOT MODIFY THIS
  double t = wb_robot_get_time();
   if (t > 0.0 && t < 3.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 3.0 && t < 5.0) { // right turn
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 4.0);
   }
   
   else if (t >= 5.0 && t < 14.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 14.0 && t < 16.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 16.0 && t < 25.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 25.0 && t < 27.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 27.0 && t < 45.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 45.0 && t < 47.0) { // right turn
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 4.0);
   }
   
   else if (t >= 47.0 && t < 56.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 56.0 && t < 58.0) { // right turn
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 4.0);
   }
   
   else if (t >= 58.0 && t < 76.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 76.0 && t < 78.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 78.0 && t < 87.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 87.0 && t < 89.0) { // left turn
     wb_motor_set_velocity(dev_left_motor, 4.0);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }
   
   else if (t >= 89.0 && t < 107.0) {
     wb_motor_set_velocity(dev_left_motor, 6.28);
     wb_motor_set_velocity(dev_right_motor, 6.28);
   }

   else {
     wb_motor_set_velocity(dev_left_motor, 0.0);
     wb_motor_set_velocity(dev_right_motor, 0.0);
   }
}