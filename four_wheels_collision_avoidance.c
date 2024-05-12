// Standard libraries for Webots functionality and input/output operations
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h> 

// Macro definitions for constants used throughout the code
#define TIME_STEP 64 // Simulation time step (milliseconds)
#define OBSTACLE_THRESHOLD 950.0 // Distance value below which an obstacle is considered detected
#define AVOID_OBSTACLE_DURATION 20 // Duration in time steps to continue avoidance maneuvers
#define MAX_SPEED 10.0 // Maximum forward speed for the robot's wheels
#define TURN_SPEED 2.0 // Speed used for turning maneuvers when avoiding obstacles

// Main program
int main(int argc, char **argv) {
  wb_robot_init(); // Initialize the robot's parameters and get it ready to work in Webots.

  int i; // a variable used to repeat a loop

  int avoid_obstacle_counter = 0; // Counter to control how long the obstacle avoidance behaviour lasts

  // Sensor setup
  WbDeviceTag ds[2]; // Accessible tags for the distance sensors
  char ds_names[2][10] = {"ds_left", "ds_right"}; // The distance sensor names used in the realm of Webots
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]); // Obtain the sensor devices using their label names.
    wb_distance_sensor_enable(ds[i], TIME_STEP); // Switch on the sensors at the predetermined time interval.
    if (ds[i] == 0) {
      printf("Error: Unable to find distance sensor %s\n", ds_names[i]); // Inspect and report any missing sensors.
    }
  }

  // Motor setup
  WbDeviceTag wheels[4]; // Access tags for the motors
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"}; // The motor names used in the world of Webots
  double actual_max_speed[4]; // Array to store the motors' actual maximum speeds
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]); // Get the motor devices using their names.
    if (wheels[i] == 0) {
      printf("Error: Unable to find motor %s\n", wheels_names[i]); // Inspect and report any missing motors.
    } else {
      wb_motor_set_position(wheels[i], INFINITY); // Rotate the motors indefinitely.
      actual_max_speed[i] = wb_motor_get_max_velocity(wheels[i]); // Obtain and save each motor's maximum velocity.
      wb_motor_set_velocity(wheels[i], 0); // begin with the motors off
      printf("Max speed for %s is %f\n", wheels_names[i], actual_max_speed[i]); // Set the diagnostics output speed to its maximum.
    }
  }

  // Main simulation loop
  while (wb_robot_step(TIME_STEP) != -1) { // Proceed with the simulation's phases until it is finished.
    double ds_values[2]; // Array to store sensor readings
    for (i = 0; i < 2; i++) {
      ds_values[i] = wb_distance_sensor_get_value(ds[i]); // Obtain the sensor's current values.
    }
    printf("Left sensor: %f, Right sensor: %f\n", ds_values[0], ds_values[1]); // Sensor readings output for debugging

    // Obstacle avoidance logic
    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--; // Decrement the counter
      double left_speed = -TURN_SPEED; // Set default turning speeds
      double right_speed = TURN_SPEED;
      if (ds_values[0] > ds_values[1]) { // Based on sensor inputs, change the turning direction

        left_speed = TURN_SPEED;
        right_speed = -TURN_SPEED;
      }
      for (i = 0; i < 4; i++) {
        if (wheels[i] != 0) {
          wb_motor_set_velocity(wheels[i], i % 2 == 0 ? left_speed : right_speed); // Set the wheels' turning speeds.
        }
      }
    } else if (ds_values[0] < OBSTACLE_THRESHOLD || ds_values[1] < OBSTACLE_THRESHOLD) {
      avoid_obstacle_counter = AVOID_OBSTACLE_DURATION; // If an obstacle is found, reset the counter.
    } else {
      for (i = 0; i < 4; i++) {
        if (wheels[i] != 0) {
          wb_motor_set_velocity(wheels[i], actual_max_speed[i] > MAX_SPEED ? MAX_SPEED : actual_max_speed[i]); // Adjust forward speed to the desired speed or the maximum permitted by the motor.
        }
      }
    }
  }

  wb_robot_cleanup(); // Closing the programme and clearing up the resources
  return 0; // Successful completion of the program
}
