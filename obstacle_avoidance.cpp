#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream> // For std::cout for debugging

#define TIME_STEP 64
#define OBSTACLE_THRESHOLD 950.0
#define AVOID_OBSTACLE_DURATION 20
#define MAX_SPEED 1.0
#define TURN_SPEED 0.5

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_left", "ds_right"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0); // Start with the motors stopped
  }

  int avoidObstacleCounter = 0;

  while (robot->step(TIME_STEP) != -1) {
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      double leftSpeed = -TURN_SPEED;
      double rightSpeed = TURN_SPEED;
      wheels[0]->setVelocity(leftSpeed);
      wheels[1]->setVelocity(rightSpeed);
      wheels[2]->setVelocity(leftSpeed);
      wheels[3]->setVelocity(rightSpeed);
    } else {
      double dsValues[2];
      for (int i = 0; i < 2; i++) {
        dsValues[i] = ds[i]->getValue();
        std::cout << "Sensor " << i << " value: " << dsValues[i] << std::endl;
      }
      
      if (dsValues[0] < OBSTACLE_THRESHOLD || dsValues[1] < OBSTACLE_THRESHOLD) {
        avoidObstacleCounter = AVOID_OBSTACLE_DURATION;
      } else {
        // No obstacles detected, move forward
        wheels[0]->setVelocity(MAX_SPEED);
        wheels[1]->setVelocity(MAX_SPEED);
        wheels[2]->setVelocity(MAX_SPEED);
        wheels[3]->setVelocity(MAX_SPEED);
      }
    }
  }

  delete robot;
  return 0;  // EXIT_SUCCESS
}
