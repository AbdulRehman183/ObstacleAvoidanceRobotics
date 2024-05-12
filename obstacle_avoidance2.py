from controller import Robot, Motor, DistanceSensor

TIME_STEP = 64
OBSTACLE_THRESHOLD = 950.0
AVOID_OBSTACLE_DURATION = 20
MAX_SPEED = 1.0
TURN_SPEED = 0.5

def main():
    robot = Robot()
    
    ds = []
    ds_names = ["ds_left", "ds_right"]
    for name in ds_names:
        sensor = robot.getDistanceSensor(name)
        sensor.enable(TIME_STEP)
        ds.append(sensor)
    
    wheels = []
    wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
    for name in wheels_names:
        motor = robot.getMotor(name)
        motor.setPosition(float('inf'))  # Infinite position for continuous rotation
        motor.setVelocity(0)  # Start with the motors stopped
        wheels.append(motor)
    
    avoid_obstacle_counter = 0
    
    while robot.step(TIME_STEP) != -1:
        if avoid_obstacle_counter > 0:
            avoid_obstacle_counter -= 1
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED
            wheels[0].setVelocity(left_speed)
            wheels[1].setVelocity(right_speed)
            wheels[2].setVelocity(left_speed)
            wheels[3].setVelocity(right_speed)
        else:
            ds_values = [sensor.getValue() for sensor in ds]
            print(f"Sensor values: {ds_values}")  # Debug output
            if ds_values[0] < OBSTACLE_THRESHOLD or ds_values[1] < OBSTACLE_THRESHOLD:
                avoid_obstacle_counter = AVOID_OBSTACLE_DURATION
            else:
                # No obstacles detected, move forward
                for wheel in wheels:
                    wheel.setVelocity(MAX_SPEED)

if __name__ == "__main__":
    main()
