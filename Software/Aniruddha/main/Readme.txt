main.ino
This file initializes the other functions and sensors and then calls the scheduler.

main.h
This file is used to define all the macros and global variables. This also helps to define all the functions so that they can be accessed by any function in the code.

Timer.ino
Uses the internal timer to get an accuracy of 1 ms. Calls the functions according to required timing intervals.

IMU.ino
Gets data from the IMU, decodes it and converts to accelerations in three axes. One can choose the units by uncommenting the required data in the main.h header file.

PIDController.ino
Computes the error and the control signal using the calculated Kp, Ki and Kd values.

MotorController.ino 
Takes the control signals and gives out PWM signals as well as the direction to the motor drivers.
