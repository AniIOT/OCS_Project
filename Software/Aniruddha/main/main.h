// import libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* Preprocessor directives */
// define magic numbers for MISRA compliance
#define NUM_ZERO       0U
#define MAX_DEG_ERROR  180U
#define MAX_PWM_VAL    255U

// define IMU reader object type
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL 

// define IMU pins
#define INTERRUPT_PIN  2U
#define LED_PIN        13U
#define MPU_ADDR       0x68; // I2C address of the MPU-6050

// define motor control pins
#define PWM_MOTOR_ONE  3U   // motor 1 speed control pin
#define PWM_MOTOR_TWO  4U   // motor 2 speed control pin
#define DIR_MOTOR_ONE  7U   // motor 1 direction control pin
#define DIR_MOTOR_TWO  8U   // motor 2 direction control pin

/* Global Variables */
// MPU control/status vars
MPU6050 mpu;                // Initialize IMU object
bool dmpReady = false;      // set true if DMP init was successful
uint8_t mpuIntStatus;       // holds actual interrupt status byte from MPU
uint8_t devStatus;          // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];     // FIFO storage buffer

// MPU orientation/motion vars
Quaternion q;              // [w, x, y, z]         quaternion container
VectorInt16 aa;            // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;        // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;       // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;       // [x, y, z]            gravity vector
float euler[3];            // [psi, theta, phi]    Euler angle container
float ypr[3];              // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID Controller vars
double sensed_output, control_signal;
double setpoint;
double Kp = -40; //proportional gain
double Ki = 0; //integral gain
double Kd = -8; //derivative gain
int T; //sample time in milliseconds (ms)
unsigned long last_time = millis();
double total_error, last_error;
int max_control;
int min_control;

// Motor Controller vars
uint8_t u8mappedMotorOnePWM = 0;
uint8_t u8mappedMotorTwoPWM = 0;
bool b8MotorOneDirection = false;
bool b8MotorTwoDirection = false;

// Scheduler vars
unsigned long prevMillis50mstask = 0;
unsigned long prevMillis100mstask = 0;
unsigned long prevMillis1000mstask = 0;
unsigned long currMillis = 0;
unsigned long prevMicros50ustask = 0;
unsigned long prevMicros100ustask = 0;
unsigned long prevMicros1000ustask = 0;
unsigned long currMicros = 0;

/* Function Definitions */
void TimedFunctionCaller_Init();
void TimedFunctionCaller();
void IMU_Init();
void IMUGetData();
void PID_Control();
void selfBalance();
