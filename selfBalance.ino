/*
 * Attempt to create self-balancing robot. 
 * Author: Felix Li
 * References: 
 *  1. previous Adeept code AdeeptSelfBalancingRobotCode.ino 
 *  2. https://github.com/TKJElectronics/BalancingRobotArduino
 */

/*Include libraries */
#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <AngleKalmanFilter.h> //can define DEBUG_ANGLEKF macro in header file to enable debugging
#include <PIDController.h>

//---------------------------
/* Global Variables */ 
//---------------------------

//--For Debug--
//#define DEBUG_SETUP //macro to debug setup
//#define DEBUG_KALMAN //macro to debug inputs into Kalman
//#define DEBUG_MOTOR //macro to debug motor output

int iter = 0; //iteration debug
unsigned long prevMillis = 0; //for timer loop 

//--For MPU Process--
//Has onboard accelerometer and gyro sensor
MPU6050 mpu; //mpu instance
int16_t ax, ay, az; //accelerometer values 
int16_t gx, gy, gz; //gyro values
float GYRO_SENSITIVITY = 131.0f; //from spec sheet: https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/

//calibration
int CALIBRATION = 100; //how many calibration measurements
int32_t axBias, ayBias, azBias;
int32_t gxBias, gyBias, gzBias;

//--For Interrupt Routine--
int INTERRUPT_MS = 50; //How often interrupt routine should trigger
float INTERRUPT_S = INTERRUPT_MS/1000.f; //expected input is in seconds

//--For Kalman Filter--
//Constants taken from TKJ Electronics example of Balancing robot (using same MPU)
const float Q_ANGLE = 0.001f;
const float Q_BIAS = 0.003f;
const float R_ACCEL = 0.03f;
float pitchAngle = 0.0f; //filter output of state variable pitch
AngleKalmanFilter kf;

//--PID Controller for pitch--
const float KP_PITCH = 14.5f; 
const float KI_PITCH = 0.31f; 
const float KD_PITCH = 0.00f; 
const float CONSTRAINT_PITCH = 3500;
const float REF_PITCH = 0.0f; //reference point
const float DEADZONE_BIAS = 10;
int motorRightBias = 10; //additional bias for right motor due to discrepancy of deadzone
PIDController pidPitch = PIDController(REF_PITCH, KP_PITCH, KD_PITCH, KI_PITCH, CONSTRAINT_PITCH, DEADZONE_BIAS); 


//--For Motor Control--
//TB6612FNG Drive module control signal
//Motor A = Left Motor, Motor B = Right Motor
#define AIN1 7
#define AIN2 6
#define BIN1 13
#define BIN2 12
#define PWMA 9
#define PWMB 10
#define STBY 8
const int FALLING_PITCH = 60; 
const int MAX_PWM = 255;

//--For RGB PWM--
const int RPin = A0; //RGB LED Pin R
const int GPin = A1; //RGB LED Pin G
const int BPin = A2; //RGB LED Pin B

//--For Buzzer--
const int buzzerPin = 11;  // define pin for buzzer

//--Mode State--
const int STOP = 0;
const int RUN = 1;
int mode = STOP;

//-------------------------------------
/** Main Functions **/ 
//-------------------------------------


/**
 * Set up all components correctly. 
*/ 
void setup() {

  Serial.begin(9600);

  //Test to see if connection to MPU is good. If so, initialize
  if (mpu.testConnection()) {
    mpu.initialize();
    delay(10); //delay 10 ms for stability
    mpu.resetSensors();
    delay(10);
    //get bias values 
    calibrateSensor();

    //set the initial angle of Kalman Filter, along with covariance values 
    float initialAngle = getAngleDegree(ayBias, azBias);
    #ifdef DEBUG_SETUP
    Serial.print("Initial Angle:"); Serial.println(initialAngle);
    #endif
    
    kf = AngleKalmanFilter(initialAngle, Q_ANGLE, Q_BIAS, R_ACCEL);

    motorInitialize();

    //Success = one long buzz
    buzz(1000);   
  } else {
    Serial.print("Error! Could not successfully communicate to MPU");

    //error is two short buzzes
    buzz(500);
    buzz(500); 
  }

  //Enable interrupts
  MsTimer2::set(INTERRUPT_MS, interrupt);
//  if (mode == RUN) MsTimer2::start();  
}

/**
 *  Calibrate the MPU6050 sensor. Figure out the bias values.
 *  Iterates CALIBRATION times.
 */
void calibrateSensor() 
{
  axBias, ayBias, azBias = 0;
  gxBias, gyBias, gzBias = 0;
  for (int i = 0; i < CALIBRATION; i++) { 
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      axBias += ax;
      ayBias += ay;
      azBias += az; 
      gxBias += gx; 
      gyBias += gy;
      gzBias += gz;
      delay(10);
  }

  axBias /= CALIBRATION;
  ayBias /= CALIBRATION;
  azBias /= CALIBRATION;
  gxBias /= CALIBRATION;
  gyBias /= CALIBRATION; 
  gzBias /= CALIBRATION; 

  #ifdef DEBUG_SETUP
  Serial.println("Calibration done. Found:"); 
  Serial.print("ax - "); Serial.println(axBias); 
  Serial.print("ay - "); Serial.println(ayBias); 
  Serial.print("az - "); Serial.println(azBias); 
  Serial.print("gx - "); Serial.println(gxBias); 
  Serial.print("gy - "); Serial.println(gyBias); 
  Serial.print("gz - "); Serial.println(gzBias); 
  #endif 
}

/**
 * Initialize the motor. 
 */
void motorInitialize()
{
  // TB6612FNGN drive module control signal initialization 
  //(from Adeept)
  pinMode(AIN1, OUTPUT);//Control the direction of the motor 1, 01 for the forward rotation, 10 for the reverse
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);//Control the direction of the motor 2, 01 for the forward rotation, 10 for the reverse
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);//Left motor PWM
  pinMode(PWMB, OUTPUT);//Right motor PWM
  pinMode(STBY, OUTPUT);//TB6612FNG enabled

  //both motors in forward rotation. STBY must be 1 for motors to be active
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
/**
 * Interrupt service routine that runs at every INTERRUPT_MS 
*/ 
void interrupt()
{
  sei(); //re-enable interrupts
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //get sensor data
  runKalman(INTERRUPT_S); //run kalman to get pitch

  float motorPwmA = pidPitch.update(pitchAngle, INTERRUPT_S) * -1; //if falling forward, move forward. If falling back, move back. Hence, opposite of PID output.
  float motorPwmB = motorPwmA; 

  //Killing the motor if falling to prevent significant damage to the robot 
  if (pitchAngle > FALLING_PITCH or pitchAngle <-FALLING_PITCH) stopRobot();
  else {
    //issue is right motor is stiffer. hence needs extra boost.
    if (motorPwmB > 0) motorPwmB += motorRightBias;
    if (motorPwmB < 0) motorPwmB -= motorRightBias;
  
    //want motor control as lean as possible so there's little delay in between triggering the motors.
    motorPwmA = motorControl(AIN1,AIN2, motorPwmA);
    motorPwmB = motorControl(BIN1,BIN2, motorPwmB);    
    
    motorOutput(PWMA, motorPwmA);
    motorOutput(PWMB, motorPwmB);    
  }
}

/**
 * Run Kalman Filter.
 * 
 *  @param: dt. How much time has passed since last iteration.
 *  @return: None. *pitchAngle is global variable*
*/ 
void runKalman(float dt)
{

  //calculate accelerometer angle 
  float measurementPitch = getAngleDegree(ay, az);
  float rateGyro = getGyro(gx, gxBias); //in this case, CW on x-axis = positive (falling forward), CCW = negative (falling backward)
  #ifdef DEBUG_KALMAN
  Serial.println("-----");   
  Serial.print("Iteration "); Serial.println(iter); 
  Serial.print("Measurement: "); Serial.print(measurementPitch); Serial.print("; rateGyro: "); Serial.print(rateGyro); Serial.print("; dt: "); Serial.println(dt, 4);
  Serial.print("Pitch:"); Serial.println(pitchAngle);
  #endif

  pitchAngle = kf.iterate(dt, rateGyro, measurementPitch);

  #ifdef DEBUG_KALMAN
  Serial.print("Filtered Pitch:"); Serial.println(pitchAngle);
  iter += 1;
  #endif
}

/**
 * Motor Control for one of the motors. 
 * 
 * @param: controlPin0. Pin ID for one of the controls. 
 * @param: controlPin1. Pin ID for one of the controls. 
 * @param: pwm. What pwm output to provide. 
 * @return: abs(pwm)
 * 
*/

float motorControl(int controlPin0, int controlPin1, float pwm)
{
  //Cap pwm 
  if (pwm > MAX_PWM) pwm = MAX_PWM; 
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;

  //Determining motor direction based on sign of PWM
  if (pwm >= 0) { //forward rotation (CW)
    digitalWrite(controlPin0, 0);
    digitalWrite(controlPin1, 1);
    return pwm;
  } else { //backward rotation (CCW)
    digitalWrite(controlPin0, 1);
    digitalWrite(controlPin1, 0);
    return -pwm;
  }
} 

/**
 * Writing the PWM to motor. Separated out with motor control to make outputs to both motors as close as possible.
 * 
 * @param: pwmPin. Pin ID for PWM. 
 * @param: pwm. What pwm output to provide. 
 * 
*/
void motorOutput(int pwmPin, float pwm)
{
  #ifdef DEBUG_MOTOR
  Serial.print("Motor Out "); Serial.print(pwmPin); Serial.print(":"); Serial.println(pwm);
  #endif
  analogWrite(pwmPin, pwm);
}

/**
 * Main loop of the code.
*/
void loop() {
  //if serial is available / bluetooth
  if(Serial.available() > 0){  
       switch(Serial.read()){
          //Stop the robot  
          case 's': {
            stopRobot();
            break;
          }

          //Allow it to run
          case 'r': {
            mode = RUN; 
            MsTimer2::start();              
            break;
          }

          //Update Kp 
          case 'p': {
            float kp = getSerialFloat();
            pidPitch.setKp(kp);
            break;
          }

          //Update Ki 
          case 'i': {
            float ki = getSerialFloat();
            pidPitch.setKi(ki);
            break;
          }

          //Update Kd
          case 'd': {
            float kd = getSerialFloat();
            pidPitch.setKd(kd);
            break;
          }

          //update biases on wheels 
          case 'b': {
            float pidBias = getSerialFloat();
            motorRightBias = getSerialInt();
            pidPitch.setBias(pidBias);            
            break;
          }

          //Run both motors with given m
          case 'm': {
            int pwm = getSerialInt();
            motorOutput(PWMA, pwm);
            motorOutput(PWMB, pwm);
            break;
          }
          
           //Print out PID and other parameters.
          case 'h': {
            pidPitch.printMe();
            Serial.print("motorRightBias: "); Serial.println(motorRightBias);
            break;
          }
       
       } //end switch 
  }

  //To Do: Different modes and their effects to be coded later
}

/** Stops the robot. */
void stopRobot()
{
  mode = STOP;
  MsTimer2::stop(); 
  motorOutput(PWMA, 0);
  motorOutput(PWMB, 0);   
}

//-------------------------------------
/** Helper Functions **/ 
//-------------------------------------

/**
 * Buzzes for designated name. 
 * 
 *  @param: buzzTime. How long to buzz in ms.
 *  @return: None 
 */
void buzz(int buzzTime) {
//    digitalWrite(buzzerPin, HIGH); 
    delay(buzzTime);
    digitalWrite(buzzerPin, LOW);  
}


/**
 *  Returns angle between two vector magnitudes.
 *  
 *  @param v0. First value direction.
 *  @param v1. Second value direction.
 *  @return Calculated angle in degrees from -180 to 180.
*/
float getAngleDegree(float v0, float v1) {
  return atan2(v0, v1) * RAD_TO_DEG;
}

/**
 *  Get Gyro value by subtracting away resting initial bias and divide by sensitivity.
 *  
 *  @param value. Gyro reading from sensor.
 *  @param bias. Initial gyro bias when it was at rest.
 *  @return Gyro rate of this rotation in degree/s.
*/
float getGyro(int16_t value, int32_t bias) {
  return (value-bias)/GYRO_SENSITIVITY;
}

/**
 * Helper function to retrieve next float from Serial communication. 
 */
float getSerialFloat()
{
  while(!Serial.available()) {
    continue;
  }

  float floatSerial = Serial.parseFloat();
  return floatSerial;

}

/**
 * Helper function to retrieve next int from Serial communication. 
 */
float getSerialInt()
{
  while(!Serial.available()) {
    continue;
  }

  int intSerial = Serial.parseInt();
  return intSerial;

}


//-------------------------------------
/** Misc/Test Functions **/ 
//-------------------------------------
/** Test loop to see KF and PID behavior 
 *  
*/
//void loop()
//{
//
////  //if serial is available / bluetooth
//  if(Serial.available() > 0){  
//       switch(Serial.read()){
//          //Stop the robot  
//          case 's': {
//            mode = STOP; 
//            stopRobot();
//            break;
//          }
//
//          case 'r': {
//            mode = RUN;
//            break;
//          }
//       } //end switch
//  }
//
//  if (mode == RUN) {
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    float dt = (millis() - prevMillis)/1000.0f;
//    prevMillis = millis();
//    runKalman(dt);
//    float motorPwmA = pidPitch.update(pitchAngle, INTERRUPT_S) * -1; //if falling forward, move forward. If falling back, move back. Hence, opposite of PID output.
//    float motorPwmB = motorPwmA; 
//  
//    //Killing the motor if falling to prevent significant damage to the robot 
//    if (pitchAngle > FALLING_PITCH or pitchAngle <-FALLING_PITCH) stopRobot();
//    else {
//      //issue is right motor is stiffer. hence needs extra boost.
//      if (motorPwmB > 0) motorPwmB += motorRightBias;
//      if (motorPwmB < 0) motorPwmB -= motorRightBias;
//    
//      //want motor control as lean as possible so there's little delay in between triggering the motors.
//      motorPwmA = motorControl(AIN1,AIN2, motorPwmA);
//      motorPwmB = motorControl(BIN1,BIN2, motorPwmB);    
//      
////      motorOutput(PWMA, motorPwmA);
////      motorOutput(PWMB, motorPwmB);    
//    }
//    Serial.print("PID: "); Serial.print(motorPwmA); Serial.print(" "); Serial.println(motorPwmB);    
//  }
//}

/** Test loop 2 to control motor incrementally and see effects of PWM.
 *  
*/

//Use below variables to increment
//float motorPwmA = 0;
//float motorPwmB = 0;
//float setPWM = 0;
//void loop()
//{
//
////  //if serial is available / bluetooth
//  if(Serial.available() > 0){  
//       switch(Serial.read()){
//          //Stop the robot  
//          case 's': {
//            mode = STOP;
//            stopRobot();
//            break;
//          }
//
//          case 'r': {
//            mode = RUN;
//            //rerun setup
//            setup(); 
//            motorPwmA = setPWM;
//            motorPwmB = setPWM;
//            break;
//          }
//
//          //change the start pwm
//          case 'p': {
//            setPWM = getSerialFloat();
//          }
//       } //end switch
//  }
//
//  if (mode == RUN) {
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    float dt = (millis() - prevMillis)/1000.0f;
//    prevMillis = millis();
//    runKalman(dt);
//
//    if (motorPwmA >= MAX_PWM) motorPwmA = MAX_PWM; 
//    if (motorPwmB >= MAX_PWM) motorPwmB = MAX_PWM;
//    
//    motorOutput(PWMA, motorPwmA);
//    motorOutput(PWMB, motorPwmB);    
//
//    //kill the motor once the pitch becomes negative
//    if (pitchAngle > 55 or pitchAngle <-55) stopRobot();
//    Serial.print("PID: "); Serial.print(motorPwmA); Serial.print(" "); Serial.println(motorPwmB);    
//  }
//}

