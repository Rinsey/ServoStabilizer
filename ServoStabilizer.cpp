
/*	
	Librería PID elaborada por Christopher Moyado
	Elaborada para la estabilización de superficies de control de un aeronave
	utilizando un MPU6050
	
	PID lib made by Christopher Moyado
	For the stabilization of ailerons, rudder and elevator of an aircraft
	Using a MPU6050 ELECTRIC CATS
*/

#include <ServoStabilizer.h>

								//int pitchPin, int rollPin, int yawPin, int throttlePin
ServoStabilizer::ServoStabilizer() {
  /*
  pitchServo.attach(pitchPin);
  rollServo.attach(rollPin);
  /*
  yawServo.attach(yawPin);
  throttleServo.attach(throttlePin);
  */
}

void ServoStabilizer::initialize() {
  Wire.begin();
  mpu.initialize();
  /* Keep at comments if you dont want to set the servos here
  pitchServo.writeMicroseconds(1500);
  rollServo.writeMicroseconds(1500); //SERVOS TO 0 
  yawServo.writeMicroseconds(1500);
  throttleServo.writeMicroseconds(1000); //Motor to 0% throttle
  */
  delay(1000); //Not needed or can be reduced to 500ms
}

void ServoStabilizer::update() {
  // Get MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch, roll, and yaw angles
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  
  // Calculate errors
  pitchError = pitchSetpoint - pitch;
  rollError = rollSetpoint - roll;
  
  // Calculate integrals
  pitchIntegral += pitchError * (millis() - lastTime) / 1000.0;
  rollIntegral += rollError * (millis() - lastTime) / 1000.0;

  // Calculate derivatives
  pitchDerivative = (pitchError - lastPitchError) / (millis() - lastTime) * 1000.0;
  rollDerivative = (rollError - lastRollError) / (millis() - lastTime) * 1000.0;

  // Calculate outputs
  pitchOutput = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative;
  rollOutput = kp * rollError + ki * rollIntegral + kd * rollDerivative;

  if (pitchOutput > 500){pitchOutput = 500;}
  if (pitchOutput < -500){pitchOutput = -500;}
  if (rollOutput > 500){rollOutput = 500;}
  if (rollOutput < -500){rollOutput = -500;}
  
  // Write outputs to servos
  /* Keep this as comment if you dont want to write the angle directly on the servos from the library
  pitchServo.writeMicroseconds(1500 - pitchOutput);
  rollServo.writeMicroseconds(1500 + rollOutput);
  yawServo.writeMicroseconds(1500 + yawOutput);
  */

  // Store current values as last values
  lastTime = millis();
  lastPitchError = pitchError;
  lastRollError = rollError;
}

//PID factors declaration
void ServoStabilizer::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

//Setpoints declaration
void ServoStabilizer::setAngles(float pitchAngle, float rollAngle) {
  pitchSetpoint = pitchAngle;
  rollSetpoint = rollAngle;
}
//PID correction output for pitch channel
float ServoStabilizer::outputsPitch() {
  return pitchOutput;
}

//PID correction output for roll channel
float ServoStabilizer::outputsRoll() {
  return rollOutput;
}

//MPU functions for outputs
float ServoStabilizer::pitchValue(){
	return pitch;
}

float ServoStabilizer::rollValue(){
	return roll;
}
 
