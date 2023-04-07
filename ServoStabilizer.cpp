/*	
	Librería PID elaborada por Christopher Moyado
	Elaborada para la estabilización de superficies de control de un aeronave
	
	PID lib made by Christopher Moyado
	For the stabilization of ailerons, rudder and elevator of an aircraft
*/

#include <ServoStabilizer.h>

ServoStabilizer::ServoStabilizer() {

}

void ServoStabilizer::initialize() {
  Wire.begin();
  delay(1000); //Not needed or can be reduced to 500ms
}

void ServoStabilizer::update(float pitchData, float rollData) {
	
  // New pitch n roll data.
  pitch = pitchData;
  roll = rollData;
  
  //Use these equations if you want to calculate the pitch n roll
  //pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  //roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  
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
