/*	
	Librería PID elaborada por Christopher Moyado
	Elaborada para la estabilización de superficies de control de un aeronave

	
	PID lib made by Christopher Moyado
	For the stabilization of ailerons, rudder and elevator of an aircraft
*/

#ifndef ServoStabilizer_h 
#define ServoStabilizer_h

#include <Wire.h> 
#include <Arduino.h> //Not needed

class ServoStabilizer {
	
  public:
    ServoStabilizer(); //Start your object in IDE
    void begin(); //Start the i2c communication, Servo to zero position (1500microSeconds)
    void update(float pitchData, float rollData); //PID outputs here
    void setGains(float kp, float ki, float kd); //PID factors declarations
    void setAngles(float pitchAngle, float rollAngle); //Setpoints
	float outputsPitch();
	float outputsRoll();
	float pitchOutput, rollOutput;
	float pitchValue();
	float rollValue();
	float pitch, roll;
	
  private:
    float kp, ki, kd;
    float pitchSetpoint, rollSetpoint;
    float pitchError, rollError;
    float pitchIntegral, rollIntegral;
    float pitchDerivative, rollDerivative;
	float lastPitchError, lastRollError;
    unsigned long lastTime;
	
};

#endif
