
/*	
	Librería PID elaborada por Christopher Moyado
	Elaborada para la estabilización de superficies de control de un aeronave
	utilizando un MPU6050
	
	PID lib made by Christopher Moyado
	For the stabilization of ailerons, rudder and elevator of an aircraft
	Using a MPU6050 ELECTRIC CATS
*/

#ifndef ServoStabilizer_h 
#define ServoStabilizer_h

#include <Wire.h> 
#include <MPU6050.h> //Using electric cats lib from arduino lib manager
#include <Arduino.h> //Not needed

class ServoStabilizer {
	
  public:
    ServoStabilizer(); //Start your object in IDE
    void initialize(); //Start the i2c communication, Servo to zero position (1500microSeconds)
    void update(); //PID outputs here and MPU 6050 readings
    void setGains(float kp, float ki, float kd); //PID factors declarations
    void setAngles(float pitchAngle, float rollAngle); //Setpoints
	float outputsPitch();
	float outputsRoll();
	float pitchOutput, rollOutput;
	float pitchValue();
	float rollValue();
	float pitch, roll;
	
  private:
    MPU6050 mpu;
	int16_t ax, ay, az, gx, gy, gz;
    float kp, ki, kd;
    float pitchSetpoint, rollSetpoint;
    float pitchError, rollError;
    float pitchIntegral, rollIntegral;
    float pitchDerivative, rollDerivative;
	float lastPitchError, lastRollError;
    unsigned long lastTime;
	
};


#endif