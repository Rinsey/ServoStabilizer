#include <ServoStabilizer.h>
#include <Servo.h>
#include <MPU6050.h>

ServoStabilizer pid;
float rollPID, pitchPID; //Para mostrar la salida de los valores PID

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200);
  mpu.initialize();
  pid.begin();
  pid.setGains(20.0,2.0,0.03); // valores kP, kI, kD
  pid.setAngles(0,0); //Para setear ángulos a los cuales se estabilizará
}

void loop() {
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  
  pid.update(pitch, roll); // Ejecutas los métodos de estabilización
  pitchPID = pid.outputsPitch(); //.outputsPitch(); da el valor de salida PID en microSeconds (Correccion)
  rollPID = pid.outputsRoll(); //.outputsRoll(); da el valor de salida PID en microSeconds (Correccion)
  
  Serial.print(rollPID);
  Serial.print("     ");
  Serial.println(pitchPID);
}
