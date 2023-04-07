
#include <ServoStabilizer.h>
#include <Servo.h>
#include <MPU6050.h>

ServoStabilizer estabilizacion();
float rollPID, pitchPID; //Para mostrar la salida de los valores PID
MPU6050 mpu;

void setup() {

  Serial.begin(115200);
  mpu.initialize();
  estabilizacion.initialize(); //La posicion de Servos a 1500 y throttle 1000 microSeconds
  estabilizacion.setGains(20.0,2.0,0.03); // valores kP, kI, kD
  estabilizacion.setAngles(0,0); //Para setear ángulos a los cuales se estabilizará
}

void loop() {
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  
  estabilizacion.update(pitch, roll); // Ejecutas los métodos de estabilización
  pitchPID = estabilizacion.outputsPitch(); //.outputsPitch(); da el valor de salida PID en microSeconds (Correccion)
  rollPID = estabilizacion.outputsRoll(); //.outputsRoll(); da el valor de salida PID en microSeconds (Correccion)
  
  Serial.print(rollPID);
  Serial.print("     ");
  Serial.println(pitchPID);
}

