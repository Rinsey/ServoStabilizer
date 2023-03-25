#define AileronPin 29
#define ElevatorPin 28
#define ThrottlePin 27
#define RudderPin 26

#include <ServoStabilizer.h>
#include <Servo.h>
#include <CrsfSerial.h>

ServoStabilizer estabilizacion(ElevatorPin,AileronPin,RudderPin,ThrottlePin);
float roll, pitch, yaw; //Para mostrar la salida de los valores PID

SerialPIO Receiver(10,11);
CrsfSerial crsf(Receiver, 200000);|

void setup() {

  Serial.begin(115200);
  
  estabilizacion.initialize(); //La posicion de Servos a 1500 y throttle 1000 microSeconds
  estabilizacion.setGains(20.0,2.0,0.03); // valores kP, kI, kD
  estabilizacion.setAngles(0,0,0); //Para setear ángulos a los cuales se estabilizará
  /*
   * La función setAngles(float,float,float); es la función a utilizar para lockear la
   * estabilización a un angulo definido, si queremos que el avion se mantenga en un rumbo
   * fijo pues utilizaremos la funcion para setear el angulo que queramos. Con un if es 
   * suficiente para guardar la última posicion que el ELRS reciba al activar el switch
   * 
   * Ejemplo
   * 
   * if (crsf.getChannel(switch) > 1500) {
   *  
   *  estabilizacion.setAngles(angleX, angleY, angleZ); //Guardas los últimos datos de los canales
   *  estabilizacion.update(); //Actualizaciones y salidas de PID
   *  
   * }else {
   * 
   *  angleX = crsf.getChannel(1);
   *  angleY = crsf.getChannel(2);
   *  angleZ = crsf.getChannel(4);
   *  
   * }
   * 
   */
}

void loop() {

  crsf.loop();
  estabilizacion.update(); // Ejecutas los métodos de estabilización
  
  pitch = estabilizacion.outputsPitch(); //.outputsPitch(); da el valor de salida PID
  roll = estabilizacion.outputsRoll(); //.outputsRoll(); da el valor de salida PID
  yaw = estabilizacion.outputsYaw(); //.outputsYaw(); da el valor de salida PID
  Serial.print(roll);
  Serial.print("     ");
  Serial.print(pitch);
  Serial.print("     ");
  Serial.println(yaw); 

}

