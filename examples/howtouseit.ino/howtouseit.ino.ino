#define AileronPin 1
#define ElevatorPin 1
#define RudderPin 1
#define ThrottlePin 1

#include <ServoStabilizer.h>

ServoStabilizer estabilizacion(ElevatorPin,AileronPin,RudderPin,ThrottlePin);

void setup() {
  
  estabilizacion.initialize();
  estabilizacion.setGains(0.03,0.01,0.03);
  estabilizacion.setAngles(0,0,0);

}

void loop() {
  // put your main code here, to run repeatedly:
  estabilizacion.update();

}
