#include <Servo.h>
#define input_angle A0
#define A0_raw_maximum  780
#define A0_raw_minimum  1022 
#define maximum_angle   68.2
#define minimum_angle   0

Servo esc;
int vel = 1000;
void setup() {
  esc.attach(9);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() >= 1){
    vel= Serial.parseInt();
    if(vel != 0)
    {
      esc.writeMicroseconds(vel); //Generar un pulso con el numero recibido
    }
  }
  unsigned long tiempo= millis();
  double angulo= angulos(input_angle);
  Serial.print(tiempo);
  Serial.print(" ");
  Serial.print(angulo);
  Serial.print(" ");
  Serial.print(analogRead(input_angle));
  Serial.print(" ");
  Serial.println(vel);
}
double theta;
float angulos(int input_angle){
  
  theta = (maximum_angle-(minimum_angle))/(A0_raw_maximum-A0_raw_minimum)*(analogRead(input_angle) - A0_raw_minimum) + minimum_angle;
  if(theta < 0){
    theta= 0;
  }
  return theta;
}
