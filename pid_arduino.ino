#include <Servo.h>
const int input_pin = A0;
const int output_pin = 9;

Servo ESC;
int angulo, vel = 1000; 
double deltatiempo, tiempo_anterior, integral, antes, output = 0;
double kp, ki, kd, setpoint= -30;
double alfa, theta, potcorr;

void setup() {
  ESC.attach(9);
  ESC.writeMicroseconds(1000);
  kp= 15;
  //ki= 30;
  //kd= 0.1;
  tiempo_anterior= 0;
  Serial.begin(9600);
  Serial.setTimeout(10);
  /*
  analogWrite(output_pin, 0);
  for(int i= 0; i<50; i++){
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  */
}

void loop() {
  unsigned long ahora = millis();
  deltatiempo= (ahora - tiempo_anterior)/1000.00;
  tiempo_anterior = ahora;
  double sensor= angulos(input_pin,alfa);
  double error= setpoint - sensor;
  output= pid(error);
  ESC.writeMicroseconds(output);

  //Establecimiento de setpoint
  if(Serial.available() >= 1)
  {
    setpoint = Serial.parseInt(); //Leer un entero por serial
  }
  
  Serial.print(ahora);
  Serial.print(" ");
  Serial.print(tiempo_anterior);
  Serial.print(" ");
  Serial.print(deltatiempo);
  Serial.print(" ");
  Serial.print(antes);
  Serial.print(" ");
  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print(sensor);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  Serial.println(output);
}

double pid(double error){
  double proporcional = error;
  integral += error*deltatiempo;
  double derivativo= (error - antes)/deltatiempo;
  antes = error;
  double output= (kp*proporcional) + (ki*integral) + (kd*derivativo);
  output = map(output, 0, 255, 1000, 2000);
  return output;
}
float yn_1 = 0;
float fiir(int input_pin, float alfa){
  float xn, yn;
  alfa = 0.17;
  yn = alfa*float(analogRead(input_pin)) + (1-alfa)*yn_1;
  yn_1 = yn;
  return yn;
}

#define A0_raw_maximum  769
#define A0_raw_minimum  1007 
#define angle_maximum   34.1
#define angle_minimum   -34.1
float angulos(int input_pin, float alfa){
  potcorr= fiir(input_pin,alfa);
  theta = (34.1-(-34.1))/(A0_raw_maximum-A0_raw_minimum)*(potcorr-A0_raw_minimum)-34.1;
  //Serial.print(theta);
  return theta;
}
