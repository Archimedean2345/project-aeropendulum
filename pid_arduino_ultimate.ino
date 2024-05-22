#include <Servo.h>
#define tiempo_muestreo 50
#define A0_raw_maximum  825
#define A0_raw_minimum  1023
#define maximum_angle   57
#define minimum_angle   0
#define max_pwm 1500
#define kp_pot A1
#define ki_pot A2
#define input_angle A0

Servo ESC;
float deltatiempo, tiempo_anterior, integral, antes, pwm_output;
float slope, u_t, kp, ki, kd, setpoint= -1;
float alfa, theta, sensor, analogo;
unsigned long tiempo_inicio;

void setup() {
  ESC.attach(9);
  ESC.writeMicroseconds(1000);
  slope= 2.45;
  kp= 1;
  ki= 0.00;
  kd= 0.00;
  tiempo_anterior= 0;
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop() {
//Muestreo de tiempo
  if(millis() - tiempo_inicio >= tiempo_muestreo){
    tiempo_inicio = millis();
    deltatiempo= (tiempo_inicio - tiempo_anterior)/1000.00;
    tiempo_anterior = tiempo_inicio;
    float sensor= mapf(analogo);
    // Cuidado siempre ver que este restando
    float error= setpoint - sensor;
    pwm_output= int(pid(error,sensor));

    //Establecimiento de setpoint y seguro para apagarlo
    if(Serial.available() >= 1){
      setpoint = Serial.parseInt(); //Leer un entero por serial
    }
    if(setpoint == -1){
      pwm_output= 1000;
    }
    //Modifica el pulso enviado al motor
    ESC.writeMicroseconds(pwm_output);
    //Imprime datos
      Serial.print(tiempo_inicio);
      Serial.print(" ");
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(sensor);
      Serial.print(" ");
      Serial.print(error);
      Serial.print(" ");
      Serial.println(pwm_output);

  }
}

float pid(float error, float angulo){
  float proporcional = error;
  integral += error*deltatiempo;
  float derivativo= (error - antes)/deltatiempo;
  antes = error;
  float kp = filtro(analogRead(kp_pot), 0, 1023, 0.00, 6.00);
  float ki = filtro(analogRead(ki_pot), 0, 1023, 0.00, 1.00);
  float u_t= (kp*proporcional) + (ki*integral) + (kd*derivativo);
  Serial.print(u_t);
  Serial.print(" ");
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  //double u = 2.1357*u_t + 1226.2;
  float u= 1240.00 + u_t + slope*angulo;
  if(u > max_pwm)
    u = 1226.20;
  return u;
}

//Filtro y angulo filtrado 
  float yn_1 = 0;
float mapf(float analogo){
  //Filtro
    float xn, yn;
    float alfa = 0.17;
    yn = alfa*float(analogRead(A0)) + (1-alfa)*yn_1;
    yn_1 = yn;

  theta = (maximum_angle - (minimum_angle))* (yn - A0_raw_minimum)/ (A0_raw_maximum - A0_raw_minimum)  + minimum_angle;
  //Serial.print(theta);
  if(theta < 0){
    theta= 0;
  }
  return theta;
}


//Filtro kp y ki
float filtro(float valor, float in_min, float in_max, float out_min, float out_max){
  //Filtro
    
  // Calculo valor
    float valor_corregido= (valor - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
    return valor_corregido;
}
