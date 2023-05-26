#include <Encoder.h>

#define POT A0
float sp; // Valor deseado en RPM
int motor_a = 3; // Pines del motor o puente H
int motor_b = 11;
float pv; // Valor medido
int ENCODER_A = 2; // Señales del encoder
int ENCODER_B = 8;
volatile int contador = 0; // Contador del encoder
unsigned long previousMillis = 0;
long interval = 100; // Tiempo de muestreo de la señal del encoder
float velMax = 6500.0; // Velocidad maxima en RPM
float velMin = 0.0;
float pMax = 860; // Valor maximo leido por el potenciometro
float pMin = 15; // Valor minimo leido por el potenciometro
int sentidoGiro = 0; // 1 = horario, -1 = antihorario, 0 = quieto

float cv;
float cv1;
float error;
float error1;
float error2;

// Constantes del PID
float kp = 1;
float ki = 7; // ki >> Reduce el tiempo de estado estacionario
float kd = 0.001;
float Tm = 0.1;

Encoder encoder(ENCODER_A, ENCODER_B);

void setup() {
  pinMode(ENCODER_A,INPUT);
  pinMode(ENCODER_B,INPUT);
  pinMode(motor_b, OUTPUT);
  pinMode(motor_a, OUTPUT);
  Serial.begin(9600);  
}

void contadorToRPM() {
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;
    contador = encoder.read();
    pv = 10*abs(contador)*(60.0/48.0); // RPM    
    encoder.write(0);          
  }
}

void controlPID() {
  error = sp - pv;

  // Ecuacion de diferencias
  cv = cv1 + (kp+ kd/Tm)*error + (-kp + ki*Tm - 2*kd/Tm)*error1 + (kd/Tm)*error2;
  cv1 = cv;
  error2 = error1;
  error1 = error;

  //Se satura la salida del PID
  if(cv > velMax){
    cv = velMax;
  }
  if(cv < 300.0){
    cv = 300.0;
  }  
}

void printDatos() {
  Serial.print("sp: ");
  Serial.print(sp);
  Serial.print(", pv: ");
  Serial.print(pv);
  Serial.print(", error: ");
  Serial.print(error);  
  Serial.print(", cv: ");
  Serial.println(cv);
}

void potenciometroARPM() {
  float pot = analogRead(POT);
  if (pot < pMin) pot = pMin;
  if (pot > pMax) pot = pMax;
  
  float m = (2*(velMax-velMin)/(pMax+pMin));
  
  // Se lee el valor deseado desde el potenciometro
  if (pot > (pMax - pMin)/2 && pot <= pMax) {
    sentidoGiro = 1;       
    sp = m*pot + (velMax - m*pMax);
  } else if (pot <= (pMax - pMin)/2 /*&& analogRead(POT) >= 15.0*/) {
    sentidoGiro = -1;
    sp = -m*pot + (velMax + m*pMin);
  } else {
    sentidoGiro = 0;
    sp = 0.0;    
  }

  if (sp < 1000) sp = 0.0;  
}

void loop() {    

  contadorToRPM();  
  
  potenciometroARPM();

  /*Serial.print("analogRead(POT): ");
  Serial.print(analogRead(POT));
  Serial.print(", sp: ");
  Serial.println(sp);*/

  // Calculo del control PID
  controlPID();

  // Se envia la señal tratada por el PID a la salida
  if (sentidoGiro == 1) {
    analogWrite(motor_b, cv*(255.0/velMax)); // De 0 a 255 
    digitalWrite(motor_a, LOW);
  } else if (sentidoGiro == -1) {
    analogWrite(motor_a, cv*(255.0/velMax)); // De 0 a 255 
    digitalWrite(motor_b, LOW);
  } else {
    digitalWrite(motor_b, LOW);
    digitalWrite(motor_a, LOW);
  }

  printDatos();

  delay(100);
}
