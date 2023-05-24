#include <Encoder.h>

#define POT A0
float sp; // Valor deseado en RPM
int motor_a = 3;
int PWM_salida = 11;
float pv; // Valor medido
int ENCODER_A = 2; // Señales del encoder
int ENCODER_B = 8;
volatile int contador = 0; // Contador del encoder
unsigned long previousMillis = 0;
long interval = 100; // Tiempo de muestreo de la señal del encoder
float velMax = 6500.0; // Velocidad maxima en RPM

float cv;
float cv1;
float error;
float error1;
float error2;

// Constantes del PID
float kp = 1;
float ki = 7; // kd >> Reduce el tiempo de estado estacionario
float kd = 0.001;
float Tm = 0.1;

Encoder encoder(ENCODER_A, ENCODER_B);

void setup() {
  pinMode(ENCODER_A,INPUT);
  pinMode(ENCODER_B,INPUT);
  pinMode(PWM_salida, OUTPUT);
  pinMode(motor_a, OUTPUT);
  Serial.begin(9600);
  //attachInterrupt(ENCODER_A, interrupcion, RISING);

  digitalWrite(motor_a, LOW);

}

void contadorToRPM() {
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;
    contador = encoder.read();
    pv = 10*contador*(60.0/48.0); // RPM
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
  Serial.println(error);  
}

void loop() {    

  contadorToRPM();

  // Se lee el valor deseado desde el potenciometro
  sp = analogRead(POT)*(velMax/882); //1023 // de 0 a 7000 rpm    

  //Serial.println(analogRead(POT));

  if (sp < 1000) sp = 0;

  // Calculo del control PID
  controlPID();

  // Se envia la señal tratada por el PID a la salida
  analogWrite(PWM_salida, cv*(255.0/velMax)); // De 0 a 255

  printDatos();

  delay(100);
}
