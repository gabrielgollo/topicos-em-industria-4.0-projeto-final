#include "Arduino.h"
#include "lcd_st7567s.h"
#include "PID_v1.h"

// Constantes para pinos
const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;
const byte TACH_INTERRUPT_PIN = 2;
const byte ANALOG_SENSOR_PIN = A0;

// Constantes para PWM
const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

// Variáveis para o Tach
volatile int count = 0;
unsigned long int lastInterruptTime = 0;
//unsigned long int start_time;

int rpm;
int sensorValue;


// PID

double Setpoint, Input, Output;

double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(OC1A_PIN, OUTPUT);
  pinMode(OC1B_PIN, INPUT);

  configureTimer1();

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(TACH_INTERRUPT_PIN), counter, RISING);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void configureTimer1() {
  // Configurar Timer1
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

void updateTachValue() {
  //start_time = millis();
  //count = 0;
  //while (millis() - start_time < 100) {}

  //rpm = count * 60 / 2;

  Serial.println(rpm);
}

int readAnalogSensor() {
  return map(analogRead(ANALOG_SENSOR_PIN), 0, 1023, 0, 100);
}

void serialReadingProccess() {
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();

    // Print the received byte
    Serial.print("Received: ");
    Serial.println(incomingByte);
  }
}

void loop() {
  sensorValue = readAnalogSensor();
  //Serial.print("analog: ");
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.println(rpm);

  setPwmDuty(sensorValue);
  //delay(10);

  myPID.Compute();
}

void setPwmDuty(byte duty) {
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

void counter() {
  unsigned long int currentTime = millis();
  unsigned long int elapsedTime = currentTime - lastInterruptTime;

  // Serial.print(currentTime);
  // Serial.print(",");
  // Serial.println(elapsedTime);
  
  if (elapsedTime >= 1500) { // Medir RPM a cada 1 segundo (em milisegundos)
    rpm = count * 40/2;
    //Serial.println(rpm);
    lastInterruptTime = currentTime;
    count = 0; // Zera o contador
    return;
  }
   
   count++;
  // Não realizar operações demoradas dentro da interrupção
}
