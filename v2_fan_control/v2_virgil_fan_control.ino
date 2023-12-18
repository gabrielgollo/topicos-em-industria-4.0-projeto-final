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
// unsigned long int start_time;

double rpm;
int sensorValue;

// PID variables Setpoint is the RPM value we want to achieve
double Setpoint, Input, Output;
double Kp = 0.5, Ki = 0.1, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void configureTimer1()
{
  // Configurar Timer1
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

int readAnalogSensor()
{
  return map(analogRead(ANALOG_SENSOR_PIN), 0, 1023, 0, 100);
}

void setup()
{
  pinMode(OC1A_PIN, OUTPUT);
  pinMode(OC1B_PIN, INPUT);

  configureTimer1();

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(TACH_INTERRUPT_PIN), counter, RISING);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  Setpoint = 1040;
  setPwmDuty(1);
}

void loop()
{
  myPID.Compute();

  serialReadingProccess();
}

void processMessage(String message)
{
  // if message starts with setpoint then set the setpoint
  if (message.startsWith("setpoint"))
  {
    Setpoint = message.substring(9).toInt();
    Serial.print("Setpoint: ");
    Serial.println(Setpoint);

    // max setpoint is 2370 rpm
    if (Setpoint > 2370)
    {
      Setpoint = 2370;
    }

    if (Setpoint < 0)
    {
      Setpoint = 0;
    }

    // compute new PID values
    myPID.Compute();

    // log new PID values
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // log new PID values
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output);
  }

  if (message.startsWith("kp"))
  {
    Kp = message.substring(2).toFloat();
    Serial.print("Kp: ");
    Serial.println(Kp);

    // compute new PID values
    myPID.Compute();

    // log new PID values
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // log new PID values
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output);
  }

  if (message.startsWith("ki"))
  {
    Ki = message.substring(2).toFloat();
    Serial.print("Ki: ");
    Serial.println(Ki);

    // compute new PID values
    myPID.Compute();

    // log new PID values
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // log new PID values
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output);
  }

  if (message.startsWith("kd"))
  {
    Kd = message.substring(2).toFloat();
    Serial.print("Kd: ");
    Serial.println(Kd);

    // compute new PID values
    myPID.Compute();

    // log new PID values
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // log new PID values
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output);
  }

  if (message.startsWith("pid"))
  {
    String pidValues = message.substring(3);
    int commaIndex = pidValues.indexOf(",");
    Kp = pidValues.substring(0, commaIndex).toFloat();
    pidValues = pidValues.substring(commaIndex + 1);
    commaIndex = pidValues.indexOf(",");
    Ki = pidValues.substring(0, commaIndex).toFloat();
    pidValues = pidValues.substring(commaIndex + 1);
    commaIndex = pidValues.indexOf(",");
    Kd = pidValues.substring(0, commaIndex).toFloat();

    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // compute new PID values
    myPID.Compute();

    // log new PID values
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    // log new PID values
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output);
  }

  if (message.startsWith("pwm"))
  {
    int pwmValue = message.substring(3).toInt();
    Serial.print("pwm: ");
    Serial.println(pwmValue);

    setPwmDuty(pwmValue);
  }

  if (message.startsWith("rpm"))
  {
    Serial.print("rpm: ");
    Serial.println(rpm);
  }
}

void serialReadingProccess()
{
  if (Serial.available() > 0)
  {
    // Read the incoming byte
    String received = Serial.readString();

    if (received.length() > 0)
    {
      // Print the received byte
      Serial.print("Received: ");
      Serial.println(received);

      processMessage(received);
    }
  }
}

void setPwmDuty(byte duty)
{
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

void counter()
{
  unsigned long int currentTime = millis();
  unsigned long int elapsedTime = currentTime - lastInterruptTime;

  // Serial.print(currentTime);
  // Serial.print(",");
  // Serial.println(elapsedTime);

  if (elapsedTime >= 2000)
  { // Medir RPM a cada 1 segundo (em milisegundos)
    rpm = count * 30 / 2;
    Input = rpm;
    // Serial.println(rpm);
    lastInterruptTime = currentTime;
    count = 0; // Zera o contador
    return;
  }

  count++;
  // Não realizar operações demoradas dentro da interrupção
}
