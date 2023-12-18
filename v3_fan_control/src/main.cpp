#include "Arduino.h"
#include "ArduinoJson.h"
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

int rpm;
int sensorValue;

// PID variables Setpoint is the RPM value we want to achieve
double Setpoint, Input, Output;
double Kp = 0.02, Ki = 0.001, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void configureTimer1()
{
  // Configurar Timer1
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

void setPwmDuty(byte duty)
{
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

void counter()
{
  unsigned long int currentTime = millis();
  unsigned long int elapsedTime = currentTime - lastInterruptTime;

  if (elapsedTime >= 1000)
  { // Medir RPM a cada 1 segundo (em milisegundos)
    rpm = count * 60 / 2;
    Input = rpm;
    lastInterruptTime = currentTime;
    count = 0; // Zera o contador
    return;
  }
  // Serial.println(count);

  count++;
  // Não realizar operações demoradas dentro da interrupção
}

void mountAndSendDataWithJson()
{
  StaticJsonDocument<512> doc;

  doc["type"] = "datas";
  doc["rpm"] = rpm;
  doc["setpoint"] = Setpoint;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["input"] = Input;
  doc["output"] = Output;

  String json;
  serializeJson(doc, json);

  Serial.println(json);
}

void processMessage(String message)
{
  // if message starts with setpoint then set the setpoint
  if (message.startsWith("setpoint"))
  {
    Setpoint = message.substring(9).toInt();

    // max setpoint is 2370 rpm
    if (Setpoint > 2400)
    {
      Setpoint = 2370;
    }

    if (Setpoint < 0)
    {
      Setpoint = 1100;
    }
  }

  if (message.startsWith("kp"))
  {
    Kp = message.substring(2).toFloat();

    // compute new PID values
    myPID.SetTunings(Kp, Ki, Kd);
  }

  if (message.startsWith("ki"))
  {
    Ki = message.substring(2).toFloat();

    // compute new PID values
    myPID.SetTunings(Kp, Ki, Kd);
  }

  if (message.startsWith("kd"))
  {
    Kd = message.substring(2).toFloat();

    // compute new PID values
    myPID.SetTunings(Kp, Ki, Kd);
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

    // compute new PID values
    myPID.SetTunings(Kp, Ki, Kd);
  }

  if (message.startsWith("pwm"))
  {
    int pwmValue = message.substring(3).toInt();

    setPwmDuty(pwmValue);
  }

  if (message.startsWith("json"))
  {
    mountAndSendDataWithJson();
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
      processMessage(received);
    }
  }
}

void transformSetpointToPwm(float reference)
{
  if (reference <= 1050)
  {
    setPwmDuty(0);
  }
  else if (reference > 2370)
  {
    setPwmDuty(100);
  }
  else
  {
    setPwmDuty(map((int)reference, 1050, 2370, 0, 100));
  }
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
  Setpoint = 1050;
  transformSetpointToPwm(Setpoint);
}

void loop()
{

  // should compute pid values and set pwm duty cycle
  myPID.Compute();
  transformSetpointToPwm(map(Output, 0, 255, 1050, 2370));

  // process serial messages
  serialReadingProccess();

  // mountAndSendDataWithJson();
  mountAndSendDataWithJson();
  delay(1005);
}