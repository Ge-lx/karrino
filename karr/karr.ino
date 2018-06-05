#include <PID_v1.h>

#include <EEPROM.h>
#include "SOutput.h"

// DIGITAL IN
#define D_HALL 2 // Hall sensor
#define D_BRK 3 // Break button
#define D_CTL 5 // Control button
#define D_FC 6 // Forward current leak ?
#define D_RC 7 // Reverse current leak ?
// ANALOG IN
const float V_CONV = 5 / 1024.0; // 1024bits over 5V
double analogVoltage(byte pin) {
  return analogRead(pin) * V_CONV;
}
#define A_12V 0 // 12V Voltage
#define A_24V 1 // 24V Voltage
#define A_POT 2 // Potentiometer
// OUTPUT
#define O_FWD 8 // Enable forward PWM
#define O_RWD 9 // Enable reverse PWM
#define PWM_F 11 // Forward PWM
#define PWM_R 10 // Reverse PWM

// --------------------------------EEPROM
#define F_TRIM 0 // Address for force trim
float force_trim;

// -------------------------------------------------------------------------------- Frequency measurement
#define MIN_FREQ 2.0
const long maxdiff = MIN_FREQ * 1000000;
float freq = 0;
volatile unsigned short pulses = 0;
volatile unsigned long firstPulse = 0;
volatile unsigned long lastPulse = 0;
void ISR_hallSensor() {
  if (pulses == 0) {
    firstPulse = micros();
  } else {
    lastPulse = micros();
  }
  pulses++;
}
void updateFreq() {
  unsigned short lastpulses = pulses;
  if (lastpulses > 1) {
    noInterrupts();
    pulses = 0;
    freq = (lastpulses * (double) 1000000) / (lastPulse - firstPulse);
    interrupts();
  } else if (micros() - lastPulse > maxdiff) {
    freq = 0;
  }
}

// -------------------------------------------------------------------------------- V -> % conversion
#define RATIO_12V 3 // Voltage divider trim
#define RATIO_24V 6
#define euler 2.71828182846
// Conversion weights
const double VC_A = 2.128e23;
const double VC_B = 4.2696202;
const double VC_C = 132.53558;
//storage variables
#define UPDATE_FREQ 2000 //ms
unsigned long lastVoltUpdate = 0;
float volt_12, volt_24;
byte chg_a, chg_b;
// returns charge in percent
byte volt_to_charge (float v) { return VC_C / (1 + (VC_A * pow(euler, ( -1 * VC_B * ((double) v))) )); }
void updateVoltages() {
  if (millis() - lastVoltUpdate > UPDATE_FREQ) {
    volt_12 = RATIO_12V * analogVoltage(A_12V);
    volt_24 = RATIO_24V * analogVoltage(A_24V);
    chg_a = volt_to_charge(volt_12);
    chg_b = volt_to_charge(volt_24 - volt_12);
    lastVoltUpdate = millis();
  }
}

// -------------------------------------------------------------------------------- Potentiometer + Centimeter -> Newton
#define TRIM_TO 5
const float POT_UREF = 3.3;
double currentForce = 0;
float updateForce() {
  double force = TRIM_TO * ((analogVoltage(A_POT) / POT_UREF) / force_trim);
  currentForce = force < 0.2 ? 0 : force;
}
void updateFTRIM() {
  force_trim = (analogVoltage(A_POT) / POT_UREF);
  EEPROM.put(F_TRIM, force_trim);
}

// -------------------------------------------------------------------------------- PID
#define PID_kP 25.5
#define PID_kI 0
#define PID_kD 0.2
#define FORCE_GOAL 1
#define MAX_INT_ERROR 2
double pid_goal = FORCE_GOAL;
double pid_out = 0;
double intError = 0;
double last_error = 0;
unsigned long lastCalcTime = millis();
// void calcPID() {
//   double error = currentForce - pid_goal; // Positive: too much force -> speed up
//   double dt = (millis() - lastCalcTime);

//   lastCalcTime = millis();
//   intError += error * PID_kI * dt; 
//   if (intError > MAX_INT_ERROR) intError = MAX_INT_ERROR;
//   if (intError < -1* MAX_INT_ERROR) intError = -1 * MAX_INT_ERROR;

//   double derivError = (error - last_error) / (double) dt;
  

//   pid_out = intError + (error * PID_kD + derivError * PID_kD) * PID_GAIN;
//   pid_out = pid_out > 255 ? 255 : (pid_out < 0 ? 0 : 1) * pid_out;
// }
void output() {
  analogWrite(pid_out < 0 ? PWM_R : PWM_F, pid_out);
}
PID pid(&currentForce, &pid_out, &pid_goal, PID_kP, PID_kI, PID_kD, REVERSE);

// -------------------------------------------------------------------------------- Control button
#define CONTROL_DEBOUNCE 50 //millis
unsigned long lastControl = 0;
boolean control = false;
void updateControl() {
  if (digitalRead(D_CTL) == LOW) {
    control = true;
    updateFTRIM();
    lastControl = millis();
  }
  if (control && digitalRead(D_CTL) != LOW && millis() - lastControl > CONTROL_DEBOUNCE) {
    control = false;
  }
}

// -------------------------------------------------------------------------------- Break button
#define DEBOUNCE_DELAY 100 //millis
volatile unsigned long lastBreak = 0;
volatile double breakForce = 0;
volatile boolean breaking = false;
void ISR_breakButton() {
  lastBreak = millis();
  breaking = true;
  breakForce = currentForce + 0.5;
}
void updateBreak() {
  if ( breaking 
    && (digitalRead(D_BRK) != LOW && millis() - lastBreak > DEBOUNCE_DELAY)
    && currentForce > breakForce) {
    breaking = false;
  }
}

void serialOut(SOutput out[], short size) {
  if (!Serial) { return; }
  for (byte i = 0; i < size; i++) {
    Serial.print(out[i].name);
    Serial.print(": ");
    Serial.print(out[i].value);
    Serial.print("\t");
  }
  Serial.println("");
}

void setup()
{
  EEPROM.get(F_TRIM, force_trim);
  Serial.begin(9600); // Try to establish serial connection

  pinMode(O_FWD, OUTPUT);
  pinMode(O_RWD, OUTPUT);
  pinMode(PWM_F, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  //setPWMfrequency(10, 15);
  //setPWMfrequency(11, 15);

  digitalWrite(O_RWD, HIGH);
  digitalWrite(O_FWD, HIGH);

  //Break button
  pinMode(D_BRK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D_BRK), ISR_breakButton, FALLING);
  //Control button
  pinMode(D_CTL, INPUT_PULLUP);

  // Frequency measurement
  pinMode(D_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D_HALL), ISR_hallSensor, FALLING);

  pid.SetMode(AUTOMATIC);
}

void loop() {
  updateBreak();
  updateControl();
  updateForce();
  updateFreq();

  //pid_goal = breaking ? 15 : FORCE_GOAL;
  //calcPID();
  pid.Compute();
  output();
  updateVoltages();
  
  SOutput out[] = {
    {F("FREQ"), freq},
    {F("FORCE"), currentForce},
    {F("PID_GOAL"), pid_goal},
    {F("12V"), volt_12},
    {F("24V"), volt_24},
    {F("PID_OUT"), pid_out},
    {F("FTRIM"), force_trim},
    {F("BREAK"), breaking ? 20 : 0},
  };
  serialOut(out, 8);
}