#include <PID_v1.h>

#include <EEPROM.h>
#include "SOutput.h"

// DIGITAL IN
#define D_HALL 2 // Hall sensor
#define D_BRK 3 // Break button
#define D_CTL 4 // Control button
#define D_FC 5 // Forward current leak ?
#define D_RC 6 // Reverse current leak ?
// ANALOG IN
const float V_CONV = 5 / 1024.0; // 1024bits over 5V
double analogVoltage(byte pin) {
  return analogRead(pin) * V_CONV;
}
#define A_12V 0 // 12V Volatge
#define A_24V 1 // 24V Voltage
#define A_POT 2 // Potentiometer
// OUTPUT
#define O_BUZZ 5 // Buzzer
#define O_FWD 8 // Enable forward PWM
#define O_RWD 9 // Enable reverse PWM
#define PWM_F 10 // Forward PWM
#define PWM_R 11 // Reverse PWM

// --------------------------------EEPROM
#define F_TRIM 0 // Address for force trim
float force_trim;

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
#define PID_kP 20
#define PID_kI 0.08
#define PID_kD 0.65 //
double pid_goal = 1;
double pwm_out = 0;
void output() {
  // if (currentForce < 1) { pwm_out = 0; }
  digitalWrite(O_FWD, pwm_out <= 0 ? HIGH : LOW);
  digitalWrite(O_RWD, pwm_out >= 0 ? HIGH : LOW);
  analogWrite(pwm_out < 0 ? PWM_R : PWM_F, pwm_out);
}
PID pid(&currentForce, &pwm_out, &pid_goal, PID_kP, PID_kI, PID_kD, REVERSE);

// -------------------------------------------------------------------------------- Frequency measurement
#define MIN_FREQ 2.0
const long maxdiff = MIN_FREQ * 1000000;
float lastFreq = 0;
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
float freq() {
  unsigned short lastpulses = pulses;
  float f = lastFreq;
  if (lastpulses > 1) {
    noInterrupts();
    pulses = 0;
    f = (lastpulses * (double) 1000000) / (lastPulse - firstPulse);
    interrupts();
    lastFreq = f;
  } else if (micros() - lastPulse > maxdiff) {
    lastFreq = 0;
  }
  return f;
}

// -------------------------------------------------------------------------------- Break button
#define DEBOUNCE_DELAY 50 //millis
volatile unsigned long lastBreak = 0;
volatile boolean breaking = false;
void ISR_breakButton() {
  lastBreak = millis();
  breaking = true;
}
void updateBreak() {
  if (breaking && digitalRead(D_BRK) != LOW && millis() - lastBreak > DEBOUNCE_DELAY) {
    breaking = false;
  }
}

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

  pinMode(O_BUZZ, OUTPUT);
  pinMode(O_FWD, OUTPUT);
  pinMode(O_RWD, OUTPUT);
  pinMode(PWM_F, OUTPUT);
  pinMode(PWM_R, OUTPUT);

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
  pid.Compute();
  output();
  updateVoltages();
  
  SOutput out[] = {
    {F("FREQ"), freq()},
    {F("FORCE"), currentForce},
    // {F("12V"), volt_12},
    // {F("24V"), volt_24},
    // {F("CHG_A"), chg_a},
    // {F("CHG_B"), chg_b},
    {F("PWN_OUT"), pwm_out},
    {F("FTRIM"), force_trim},
    {F("BREAK"), breaking},
    {F("CRTL"), control}
  };
  serialOut(out, 6);
}