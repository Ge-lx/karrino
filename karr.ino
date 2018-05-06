// DIGITAL IN
#define D_HALL 2 // Hall sensor
#define D_BRK 3 // Break button
// ANALOG IN
const float V_CONV = 1024.0 / 5; // 1024bits over 5V
float analogVoltage(byte pin) {
  return analogRead(pin) * V_CONV;
}
#define A_12V 0 // 12V Volatge
#define A_24V 1 // 24V Voltage
#define A_POT 2 // Potentiometer
// OUTPUT
#define O_BUZZ 3
#define PWM_F 10
#define PWM_R 11

void setup()
{
  Serial.begin(9600); // Try to establish serial connection

  //Break button
  pinMode(D_BRK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D_BRK), ISR_breakButton, CHANGE);
  
  // Frequency measurement
  pinMode(D_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D_HALL), ISR_hallSensor, FALLING);
}

// ------------------------------------------------ V -> % conversion
#define RATIO_12V  3 // Voltage divider trim
#define RATIO_24V  6
// Conversion weights
#define VC_A 66
#define VC_B -2349
#define VC_C 27920
#define VC_D -110736
//storage variables
float volt_12, volt_24;
byte chg_a, chg_b;
// returns charge in percent
byte volt_to_charge(float v) { return VC_A*pow(v, 3) + VC_B*pow(v, 2) + VC_C*v + VC_D; }
void updateVoltages() {
  volt_12 = RATIO_12V * analogVoltage(A_12V);
  volt_24 = RATIO_24V * analogVoltage(A_24V);
  chg_a = volt_to_charge(volt_12);
  chg_b = volt_to_charge(volt_24 - volt_12);
}

// ------------------------------------------------ Frequency measurement
#define MIN_FREQ 0.5
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

// ------------------------------------------------ Break button
#define DEBOUNCE_DELAY 50 //millis
volatile unsigned long lastBreak = 0;
volatile boolean breaking = false;
void ISR_breakButton() {
  if (digitalRead(D_BRK) == LOW) {
    lastBreak = millis();
    breaking = true;
  }
}
void updateBreak() {
  if (breaking) {
    breaking = millis() - lastBreak < DEBOUNCE_DELAY;
  }
}

void serialOut(float output[], short size) {
  if (!Serial) { return; }
  for (byte i = 0; i < size; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(output[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

void loop() {
  updateBreak();
  updateVoltages();
  
  float out[] = {freq()};
  serialOut(out, 1);
  delay(100);
}
