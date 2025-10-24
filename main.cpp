#include <Arduino.h>

// --- Motor pins ---
const int ENA_X = 23;
const int DIR_X = 22;
const int PUL_X = 21;
const int CH_X  = 0;

// --- Camera center ---
const int targetX = 320;

// --- PID parameters ---
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.1;

// --- PID state ---
float errorX = 0, prevErrorX = 0, integralX = 0;

// --- Motion limits ---
const int MAX_FREQ = 3000;   // <-- safe upper limit (Hz)
const int MIN_FREQ = 30;     // <-- minimum step rate
int currentFreq = 0;          // for smooth ramping
const int rampRate = 100;     // Hz per update step


int lastTime = 0;
bool dirx = (random(2) > 0) ? HIGH : LOW;

String input;

void setup() {
  Serial.begin(115200);
  pinMode(ENA_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  digitalWrite(ENA_X, LOW);

  ledcSetup(CH_X, 0, 8);
  ledcAttachPin(PUL_X, CH_X);
  lastTime = millis();
}

void loop() {

  
  
  if (Serial.available()) {
    // delay(5);
    // return;
    lastTime = millis();
    input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      int x = input.substring(0, commaIndex).toInt();

      // --- PID compute ---
      errorX = -(targetX - x);
      integralX += errorX;
      float derivativeX = errorX - prevErrorX;
      float outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
      prevErrorX = errorX;

      // --- Direction ---
      digitalWrite(DIR_X, (outputX > 0) ? HIGH : LOW);

      // --- Desired frequency ---
      int targetFreq = constrain(abs(outputX) * 8, 0, MAX_FREQ);
      if (targetFreq < MIN_FREQ) targetFreq = 0;

      // --- Smooth ramp to new frequency ---
      if (currentFreq < targetFreq)
        currentFreq += rampRate;
      else if (currentFreq > targetFreq)
        currentFreq -= rampRate;

      currentFreq = constrain(currentFreq, 0, MAX_FREQ);

      // --- Apply to PWM ---
      ledcWriteTone(CH_X, currentFreq);
      ledcWrite(CH_X, 128); // 50% duty
    }
  }
  else
  {
    // delay(5);
    // return;
    if(millis()-lastTime>2000)
    {
      delay(1);
      // --- Direction ---

      if(millis()-lastTime>20000)
      {
        dirx = !dirx;
        lastTime = millis()-3000;
      }

      
      digitalWrite(DIR_X, dirx);
      // --- Desired frequency ---
      int targetFreq = constrain(abs(180) * 8, 0, MAX_FREQ);
      if (targetFreq < MIN_FREQ) targetFreq = 0;

      // --- Smooth ramp to new frequency ---
      if (currentFreq < targetFreq)
        currentFreq += rampRate;
      else if (currentFreq > targetFreq)
        currentFreq -= rampRate;

      currentFreq = constrain(currentFreq, 0, MAX_FREQ);

      // --- Apply to PWM ---
      ledcWriteTone(CH_X, 1.5*currentFreq);
      ledcWrite(CH_X, 128); // 50% duty
    }
  }


}



