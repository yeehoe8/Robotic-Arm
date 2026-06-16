#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String inputString = "";
boolean stringComplete = false;

// --- SMOOTHING VARIABLES ---
int currentPWM[6] = {324, 319, 131, 287, 299, 307}; // Starts at Home
int targetPWM[6]  = {324, 319, 131, 287, 299, 307}; 

unsigned long lastMoveTime = 0;
// We speed this up to 5ms because Cartesian paths generate many tiny micro-steps
const int stepDelay = 5; 

void setup() {
  // UPGRADED TO HIGH-SPEED USB COMMUNICATION
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 
  delay(10);

  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, currentPWM[i]);
  }
}

void loop() {
  // 1. READ INCOMING USB NATIVE DATA
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '<') {
      inputString = ""; 
    } else if (inChar == '>') {
      stringComplete = true; 
    } else {
      inputString += inChar;
    }
  }

  // 2. PARSE NEW TARGET
  if (stringComplete) {
    int tempPWM[6];
    int lastIndex = 0;
    int counter = 0;

    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.substring(i, i+1) == ",") {
        tempPWM[counter] = inputString.substring(lastIndex, i).toInt();
        lastIndex = i + 1;
        counter++;
      }
    }
    
    if (counter == 5) {
        tempPWM[5] = inputString.substring(lastIndex).toInt();
        for(int j = 0; j < 6; j++){
            targetPWM[j] = tempPWM[j];
        }
    }
    
    stringComplete = false;
    inputString = "";
  }

  // 3. SMOOTH MOVEMENT ENGINE
  if (millis() - lastMoveTime >= stepDelay) {
    lastMoveTime = millis();
    
    for (int i = 0; i < 6; i++) {
      if (currentPWM[i] != targetPWM[i]) {
        if (currentPWM[i] < targetPWM[i]) {
          currentPWM[i]++;
        } else {
          currentPWM[i]--;
        }
        pwm.setPWM(i, 0, currentPWM[i]);
      }
    }
  }
}
