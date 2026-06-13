#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SoftwareSerial Bluetooth(2, 7); // RX, TX

String inputString = "";
boolean stringComplete = false;

// --- COORDINATED KINEMATICS VARIABLES ---
int currentPWM[6] = {324, 319, 131, 287, 299, 307}; // Current physical position
int targetPWM[6]  = {324, 319, 131, 287, 299, 307}; // The goal position
int startPWM[6]   = {324, 319, 131, 287, 299, 307}; // Snapshot of where the move began

long maxDelta = 0;     // The maximum number of steps required by the longest move
long moveProgress = 0; // Tracks the current step out of maxDelta

unsigned long lastMoveTime = 0;
// SPEED CONTROL: 5ms to 10ms is standard for smooth coordinated motion.
const int stepDelay = 8; 

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600); 
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 
  delay(10);

  // Initialize servos at home, applying the Phase-Shift offsets
  for (int i = 0; i < 6; i++) {
    int phaseOffset = i * 400; // Staggers the pulse start times to prevent brownouts
    pwm.setPWM(i, phaseOffset, phaseOffset + currentPWM[i]);
  }
}

void loop() {
  // 1. READ INCOMING BLUETOOTH DATA (Non-blocking)
  while (Bluetooth.available()) {
    char inChar = (char)Bluetooth.read();
    
    if (inChar == '<') {
      inputString = ""; 
    } else if (inChar == '>') {
      stringComplete = true; 
    } else {
      inputString += inChar;
    }
  }

  // 2. PARSE AND PREPARE COORDINATED MOTION
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

        // Snapshot the current position and calculate the new max distance
        maxDelta = 0;
        for(int j = 0; j < 6; j++) {
            startPWM[j] = currentPWM[j];
            targetPWM[j] = tempPWM[j];
            
            long delta = abs(targetPWM[j] - startPWM[j]);
            if (delta > maxDelta) {
                maxDelta = delta;
            }
        }
        
        // Reset the progress counter for the new move
        moveProgress = 0;
    }
    
    stringComplete = false;
    inputString = "";
  }

  // 3. COORDINATED LINEAR INTERPOLATION ENGINE
  if (millis() - lastMoveTime >= stepDelay) {
    lastMoveTime = millis();
    
    // If we haven't reached the end of the move
    if (moveProgress < maxDelta) {
      moveProgress++; // Advance 1 global time step
      
      for (int i = 0; i < 6; i++) {
        // Mathematically scale every joint to arrive perfectly on the last step
        currentPWM[i] = startPWM[i] + ((targetPWM[i] - startPWM[i]) * moveProgress) / maxDelta;
        
        // --- THE PHASE-SHIFT SURGE PROTECTOR ---
        // Instead of starting all pulses at tick 0, we space them 400 ticks apart.
        // This ensures the current draw is distributed evenly across the duty cycle.
        int phaseOffset = i * 400; 
        pwm.setPWM(i, phaseOffset, phaseOffset + currentPWM[i]);
      }
    }
  }
}