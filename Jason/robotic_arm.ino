#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- HARDWARE CONTROL PINS ---
const int irSensorPin = 13;    // IR Sensor input
const int conveyorPin = 4;     // Relay for Conveyor Belt
const int suckMotorPin = 6;    // Relay for Suction Pump

bool waitingForPickup = false;
unsigned long lastPingTime = 0;

// --- SETTLING & VERIFICATION FILTER ---
unsigned long verifyStartTime = 0;
bool isVerifying = false;

// E-STOP SAFETY LOCK
bool eStopActive = false; 

String inputString = "";
boolean stringComplete = false;

// --- EDGE-SIDE SMOOTHING INTERPOLATOR ---
float currentSmoothedPWM[6] = {324, 319, 131, 287, 299, 307};
int targetPWM[6] = {324, 319, 131, 287, 299, 307};
float smoothingFactor = 0.4; // 0.4 at 50Hz provides excellent shock-absorption without lagging behind

unsigned long lastInterpolateTime = 0;
const unsigned long interpolateInterval = 20; // 50Hz update rate (20ms)

void setup() {
  // High-speed 115200 baud is required to catch Python's 30FPS trajectory stream
  Serial.begin(115200); 
  
  // Using INPUT_PULLUP prevents the "Phantom Trigger" bug!
  pinMode(irSensorPin, INPUT_PULLUP); 
  pinMode(conveyorPin, OUTPUT);
  pinMode(suckMotorPin, OUTPUT);
  
  digitalWrite(suckMotorPin, LOW); 
  digitalWrite(conveyorPin, HIGH); // Auto-start the conveyor belt
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 
  delay(10);
  
  // Lock arm rigidly to home position on boot
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, targetPWM[i]);
  }
}

void loop() {
  // ==========================================
  // 1. HARDWARE SENSOR LOGIC (INSTANT HALT + 500ms SETTLE)
  // ==========================================
  if (!eStopActive) {
      bool currentIrState = digitalRead(irSensorPin);
      
      // Only process sensor if we are currently running the conveyor
      if (!waitingForPickup) {
          if (currentIrState == LOW) { // Object is blocking the sensor
              // 1. START TIMER: Do NOT stop the belt yet!
              if (!isVerifying) {
                  isVerifying = true;
                  verifyStartTime = millis();
              } 
              // 2. CONTINUOUS FILTER: If it stays for 500ms continuously, THEN stop the belt
              else if (millis() - verifyStartTime >= 200) {
                  digitalWrite(conveyorPin, LOW); // Stop the conveyor NOW
                  waitingForPickup = true;        // Flag ready for Python
                  isVerifying = false;            // Reset the tracker
              }
          } else {
              // 3. RECOVERY: If it was a glitch (object left before 500ms), just reset timer
              isVerifying = false;
          }
      }
    
      // Broadcast to Python that an object is ready over USB
      if (waitingForPickup) {
        if (millis() - lastPingTime > 1000) {
          Serial.println("OBJ_DETECTED"); 
          lastPingTime = millis();
        }
      }
  }

  // ==========================================
  // 2. READ INCOMING USB DATA STREAM
  // ==========================================
  while (Serial.available() && !stringComplete) {
    char inChar = (char)Serial.read();
    if (inChar == '<') {
      inputString = "";
    } else if (inChar == '>') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // ==========================================
  // 3. PARSE COMMANDS & ENFORCE SAFETY
  // ==========================================
  if (stringComplete) {
      
    // --- EMERGENCY STOP TRIGGER ---
    if (inputString == "ESTOP") {
      eStopActive = true;
      digitalWrite(conveyorPin, LOW);
      digitalWrite(suckMotorPin, LOW);
      waitingForPickup = false;
      isVerifying = false;
    }
    // --- EMERGENCY STOP RESET ---
    else if (inputString == "RESET") {
      eStopActive = false;
    }
    // --- STANDARD SYSTEM RESUME ---
    else if (inputString == "RESUME" && !eStopActive) {
      digitalWrite(conveyorPin, HIGH); // Restart conveyor
      waitingForPickup = false; 
      isVerifying = false;
    } 
    // --- TRAJECTORY FRAME EXECUTION ---
    else if (!eStopActive) {
      // It's a 7-value array from Python: <q1,q2,q3,q4,q5,q6,sol>
      waitingForPickup = false; 
      isVerifying = false;
      
      int tempPWM[6];
      int solState = 0;
      int lastIndex = 0;
      int counter = 0;

      // Extract comma-separated values
      for (int i = 0; i < inputString.length(); i++) {
        if (inputString.substring(i, i+1) == ",") {
          if (counter < 6) {
            tempPWM[counter] = inputString.substring(lastIndex, i).toInt();
          }
          lastIndex = i + 1;
          counter++;
        }
      }
      
      // If we successfully caught 6 commas (meaning 6 joints + 1 solenoid state)
      if (counter == 6) {
          solState = inputString.substring(lastIndex).toInt();
          
          for(int j = 0; j < 6; j++){
              targetPWM[j] = tempPWM[j]; // Set the tracking target, let the interpolator do the rest!
          }

          // Apply Solenoid State instantly with the movement frame
          if (solState == 1) {
              digitalWrite(suckMotorPin, HIGH);
          } else {
              digitalWrite(suckMotorPin, LOW);
          }
      }
    }
    
    stringComplete = false;
    inputString = "";
  }

  // ==========================================
  // 4. TIMED HARDWARE LOW-PASS FILTER
  // ==========================================
  // Evaluate exactly every 20ms to match 50Hz physical servo limits
  if (millis() - lastInterpolateTime >= interpolateInterval) {
      lastInterpolateTime = millis();
      
      for(int j = 0; j < 6; j++) {
          // Only update if there is a meaningful difference
          if (abs(currentSmoothedPWM[j] - targetPWM[j]) > 0.1) {
              currentSmoothedPWM[j] += (targetPWM[j] - currentSmoothedPWM[j]) * smoothingFactor;
              pwm.setPWM(j, 0, (int)currentSmoothedPWM[j]);
          }
      }
  }
}
