#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- HARDWARE CONTROL PINS ---
const int irSensorPin = 3;     // IR Sensor input
const int conveyorPin = 4;     // Relay for Conveyor Belt
const int suckMotorPin = 6;    // Relay for Suction Pump

bool lastIrState = HIGH;      
bool waitingForPickup = false;
unsigned long lastPingTime = 0;

String inputString = "";
boolean stringComplete = false;

// Initial Home Position (Must match Python's assumed boot state)
int currentPWM[6] = {324, 319, 131, 287, 299, 307};

void setup() {
  // MASTER USB SERIAL LINK
  // High-speed 115200 baud is required to catch Python's 30FPS trajectory stream
  Serial.begin(115200); 
  
  pinMode(irSensorPin, INPUT_PULLUP); 
  pinMode(conveyorPin, OUTPUT);
  pinMode(suckMotorPin, OUTPUT);
  
  // Initial hardware state
  // NOTE: If using Active-LOW relays, swap HIGH and LOW here!
  digitalWrite(suckMotorPin, LOW); 
  digitalWrite(conveyorPin, HIGH); // Auto-start the conveyor belt
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 
  delay(10);
  
  // Lock arm rigidly to home position on boot
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, currentPWM[i]);
  }
}

void loop() {
  // ==========================================
  // 1. HARDWARE SENSOR LOGIC (CONVEYOR HALT)
  // ==========================================
  bool currentIrState = digitalRead(irSensorPin);
  
  // Trigger on falling edge (Object breaks the beam)
  if (currentIrState == LOW && lastIrState == HIGH) { 
    digitalWrite(conveyorPin, LOW); // Stop the conveyor
    waitingForPickup = true;        
    delay(50); // 50ms Debounce to prevent noisy sensor triggers
  }
  lastIrState = currentIrState;

  // Broadcast to Python that an object is ready over USB
  // We use a 1-second interval so we don't flood the serial buffer
  if (waitingForPickup) {
    if (millis() - lastPingTime > 1000) {
      Serial.println("OBJ_DETECTED"); 
      lastPingTime = millis();
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
  // 3. PARSE COMMANDS & APPLY INSTANTLY
  // ==========================================
  if (stringComplete) {
    if (inputString == "RESUME") {
      digitalWrite(conveyorPin, HIGH); // Restart conveyor
      waitingForPickup = false; 
    } 
    // Kept for backward compatibility/manual overrides
    else if (inputString == "SUCK_ON") {
      digitalWrite(suckMotorPin, HIGH); 
    }
    else if (inputString == "SUCK_OFF") {
      digitalWrite(suckMotorPin, LOW); 
    }
    else {
      // It's a 7-value array from Python: <q1,q2,q3,q4,q5,q6,sol>
      // We clear waitingForPickup in case of a manual override
      waitingForPickup = false; 
      
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
              currentPWM[j] = tempPWM[j];
              
              // WRITE INSTANTLY
              pwm.setPWM(j, 0, currentPWM[j]); 
          }

          // Apply Solenoid State instantly with the movement frame!
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
}
