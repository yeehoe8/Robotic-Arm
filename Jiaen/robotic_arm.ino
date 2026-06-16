#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h> 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SoftwareSerial Bluetooth(2, 7); // RX (Pin 2) to HC-05 TX, TX (Pin 7) to HC-05 RX

// --- HARDWARE PINS ---
const int irSensorPin = 3;    // IR Sensor OUT pin
const int conveyorPin = 4;     // Relay control for the conveyor belt
const int suckMotorPin = 6;    // Relay control for the suction motor
bool lastIrState = HIGH;       // Assumes IR sensor is Active LOW (triggers on LOW)

String inputString = "";
boolean stringComplete = false;

// --- SMOOTHING VARIABLES ---
int currentPWM[6] = {324, 319, 131, 287, 299, 307}; // Starts at Home
int targetPWM[6]  = {324, 319, 131, 287, 299, 307}; 

unsigned long lastMoveTime = 0;
// Speed up to 5ms because Cartesian paths generate many tiny micro-steps
const int stepDelay = 5; 

void setup() {
  Serial.begin(9600);    // Standard Serial Monitor for debugging via USB
  Bluetooth.begin(9600); // Default baud rate for HC-05 module
  
  pinMode(irSensorPin, INPUT);
  pinMode(conveyorPin, OUTPUT);
  pinMode(suckMotorPin, OUTPUT);
  
  // Ensure suction is off at boot and auto-start the conveyor
  digitalWrite(suckMotorPin, LOW); 
  digitalWrite(conveyorPin, HIGH); 
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); 
  delay(10);

  // Lock arm to the starting home position
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, currentPWM[i]);
  }
  
  Serial.println("System Ready. Conveyor Running...");
}

void loop() {
  // ==========================================
  // 1. HARDWARE SENSOR LOGIC
  // ==========================================
  bool currentIrState = digitalRead(irSensorPin);
  
  // Object detected (transition from HIGH -> LOW)
  if (currentIrState == LOW && lastIrState == HIGH) { 
    digitalWrite(conveyorPin, LOW); // Stop the conveyor immediately
    Bluetooth.println("OBJ_DETECTED"); // Send trigger flag to Python/MATLAB
    Serial.println("Object Detected! Conveyor Halted. Awaiting instructions...");
    delay(50); // Debounce to prevent multiple triggers
  }
  lastIrState = currentIrState;

  // ==========================================
  // 2. READ INCOMING BLUETOOTH DATA
  // ==========================================
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

  // ==========================================
  // 3. PARSE NEW TARGET OR COMMAND
  // ==========================================
  if (stringComplete) {
    
    // Command A: Resume Conveyor
    if (inputString == "RESUME") {
      digitalWrite(conveyorPin, HIGH); 
      Serial.println("Task Complete. Resuming Conveyor.");
    } 
    // Command B: Turn Suction ON
    else if (inputString == "SUCK_ON") {
      digitalWrite(suckMotorPin, HIGH);
      Serial.println("Suction Motor: ON");
    }
    // Command C: Turn Suction OFF
    else if (inputString == "SUCK_OFF") {
      digitalWrite(suckMotorPin, LOW);
      Serial.println("Suction Motor: OFF");
    }
    // Command D: Parse the 6 PWM joint targets
    else {
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
    }
    
    stringComplete = false;
    inputString = "";
  }

  // ==========================================
  // 4. SMOOTH MOVEMENT ENGINE
  // ==========================================
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
