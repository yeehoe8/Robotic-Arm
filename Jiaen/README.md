# The Operational Sequence
## 1. System Boot & Standby

The Arduino powers up, instantly locks the arm to the Home [0,0,0,0,0,0] position, ensures the suction motor is OFF, and turns the conveyor belt ON.

You click "Connect" in the UI. The dashboard links to Python, sets the UI state to WAITING_FOR_OBJ, and begins silently asking the Arduino for updates every 800ms.

## 2. The Trigger

An object travels down the belt and breaks the IR beam.

The Arduino immediately cuts power to the conveyor belt (halting the object in place) and shouts "OBJ_DETECTED" over Bluetooth.

The UI catches this message, stops polling, and unlocks the "Step A" Pick buttons.

## 3. The Pick (executePick)

You click the button. The UI calculates the L-Shape trajectory, sweeping the arm horizontally over the object, then plunging perfectly straight down to the pickZ depth.

The UI sends <SUCK_ON>. The Arduino fires the suction relay.

The system pauses for 600ms to let the vacuum seal form against the object, then unlocks "Step B".

## 4. The Place (executePlace)

The arm swings back to the Home position to clear the pickup zone.

The arm calculates a new L-Shape trajectory to hover over the Place coordinate, then plunges down.

The UI sends <SUCK_OFF>. The Arduino cuts the vacuum. The system waits 500ms for gravity to drop the object.

The arm returns to Home. The UI sends <RESUME>, starting the conveyor belt to fetch the next object, and the whole cycle resets.

# ⚠️ Potential Hardware & Kinematic Issues to Watch
Everything is coded correctly, but real-world physics might throw a few curveballs at you here.

##  1. The "Home Swing" Smash Hazard
When your arm finishes the pick, it calls moveArmToHome(). That function does not use your L-Shape IK; it just interpolates the raw joint angles to zero. This means the arm will swing in a curved arc to get home. If your object is inside a bin or surrounded by walls, that curved swing might smash the object into the side of the bin before it clears the lip.

Fix: Ideally, you want to pull straight up (vertically) before moving back to Home.

## 2. Relay Polarity Inversion
You are using digitalWrite(conveyorPin, HIGH) to turn the conveyor ON. Keep in mind that 90% of cheap Arduino relay modules are Active-LOW. This means sending HIGH turns the motor off, and sending LOW turns it on. If your conveyor doesn't start, or your vacuum runs backwards, you just need to swap the HIGH and LOW commands in your .ino file.

## 3. Bluetooth Bandwidth Saturation
You are running the HC-05 at its default 9600 baud rate. During trajectory playback, you are blasting a 26-byte string to the Arduino every 33 milliseconds. That equates to about 780 bytes per second, which is pushing the absolute maximum physical limit of a 9600 baud connection.

Symptom: If the arm starts to jitter or randomly pause during the L-Shape movement, the Bluetooth buffer is choking.

Fix: If this happens, change setTimeout(playNextFrame, 33) to 45 in your HTML to slightly slow down the data stream.

Would you like me to rewrite the moveArmToHome() function in your HTML so that it utilizes the IK engine to lift perfectly straight up before traversing, eliminating the smash hazard?
