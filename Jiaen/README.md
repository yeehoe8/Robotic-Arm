### ⚙️ The Operational Sequence

**1. Boot & Idle (Waiting for Object)**

* The Arduino boots up, locks the servos to the Home position `[0,0,0,0,0,0]`, turns the suction OFF, and turns the conveyor ON.
* You connect via the web UI. The UI state changes to `WAITING_FOR_OBJ`.
* The Python server quietly pings the Arduino over Bluetooth every 800ms asking, "Do you see anything?"

**2. Detection & Hault**

* An object breaks the IR beam.
* The Arduino instantly cuts power to the conveyor relay, freezing the object in place, and transmits `"OBJ_DETECTED"`.
* The UI receives this, stops polling to free up Bluetooth bandwidth, alerts you, and enables the **"Go to Coordinate & SUCK ON"** button.

**3. The Pick Sequence**

* You click the Pick button.
* Python calculates a smooth, point-to-point arc to the XYZ coordinate.
* The arm swings down to the object.
* The UI sends the `<SUCK_ON>` command. The Arduino triggers the suction relay.
* The system pauses for 600ms to allow the vacuum pressure to build, ensuring a solid grip.
* The UI state updates to `HOLDING_OBJ` and unlocks the Place button.

**4. The Place & Reset Sequence**

* You click the Place button.
* The arm sweeps in a smooth arc back to the exact Home position `[0,0,0,0,0,0]` to clear the pickup zone.
* Python calculates the next arc to the drop-off XYZ coordinate.
* The arm swings to the drop-off zone.
* The UI sends `<SUCK_OFF>`. The vacuum cuts out.
* The system waits 500ms for the object to fall.
* The arm swings back to Home.
* Finally, the UI sends `<RESUME>`. The Arduino fires the conveyor belt relay back up, the UI state resets to `WAITING_FOR_OBJ`, and the robot is ready for the next object.

---

### ⚠️ Hardware Traps to Be Aware Of

Because you are moving physical objects in the real world now, kinematics and electronics start to overlap. Watch out for these four things during your physical testing:

**1. The "Home Swing" Smash Hazard**
During the Place sequence, the first thing your code does is `moveArmToHome()`. Because this uses point-to-point joint interpolation, all motors move to `0°` at the exact same time. If your object is sitting inside a shallow box or has tall objects around it, the arm will drag the object in a horizontal arc *through* those obstacles before it gains enough altitude to clear them.

* *How to fix it later:* If you find the arm knocking things over, you will need to add an intermediate "safe lift" coordinate (e.g., move Z up by 100mm) before executing the `moveArmToHome()` function.

**2. Relay Polarity Inversion**
In your Arduino code, you use `digitalWrite(conveyorPin, HIGH)` to turn the conveyor on, and `LOW` to stop it.

* *The Trap:* 90% of cheap DIY relay modules (the ones with the blue or black boxes) are **Active-LOW**. That means sending `LOW` actually turns the motor ON, and `HIGH` turns it OFF. If your conveyor starts running backwards, or your suction motor turns on when it shouldn't, simply swap the `HIGH` and `LOW` commands in your `.ino` file.

**3. IR Sensor Ambient Light Interference**
IR sensors are highly susceptible to ambient room lighting, especially sunlight or fluorescent overhead tubes.

* *The Trap:* If your room lighting changes, the sensor might trigger a false `OBJ_DETECTED` signal. Use the small potentiometer on the back of the IR sensor module to tune the sensitivity directly under the lighting conditions you plan to demo the robot in.

**4. Bluetooth Buffer Limits at 9600 Baud**
You are sending a heavy stream of trajectory frames at 30 frames per second over a 9600 baud Bluetooth connection.

* *The Trap:* The HC-05 module is robust, but 9600 baud has a hard data-rate limit. If you notice the arm briefly stuttering, twitching, or pausing mid-arc, it means the Bluetooth buffer is choking on the data.
* *The Fix:* If this happens, open your `index.html` and change `setTimeout(playNextFrame, 33)` to `45` or `50`. This slightly slows down the playback speed, giving the Bluetooth module time to digest the data packets.
