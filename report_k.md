### Introduction
	
My teammate and I started our project by building the car and circuit in lab sessions, following the provided guidance. To make sure the car was reliable for long term, we took the extra step of custom-trimming all the wires so that they fit perfectly into their slots to ensure organized cable management and prevent loose connections.\
As we moved forward, we decided to make the car much smarter and more precise via adding three additional sensors, bringing the total to five tracking sensors, in addition to the front bumper sensor. While my teammate spent most of their time writing the software to control the car, I took the responsibility of the physical machine. This involved a lot of hands-on work, such as tightening the screws to keep the build sturdy, carefully adjusting the configuration of sensors (including position and sensitivity), and fixing any electrical issues that arose in the circuits. \
Additionally, we worked closely during the test phase to fine-tune the movement of the car. We spent a lot of time on measuring the time for car to spin and exactly how long it took to force the switch between different tasks. This helped us ensure that the demonstration could work fine.\
### Logic Design
### Overall Structure:
Instead of a basic “if-else” loop that blindly reacts to sensor, the code works via time and event. The code is divided into 18 parts for each specific tasks in the map. We constantly tracks the car’s progress using the “currentState” variable.\
First, the car remains locked in a “stopped” state after initialized. It must detect something to start the rest of the code. Then, we used “runMissionMode” for mission control. The variable looks up the current stage in several global arrays to determine its tasks. For instance, checking “STATE_ACTION” to see if it should track the line, spin in place or reverse for the last task.\ 
In line-tracking mode, we implemented that the car should uses the five sensors to ensure it stays at center. However, as it knows its “currentState”, it can override standard line-tracking mode. For example, if it knows it is at stage 5, it will force a right turn at junction instead of the normal way. This could ensure we prioritize the turn that the tasks need, preventing false turning.\
While transiting from different stages, the car continually check if the physical requirement were met to advance to the next stage (e.g. hitting a T-junction or detection of wall). Once the condition for transition is met, “currentState” increments and we load the rules for the next segment of the track for completing the task.\
Functions\
-	readBinaryStable(int pin): a crucial noise-filtering function that can ensure correct motion. As the sensors can misfire due to some light, the function is set to let the sensors “vote”. The function reads a digital pin three times and use a majority vote (e.g., if two readings are white and one is dark, the output evaluates to white) to ensure the sensor data is stable.\
-	clampPower(float p): This function is designed to ensure the motor power requests never exceed the 1.0 maximum or drop below 0.0, preventing mathematical errors in PWM calculation.\
-	scaledDurationMs(unsigned long baseMs): Multiplies time values by a global “MISSION_TIME_MULTIPLIER”, allowing the car to easily speed up or slow down the entire run’s timing parameter from a single variable. This can ensure our time to force transition between stages is consistent.\
-	setWheelPower(float leftPower, float rightPower): To compensate for this physical hardware asymmetry, we implemented a scaling factor (“RIGHT_PWM_MULTIPLIER” = 1.054). This function converts normalized power requests into offset PWM values, ensuring the chassis drives in a perfectly straight line.\
-	applyDirectionalCommandWithStability(…): We manage the H-bridge direction pins. It includes a “stability trick” that briefly sets both direction pins HIGH before changing them, preventing sudden voltage spikes that could restart the board. This prevents sudden voltage spikes to the motors, which we observed was causing brownouts or board resets.\
-	setForwardDirection() / setBackwardDirection(): Simple wrappers that configure the “DIR” pins for forward or reverse travel.\
-	applySteerCommandRaw(…): The base steering function. This depends on “cruisePower”, it decides whether to completely stop the inner wheel during a turn or run slightly in reverse for a tighter pivot.\
-	recordSteerChangeAndUpdateAdaptiveDebounce(...) & applySteerCommandDebounced(...): These form an advanced anti-oscillation system. They record the last 7 steering commands. If the car changes direction too rapidly within a 600ms window (indicating it is shaking back and forth), the system briefly locks the steering into a stable state to force the car to settle down. However, after the completion of the project, I found out that the reason for this is due to having one less capacitor to control the sudden change in pin.\
-	setSteerLeftForCruise(), setSteerRightForCruise(), setSteerStraightForCruise(): Higher-level wrappers that pass left/right/straight commands through the adaptive debouncer.
-	setEntryTurnLeftAggressive() / setEntryTurnRightAggressive(): Used specifically at track junctions. These apply maximum power in opposite directions (one wheel forward, one backward) to execute sharp, explicit pivots.\
-	getMissionCruisePower(int intervalState): A dynamic speed controller. It returns different power levels based on the current stage. For example, it slows down to “POWER_HALF” during the complex turns and the loop, but runs at “POWER_MAX” during the final backward stretch.\
-	transitionConditionMet(int state): The "eyes" of the tracking code. It checks specific sensor combinations required to beat the current stage. For example, Stage 16 checks if the bumper sensor detects the white wall (COND_BUMPER_ON_WHITE) to trigger the reversal.\
-	runLineTrackSimple(...): The core line-following algorithm. It reads the array of 5 tracking sensors. If the car wanders, it triggers the appropriate “setSteer...” function to correct it. However, since the car may overrun and the time limit does not allow the car to wander too long, we decided to do a force transistion. It accepts “forcedTurn” commands from the tracker to blindly turn at specific stage junctions regardless of sensor input. This can ensure that in specific time, it could transit to the next stage earlier for time shortening.\
-	runMissionMode(): The main loop of the project. It tracks how long the car has been in a specific stage, executes the assigned action (“ACT_LINE_TRACK”, “ACT_SPIN_360_RIGHT”, etc.), and checks “transitionConditionMet” to advance the code to the next task.
PWM\
For the PWM, we set a “POWER_FULL” variable, and it is 0.84 on default (which multiply by 255 and scales to 214). This default value ensures the car can beat the 35s limit, while retaining enough control to stay on the track. Furthermore, as introduced above, the left motor is slightly faster than right motor and their value is found that left motor is faster than right motor by 1.054. Hence, we set a “RIGHT_PWM_MULTIPLIER” variable as 1.054 to force the right motor spinning the same speed as left motor, and the following table shows the PWM values we take for each track.\

| Track / Segment      | Targeted Power (POWER_FULL Value) | Scaled PWM (Left / Right) |
|----------------------|-----------------------------------|---------------------------|
| Straight / Long runs | 0.84                              | 214 / 225                 |
| Turns                | 0.7                               | 178 / 188                 |
| Loop / S-Curves      | 0.63                              | 160 / 169                 |
| Final Reverse        | 0.948                             | 241 / 255                 | \

These values are set to ensure the car does not go off tracks easily, and we can take control of the car easily. Lastly, the Logic Flow Chart in next page:


```
graph TD
    %% Styling
    classDef startEnd fill:#f9f,stroke:#333,stroke-width:2px;
    classDef process fill:#bbf,stroke:#333,stroke-width:2px;
    classDef decision fill:#ff9,stroke:#333,stroke-width:2px;

    %% Nodes
    A([Power On / Reset]):::startEnd --> B[Initialize Pins & Read Sensors]:::process
    
    %% Arming Phase
    B --> C{Are all 3 center sensors on White?}:::decision
    C -- No --> B
    C -- Yes --> D[Arm Start Line & Save Bumper Baseline]:::process
    
    %% Trigger Phase
    D --> E{Is Bumper Toggled?}:::decision
    E -- No --> E
    E -- Yes --> F[Release Start Gate: currentState = 3]:::process
    
    %% Main FSM Loop
    F --> G[Enter: runMissionMode Loop]:::process
    G --> H{Check STATE_ACTION for currentState}:::decision
    
    %% Actions
    H -- ACT_LINE_TRACK --> I[runLineTrackSimple: Adjust steering based on sensors]:::process
    H -- ACT_SPIN_360 --> J[Spin: Apply opposite PWM to left/right motors]:::process
    H -- ACT_BACKWARD --> K[Reverse: Set DIR pins LOW, apply PWM]:::process
    H -- ACT_STOP --> L[Stop: Set PWM to 0]:::process
    
    %% Condition Checking
    I --> M
    J --> M
    K --> M
    L --> M
    
    M{Is transitionConditionMet for current stage?}:::decision
    M -- "No (Keep running current action)" --> G
    M -- "Yes (Event triggered)" --> N[Increment: currentState + 1]:::process
    
    %% End Check
    N --> O{Is currentState >= 18?}:::decision
    O -- No --> G
    O -- Yes --> P([Trial Finished: System Halt]):::startEnd
```
  
### Debugging Report
-	Bug 1: Implementation and Sensor Integration
While the initial logic for the code appeared sound, several implementation errors occurred due to limited experience with C++. Although the original code functioned initially, its accuracy was found to be insufficient during testing, requiring the installation of a new sensor. However, the code was not well-organized, resulting difficulty during the integration with new sensor’s signal, which required additional logic. This eventually caused the implementation to fail entirely. To resolve the issue efficiently, we decided to rewrite the code from scratch. This re-implementation successfully addressed the underlying logic and integration problems, restoring the system to function fully.\
-	Bug 2: Hardware Troubleshooting
During a follow-up test with the updated code, the motors initially functioned correctly. However, after approximately an hour and a subsequent code update, the right motor failed to rotate. Even after flashing the original, which the code was known as working, the motor remained stationary. We initially investigated potential wiring issues and suspected poor connections between critical components like the inverter, H-bridge and Nano Board, though these were eventually ruled out. As one motor continued to function while the other did not, the power supply was also excluded as a possible cause. The investigation then shifted to a potential hardware failure, specifically a burned H-bridge or inverter. Following a debugging session, the H-bridge was identified as the source of the problem. To ensure an accurate troubleshoot, I brought the car to Laboratory 1 session to seek help and consulted by a teaching assistant (TA). The TA confirmed the hardware failure and replaced the H-bridge and provide extra components in case components fail again in the future. Changing the H-Bridge resolved the problem of right motor.\
### Results and Conclusion
Overall, the project was a success, and I believe my primary contribution lay in the physical assembly and hardware optimization of the car. However, as a newcomer to programming, I identified software implementation and systematic debugging as areas for further development. While many of the software issues were identified by my teammate, participating in that process provided me with a valuable learning opportunity.\
One of the most successful aspects of our code was the turning logic. The robot maintained a centered position throughout most of the trial, ensuring high operational stability. However, the complexity of our code increased the time required for troubleshooting as it took us more time to discover what part of the code must be changed. A significant challenge was an unexpected failure of the H-bridge. After a debugging process, we identified the fault to the hardware component and coordinated with the Teaching Assistant for a replacement.\
If I were to start over, I would prioritize developing a comprehensive logic flowchart prior to any coding. Mapping the logic visually would have prevented the "off-track" errors encountered in early drafts. With additional time, I would implement a PID-based control system. By using our five-sensor array to generate a continuous "error signal", we could apply mathematical operations to tune the motor speeds and enhance stability compared to our current discrete logic. Finally, our only bad decision was not to double check the circuit upon discussion on circuit. We later discovered that we missed a capacitor, this would lead to minor electrical instability during runs. This project served as an excellent introduction to the integration of hardware and software, the core of robotics, and has provided a clear roadmap for my future improvement in embedded systems.
