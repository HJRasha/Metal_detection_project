Need these components:
1. Inductive Proximity Sensor LJ12A3-4-Z/BY (https://electropeak.com/learn/interfacing-inductive-proximity-sensor-lj12a3-4-z-3-wire-with-arduino/)
2. ProtoCentral FDC1004 capacitance sensor (https://protocentral.github.io/ProtoCentral_fdc1004_breakout/)
3. Micro SD card module (https://lastminuteengineers.com/arduino-micro-sd-card-module-tutorial/)
4. 6 DOF Robotic Arm
5. Arduino Uno
6. 9 volt battery (2)
7. Aluminium foil paper (2)

Need to install following libraries:
1. ProtoCentral FDC1004 Capacitive Sensor Library<Protocentral_FDC1004.h>(https://www.arduino.cc/reference/en/libraries/protocentral-fdc1004-capacitive-sensor-library/)
2. VarSpeedServo library (https://github.com/netlabtoolkit/VarSpeedServo)
3. include the following libraries: <SPI.h>,<SD.h>,<Wire.h>  

For distinguish the plastic and the paper, when serial monitor shows start calibrating at the very begining, place plastic object at the capacitive bench first after 5 sec again place a paper. Then the calibration is done. Each time this is needed to be done.

For controlling the 6DOF robotic arm check this article (https://www.instructables.com/Using-Arduino-Uno-for-XYZ-Positioning-of-6-DOF-Rob/), And according to you box setup you need to modify the servo controlling parameters.Configure all the servo positions according to you destination.


All the data will be saved at the local database in a micro SD card module.