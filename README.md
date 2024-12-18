# Automated Object Sorting System Using Capacitive and Inductive Sensors

**Description:**  
This project aims to automate the sorting of objects using capacitive and inductive sensors, a 6 DOF robotic arm, and an Arduino Uno. The system distinguishes between materials (plastic and paper) using sensors, and the sorted data is stored in a local database on an SD card module.

---

**Hardware Specifications:**  
Below are the required components for the project:

1. **Inductive Proximity Sensor**: LJ12A3-4-Z/BY  
   [Interfacing Guide](https://electropeak.com/learn/interfacing-inductive-proximity-sensor-lj12a3-4-z-3-wire-with-arduino/)  
2. **Capacitive Sensor**: ProtoCentral FDC1004  
   [Library Documentation](https://protocentral.github.io/ProtoCentral_fdc1004_breakout/)  
3. **Micro SD Card Module**:  
   [Tutorial](https://lastminuteengineers.com/arduino-micro-sd-card-module-tutorial/)  
4. **6 DOF Robotic Arm**  
5. **Arduino Uno**  
6. **9 Volt Batteries (2)**  
7. **Aluminium Foil Paper (2)**  

---

**Required Libraries:**  
The following libraries must be installed to implement the project:  

1. [ProtoCentral FDC1004 Capacitive Sensor Library](https://www.arduino.cc/reference/en/libraries/protocentral-fdc1004-capacitive-sensor-library/)  
   - Include: `<Protocentral_FDC1004.h>`  
2. [VarSpeedServo Library](https://github.com/netlabtoolkit/VarSpeedServo)  
3. Standard Arduino Libraries:  
   - `<SPI.h>`  
   - `<SD.h>`  
   - `<Wire.h>`  

---

**Calibration Process:**  

To distinguish between plastic and paper:  
1. When the serial monitor displays "Start calibrating" at the beginning, place a **plastic object** on the capacitive bench first.  
2. After 5 seconds, replace the plastic object with **paper**.  
3. Calibration is now complete.  
4. **Note**: Calibration must be performed each time the system starts.  

---

**Robotic Arm Configuration:**  

To control the 6 DOF robotic arm:  
- Refer to this [guide](https://www.instructables.com/Using-Arduino-Uno-for-XYZ-Positioning-of-6-DOF-Rob/).  
- Modify the servo controlling parameters according to your box setup.  
- Configure all servo positions to match the desired destinations for sorted objects.

---

**Data Storage:**  
All sorting data will be saved locally using a **Micro SD Card Module**.  
- Ensure the SD card is formatted correctly.  
- Verify successful data logging using the Arduino Serial Monitor.

---
