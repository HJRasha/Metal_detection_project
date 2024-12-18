#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include <VarSpeedServo.h>


#define indSensor 2  // Digital pin connected to the sensor output
#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 0                          // channel to be read
#define MEASURMENT 0                       // measurment channel

File dataFile;
FDC1004 FDC;

float Cal_Cap;

// change this to match your SD shield or module;
const int chipSelect = 10;
int capdac = 0;
float plasticBaseline;      // Baseline capacitance value for plastic
float paperBaseline;        // Baseline capacitance value for paper
float threshold = 50.00;       // Adjust this threshold based on your testing
#define CHANNEL 0                          // channel to be read

//robotic_arm
// Define servo motors
#define baseServo 4
#define shlServo 5
#define elbServo 6
#define wriServo 7
#define handServo 8
#define griServo 9

//Arm Dimension (mm)
#define BASE_HGT 90 //base height
#define HUMERUS 100 //shoulder-to-elbow "bone"
#define ULNA 135 //elbow-to-wrist "bone"
#define GRIPPER 200 //gripper (incl.heavy duty wrist rotate mechanism) length "
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5)) //float to long conversion

//pre-calculation
float hum_sq = HUMERUS * HUMERUS;
float uln_sq = ULNA * ULNA;
int servoSPeed = 10;

//ServoShield servos; //ServoShield object
VarSpeedServo servo1, servo2, servo3, servo4, servo5, servo6;

int loopCounter = 0;
int pulseWidth = 6.6;
int microsecondsToDegrees;

uint16_t line = 1;

void setup() {
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  pinMode(indSensor, INPUT);

  Wire.begin();        //i2c begin

  // Attach servo motors to pins
  servo1.attach( baseServo, 544, 2400 );
  servo2.attach( shlServo, 544, 2400 );
  servo3.attach( elbServo, 544, 2400 );
  servo4.attach( wriServo, 544, 2400 );
  servo5.attach( handServo, 544, 2400 );
  servo6.attach( griServo, 544, 2400 );
  delay( 5500 );
  //servos.start(); //Start the servo shield
  servo_park();
  delay(4000);

  Serial.println("Start Calibrating the Capacitive Sensor");

  calibrateBaselines(); // after calibrating the capacitance value, comment this line

  home_position ();
}

void loop() {
  //Return to home position
  home_position();
  delay(1000);
  
  //pick the object from conveyer belt
  servo_pick();
  delay(1000);

  //park to the detection zone
  servo_park();
  delay(1000);

  

  //Create a file to store
  dataFile = SD.open("objectinfo.txt", FILE_WRITE);

  // Read the digital input from the sensor
  int inductiveValue = digitalRead(indSensor);

  // Measure the current capacitance value
  ret_capacitance();

  // Determine metal presence based on sensor output
  if (inductiveValue == HIGH) {
    // Metal detected
    Serial.println("Metal Detected");
    throw_redbox ();
    if (dataFile)
    {
      dataFile.print(line);
      dataFile.print(": Metal obeject has entered red box");
      dataFile.print(line++);
    }
    else
      Serial.println("error opening DHT11Log.txt");

  }
  else {
    // Determine material based on threshold and respective baseline
    if (abs(Cal_Cap - plasticBaseline) < threshold) {
      // Plastic detected
      throw_bluebox ();
      Serial.println("Plastic Detected");
      if (dataFile)
      {
        dataFile.print(line);
        dataFile.print(": plastic obeject has entered blue box");
        dataFile.print(line++);
      }
      else
        Serial.println("error opening DHT11Log.txt");

    } else if (abs(Cal_Cap - paperBaseline) < threshold) {
      // Paper detected
      throw_greenbox ();
      Serial.println("Paper Detected");
      if (dataFile)
      {
        dataFile.print(line);
        dataFile.print(": Paper obeject has entered green box");
        dataFile.print(line++);
      }
      else
        Serial.println("error opening DHT11Log.txt");
    } else {
      // other material detected
      throw_yellowbox ();
      Serial.println("other type of Material Detected");
      if (dataFile)
      {
        dataFile.print(line);
        dataFile.print(": other obeject has entered blue box");
        dataFile.print(line++);
      }
      else
        Serial.println("error opening DHT11Log.txt");
    }
  }
  //delay(500); // Adjust delay based on your application's requirements
}

void calibrateBaselines() {
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);

  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! FDC.readMeasurement(MEASURMENT, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    Serial.print((((float)capacitance / 1000)), 4);
    Serial.print("  pf, ");

    if (msb > UPPER_BOUND)               // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }
    Serial.println("Calibrating for Plastic... Please place plastic on the sensor.");
    delay(5000);
    plasticBaseline = capacitance;
    Serial.print("Plastic Baseline: ");
    Serial.println(plasticBaseline);

    Serial.println("Calibrating for Paper... Please place paper on the sensor.");
    delay(5000);
    paperBaseline = capacitance;
    Serial.print("Paper Baseline: ");
    Serial.println(paperBaseline);

    Serial.println("Calibration Complete.");
  }
}

void servo_park()
{
  //servos.setposition( BAS_SERVO, 1500 );
  servo1.write(90, 10);
  //servos.setposition( SHL_SERVO, 2100 );
  servo2.write(90, 10);
  //servos.setposition( ELB_SERVO, 2100 );
  servo3.write(90, 10);
  //servos.setposition( WRI_SERVO, 1800 );
  servo4.write(90, 10);
  //servos.setposition( WRO_SERVO, 600 );
  servo5.write(90, 10);
  //servos.setposition( GRI_SERVO, 900 );
  servo6.write(80, 10);
  delay(500);
}

void servo_pick()
{
  //servos.setposition( BAS_SERVO, 1500 );
  servo1.write(32, 10);
  //servos.setposition( SHL_SERVO, 2100 );
  servo2.write(87, 10);
  //servos.setposition( ELB_SERVO, 2100 );
  servo3.write(122, 10);
  //servos.setposition( WRI_SERVO, 1800 );
  servo4.write(20, 10);
  //servos.setposition( WRO_SERVO, 600 );
  servo5.write(12, 10);
  //servos.setposition( GRI_SERVO, 900 );
  servo6.write(30, 10);
  delay(500);
}

void home_position () {
  //servos.setposition( BAS_SERVO, 1500 ); //change according to your box position
  servo1.write(90, 10);
  //servos.setposition( SHL_SERVO, 2100 ); //change according to your box position
  servo2.write(90, 10);
  //servos.setposition( ELB_SERVO, 2100 ); //change according to your box position
  servo3.write(90, 10);
  //servos.setposition( WRI_SERVO, 1800 ); //change according to your box position
  servo4.write(90, 10);
  //servos.setposition( WRO_SERVO, 600 ); //change according to your box position
  servo5.write(90, 10);
  //servos.setposition( GRI_SERVO, 900 ); //change according to your box position
  servo6.write(90, 10);
  delay(500);
}


void throw_redbox () {

  //servos.setposition( BAS_SERVO, 1500 ); //change according to your box position
  servo1.write(15, 10);
  //servos.setposition( SHL_SERVO, 2100 ); //change according to your box position
  servo2.write(15, 10);
  //servos.setposition( ELB_SERVO, 2100 ); //change according to your box position
  servo3.write(165, 10);
  //servos.setposition( WRI_SERVO, 1800 ); //change according to your box position
  servo4.write(180, 10);
  //servos.setposition( WRO_SERVO, 600 ); //change according to your box position
  servo5.write(180, 10);
  //servos.setposition( GRI_SERVO, 900 ); //change according to your box position
  servo6.write(80, 10);
  delay(500);
}
void throw_bluebox () {

  //servos.setposition( BAS_SERVO, 1500 );
  servo1.write(45, 10); //change according to your box position
  //servos.setposition( SHL_SERVO, 2100 );
  servo2.write(45, 10); //change according to your box position
  //servos.setposition( ELB_SERVO, 2100 );
  servo3.write(90, 10); //change according to your box position
  //servos.setposition( WRI_SERVO, 1800 );
  servo4.write(90, 10); //change according to your box position
  //servos.setposition( WRO_SERVO, 600 );
  servo5.write(180, 10); //change according to your box position
  //servos.setposition( GRI_SERVO, 900 );
  servo6.write(80, 10);  //change according to your box position
  delay(500);
}
void throw_greenbox () {
  //servos.setposition( BAS_SERVO, 1500 );
  servo1.write(135, 10); //change according to your box position
  //servos.setposition( SHL_SERVO, 2100 );
  servo2.write(135, 10); //change according to your box position
  //servos.setposition( ELB_SERVO, 2100 );
  servo3.write(0, 10);  //change according to your box position
  //servos.setposition( WRI_SERVO, 1800 );
  servo4.write(180, 10); //change according to your box position
  //servos.setposition( WRO_SERVO, 600 );
  servo5.write(90, 10); //change according to your box position
  //servos.setposition( GRI_SERVO, 900 );
  servo6.write(80, 10); //change according to your box position
  delay(500);
}
void throw_yellowbox () {
  //servos.setposition( BAS_SERVO, 1500 );
  servo1.write(160, 10); //change according to your box position
  //servos.setposition( SHL_SERVO, 2100 );
  servo2.write(160, 10); //change according to your box position
  //servos.setposition( ELB_SERVO, 2100 );
  servo3.write(90, 10); //change according to your box position
  //servos.setposition( WRI_SERVO, 1800 );
  servo4.write(180, 10); //change according to your box position
  //servos.setposition( WRO_SERVO, 600 );
  servo5.write(90, 10); //change according to your box position
  //servos.setposition( GRI_SERVO, 900 );
  servo6.write(80, 10); //change according to your box position
  delay(500);
}

void ret_capacitance() {
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);

  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! FDC.readMeasurement(MEASURMENT, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    Cal_Cap = ((float)capacitance / 1000);

    Serial.print((((float)capacitance / 1000)), 4);
    Serial.print("  pf, ");

    if (msb > UPPER_BOUND)               // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }
  }
}


