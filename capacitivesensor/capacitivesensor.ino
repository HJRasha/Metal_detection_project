const int sensorPin = A0; // Analog pin connected to the sensor output
const int ledPin = 13;   // Pin connected to an LED for indicating material presence

int plasticBaseline;      // Baseline capacitance value for plastic
int paperBaseline;        // Baseline capacitance value for paper
int threshold = 50;       // Adjust this threshold based on your testing

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  // Calibrate the sensor for plastic and paper
  calibrateBaselines();
}

void loop() {
  // Measure the current capacitance value
  int sensorValue = analogRead(sensorPin);

  // Determine material based on threshold and respective baseline
  if (abs(sensorValue - plasticBaseline) < threshold) {
    // Plastic detected
    digitalWrite(ledPin, HIGH);
    Serial.println("Plastic Detected");
  } else if (abs(sensorValue - paperBaseline) < threshold) {
    // Paper detected
    digitalWrite(ledPin, HIGH);
    Serial.println("Paper Detected");
  } else {
    // No material detected
    digitalWrite(ledPin, LOW);
    Serial.println("No Material Detected");
  }

  delay(500); // Adjust delay based on your application's requirements
}

void calibrateBaselines() {
  Serial.println("Calibrating for Plastic... Please place plastic on the sensor.");
  delay(5000);
  plasticBaseline = analogRead(sensorPin);
  Serial.print("Plastic Baseline: ");
  Serial.println(plasticBaseline);

  Serial.println("Calibrating for Paper... Please place paper on the sensor.");
  delay(5000);
  paperBaseline = analogRead(sensorPin);
  Serial.print("Paper Baseline: ");
  Serial.println(paperBaseline);

  Serial.println("Calibration Complete.");
}

