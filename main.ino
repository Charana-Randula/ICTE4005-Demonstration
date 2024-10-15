#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// Define pins for the ultrasonic sensor and relay
#define trigPin 8
#define echoPin 10
#define relayPin 7 // Define pin for the relay
#define variableResistorPin A1 // Define pin for the variable resistor

// Create instances of the MPU6050 and Servo objects
MPU6050 mpu;
Servo myServo;

// PID parameters
float Kp = 1.0; // Proportional gain
float Ki = 0.2; // Integral gain
float Kd = 0.0; // Derivative gain

// PID variables
float setPoint = 90; // Desired servo angle (center position)
float lastError = 0; // Last error value
float integral = 0;  // Integral sum

// Define the servo range
const int minAngle = 35;
const int maxAngle = 125;

// Ultrasonic sensor variables
long duration;
int distance;
bool finDetected = false;
unsigned long lastFinTime = 0;
unsigned long debounceDelay = 200;  // Debounce delay to prevent double counts (200 ms)

// RPM variables
unsigned long lastPassTime = 0; // Time of last fin detection
float rpm = 0; // Store RPM value

void setup() {
  Serial.begin(115200); // Start serial communication

  // Wait until the serial port is ready
  while (!Serial) {
  }
  Wire.begin();
  initializeMPU();
  initializeServo();
  initializeUltrasonic();
  
  pinMode(relayPin, OUTPUT); // Set relay pin as output
}

void loop() {
  updateServo();
  checkFinDetection();
  checkRelay(); // Check relay conditions
  delay(100); // Adjust delay as needed
}

// Function to initialize the MPU6050
void initializeMPU() {
  mpu.initialize();
}

// Function to initialize the Servo
void initializeServo() {
  myServo.attach(9); // Change to your servo pin
  myServo.write(setPoint); // Set initial position to the center
  delay(1000); // Wait for the servo to reach position
}

// Function to initialize the ultrasonic sensor
void initializeUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Function to update the servo based on MPU6050 readings
void updateServo() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the desired angle based on the Y-axis value
  if (ay < 0) { // If Y is negative
    setPoint += 35; // Increase the target position
  } else if (ay > 0) { // If Y is positive
    setPoint -= 35; // Decrease the target position
  }

  // Constrain the setPoint to be within the defined range
  setPoint = constrain(setPoint, minAngle, maxAngle);

  // Get the current servo angle
  float input = myServo.read();

  // Calculate PID error
  float error = setPoint - input;

  // Proportional term
  float proportional = Kp * error;

  // Integral term
  integral += Ki * error;

  // Derivative term
  float derivative = Kd * (error - lastError);
  
  // PID output
  float output = proportional + integral + derivative;

  // Constrain output to servo range
  output = constrain(setPoint + output, minAngle, maxAngle);
  
  // Set the servo angle
  myServo.write(output);

  // Save error for next iteration
  lastError = error;
}

// Function to check for fin detection using the ultrasonic sensor
void checkFinDetection() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin and calculate distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Convert time to distance (in cm)

  unsigned long currentTime = millis();

  // Check if the fin is passing in front of the sensor (within 5 cm)
  if (distance <= 5 && !finDetected) {
    if (currentTime - lastFinTime > debounceDelay) {
      // If enough time has passed since last detection, record a pass
      Serial.println("PASS");  // Send data to serial
      lastFinTime = currentTime;  // Update last detected time
      int16_t tempRaw = mpu.getTemperature();
      float temperature = (tempRaw / 340.0) + 36.53;
      Serial.println(temperature);
      int tempInt = ceil(temperature);
      Wire.beginTransmission(8);  // Send data to slave with address 8
      Wire.write((byte*)&temperature, sizeof(temperature)); 
      Wire.endTransmission();     // End the transmission
      
      // Call calculateRPM when a fin is detected
      calculateRPM(); // Update RPM based on fin detection
    }
    finDetected = true;  // Fin detected
  }
  // Reset once the fin has moved out of range
  else if (distance > 5 && finDetected) {
    finDetected = false;  // Reset detection flag
  }
}

// Function to calculate RPM based on the last fin detection time
float calculateRPM() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  if (lastPassTime != 0) { // Ensure lastPassTime is initialized
    unsigned long timeDifference = currentTime - lastPassTime; // Calculate time difference
    if (timeDifference > 0) { // Avoid division by zero
      rpm = 60.0 / (timeDifference / 1000.0 * 3); // 3 fins = 1 full revolution
    }
  }
  lastPassTime = currentTime; // Update the last pass time
  return rpm; // Return the calculated RPM
}

// Function to check the relay state based on voltage and RPM
void checkRelay() {
  int variableResistorValue = analogRead(variableResistorPin); // Read voltage from the variable resistor
  float voltage = variableResistorValue * (5.0 / 1023.0); // Convert to voltage
  // Serial.println(voltage); // Print voltage for debugging

  // // Send the voltage as an integer to the Python program
  int voltageInt = static_cast<int>(voltage); // Convert voltage to integer
  Serial.println(voltageInt); // Send integer voltage

  // Conditions to turn off the relay
  if (rpm > 5 && voltage > 3.0) { // Adjust the voltage threshold as needed
    digitalWrite(relayPin, HIGH); // Turn on the relay
  } else {
    digitalWrite(relayPin, LOW); // Turn off the relay
  }
}