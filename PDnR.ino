#include <Wire.h>
#include <VL53L0X.h>
#include <AFMotor.h>
#include <Servo.h>

#define XSHUT_PIN1 A0 // Change these pins to match your setup
#define XSHUT_PIN2 A1
#define XSHUT_PIN3 A2
#define XSHUT_PIN4 A3

// DC motor on M2
AF_DCMotor motor1(4);
AF_DCMotor motor2(3);
AF_DCMotor motor3(2);

#define SERVO_PIN 9 // Pin for servo motor

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

Servo servoMotor;

// Adjustable pothole threshold (in millimeters)
int potholeThreshold = 80;
int delayval =2800;// Adjust delay for the exact location

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(XSHUT_PIN1, OUTPUT);
  pinMode(XSHUT_PIN2, OUTPUT);
  pinMode(XSHUT_PIN3, OUTPUT);
  pinMode(XSHUT_PIN4, OUTPUT);

  motor1.setSpeed(200);
  motor1.run(RELEASE);
  motor2.setSpeed(200);
  motor2.run(RELEASE);
  motor3.setSpeed(200);
  motor3.run(RELEASE);

  servoMotor.attach(SERVO_PIN); // Attach servo motor
   // Initialize sensor 1
  pinMode(XSHUT_PIN1, OUTPUT);
  digitalWrite(XSHUT_PIN1, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN1, HIGH);
  delay(10);
  if (!sensor1.init()) {
    Serial.println("Failed to initialize Sensor 1!");
    while (1);
  }
  sensor1.setAddress(0x31); // Set unique address for sensor 1
  sensor1.setMeasurementTimingBudget(20000);
  sensor1.startContinuous();
  
  // Initialize sensor 2
  pinMode(XSHUT_PIN2, OUTPUT);
  digitalWrite(XSHUT_PIN2, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN2, HIGH);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Failed to initialize Sensor 2!");
    while (1);
  }
  sensor2.setAddress(0x32); // Set unique address for sensor 2
  sensor2.setMeasurementTimingBudget(20000);
  sensor2.startContinuous();
  
  // Initialize sensor 3
  pinMode(XSHUT_PIN3, OUTPUT);
  digitalWrite(XSHUT_PIN3, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN3, HIGH);
  delay(10);
  if (!sensor3.init()) {
    Serial.println("Failed to initialize Sensor 3!");
    while (1);
  }
  sensor3.setAddress(0x33); // Set unique address for sensor 3
  sensor3.setMeasurementTimingBudget(20000);
  sensor3.startContinuous();
  
  // Initialize sensor 4
  pinMode(XSHUT_PIN4, OUTPUT);
  digitalWrite(XSHUT_PIN4, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN4, HIGH);
  delay(10);
  if (!sensor4.init()) {
    Serial.println("Failed to initialize Sensor 4!");
    while (1);
  }
  sensor4.setAddress(0x34); // Set unique address for sensor 4
  sensor4.setMeasurementTimingBudget(20000);
  sensor4.startContinuous();
  servoMotor.write(90); // Change this value to the center position 
}

void loop() {
  // Read distance from sensor 1
  uint16_t distance1 = sensor1.readRangeContinuousMillimeters();
  
  // Read distance from sensor 2
  uint16_t distance2 = sensor2.readRangeContinuousMillimeters();
  
  // Read distance from sensor 3
  uint16_t distance3 = sensor3.readRangeContinuousMillimeters();
  
  // Read distance from sensor 4
  uint16_t distance4 = sensor4.readRangeContinuousMillimeters();

  if (distance1 > potholeThreshold) {
    Serial.println("Pothole detected by Sensor 1!");
    delay(delayval); // Wait 2 seconds
    
    stopMotor(motor1);
    stopMotor(motor3);
    startMotor(motor2);
    
    for (int i = 0; i < 3; i++) {
      servoManeuver(20); // Move servo 20 degrees right
      delay(500);         // Adjust delay for desired speed
      servoManeuver(40); // Move servo 40 degrees left
      delay(500);
    }
    servoMotor.write(90);
    stopMotor(motor2);
    startMotor(motor1);
    startMotor(motor3);
  } else if (distance2 > potholeThreshold) {
    Serial.println("Pothole detected by Sensor 2!");
    delay(delayval); // Wait 
    
    stopMotor(motor1);
    stopMotor(motor3);
    startMotor(motor2);
    
    for (int i = 0; i < 3; i++) {
      servoManeuver(90); // Move servo 90 degrees right
      delay(500);         // Adjust delay for desired speed
      servoManeuver(120); // Move servo 120 degrees left
      delay(500);
    }
    servoMotor.write(90);
    stopMotor(motor2);
    startMotor(motor1);
    startMotor(motor3);
  }

   else if (distance3 > potholeThreshold) {
    Serial.println("Pothole detected by Sensor 3!");
    delay(delayval); // Wait
    
    stopMotor(motor1);
    stopMotor(motor3);
    startMotor(motor2);
    
    for (int i = 0; i < 3; i++) {
      servoManeuver(130); // Move servo 130 degrees right
      delay(500);         // Adjust delay for desired speed
      servoManeuver(150); // Move servo 150 degrees left
      delay(500);
    }
    servoMotor.write(90);
    stopMotor(motor2);
    startMotor(motor3);
    startMotor(motor1);
  }
   else if (distance4 > potholeThreshold) {
    Serial.println("Pothole detected by Sensor 4!");
    delay(delayval); // Wait 
    
    stopMotor(motor1);
    stopMotor(motor3);
    startMotor(motor2);
    
    for (int i = 0; i < 3; i++) {
      servoManeuver(80); // Move servo 80 degrees right
      delay(500);         // Adjust delay for desired speed
      servoManeuver(45); // Move servo 45 degrees left
      delay(500);
    }
    servoMotor.write(90);
    stopMotor(motor2);
    startMotor(motor1);
    startMotor(motor3);
  }
  else {
    // Start motor A,C if no pothole is detected
    motor1.run(FORWARD);
    motor3.run(FORWARD);
  }
  Serial.print("Distance from Sensor 1: ");
  Serial.println(distance1);
  Serial.print("Distance from Sensor 2: ");
  Serial.println(distance2);
  Serial.print("Distance from Sensor 3: ");
  Serial.println(distance3);
  Serial.print("Distance from Sensor 4: ");
  Serial.println(distance4);
}

// Function to stop a motor
void stopMotor(AF_DCMotor& motor) {
  motor.run(RELEASE);
}

// Function to start a motor
void startMotor(AF_DCMotor& motor) {
  motor.run(FORWARD);
}

// Function for smoother servo movement (adjust angle and delay as needed)
void servoManeuver(int angle) {
  for (int i = servoMotor.read(); i != angle; i += (angle > servoMotor.read()) ? 5 : -5) {
    servoMotor.write(i);
    delay(10);
  }
  servoMotor.write(angle);
}