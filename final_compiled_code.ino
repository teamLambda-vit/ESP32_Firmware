/* 
 * Dual VL53L5CX Time-of-Flight Sensors + ICM-20948 IMU
 * Reads center 4 pixels from each ToF sensor and full IMU data
 * Includes Fall Detection using IMU with Pitch, Roll, and Yaw
 */

// ===================== LIBRARIES =====================
#include <Wire.h>                       // I2C communication library
#include <SparkFun_VL53L5CX_Library.h>  // For VL53L5CX ToF sensors
#include "ICM_20948.h"                  // For ICM-20948 IMU
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <math.h>                       // Added for atan2 and sqrt functions

const char* ssid = "Abinow";
const char* password ="12345678";

WiFiServer server(80);
WiFiClient client;
String data = "";

// Pins
#define pwm_drive 37
#define dir_drive 17
#define pwm_brush 16
#define dir_brush 38
#define lin_act_1_pwm 45
#define lin_act_1_dir 46
//#define voltage_pin 39
//#define current_sensor_pin 34  // Current sensor (HW-872A) analog pin

// ===================== TOF CONFIG =====================
SparkFun_VL53L5CX myImager1;            // First ToF sensor object
SparkFun_VL53L5CX myImager2;            // Second ToF sensor object

int sensorAddress1 = 0x44;              // Custom I2C address for sensor 1
int sensorAddress2 = 0x29;              // Default address for sensor 2

int sensorReset1 = 19;                  // GPIO pin for sensor 1 reset was 14 earlier
int sensorReset2 = 15;                  // GPIO pin for sensor 2 reset

VL53L5CX_ResultsData measurementData1;  // Storage for sensor 1 data
VL53L5CX_ResultsData measurementData2;  // Storage for sensor 2 data

int imageResolution = 0;                // Stores resolution (e.g., 64 for 8x8)
int imageWidth = 0;                     // Grid width (e.g., 8)

// ====== Obstacle detection config ======
unsigned long lastToFCheck = 0;
const uint16_t OBSTACLE_THRESHOLD = 300; // mm (30 cm)

// ====== FALL DETECTION CONFIG (IMU) ======
#define MAX_DELTA_PITCH_ROLL 20.0       // Max allowed tilt change in degrees. TUNE THIS!
#define MAX_DELTA_YAW 45.0              // Max allowed yaw change in degrees. TUNE THIS!

unsigned long lastIMUCheck = 0;
const uint16_t IMU_CHECK_INTERVAL = 100; // Check every 100ms
unsigned long lastIMUUpdate = 0;

float initialPitch = 0.0;
float initialRoll = 0.0;
float initialYaw = 0.0;
float currentPitch = 0.0;
float currentRoll = 0.0;
float currentYaw = 0.0;

bool isInitialAngleSet = false;

// Variables
int rpm = 0;
int pos = 0;
int new_pos = 0;
int dist = 0;
int linear_actr_pwm = 255;
long lastMPUCheck = 0;
long lastActionTime = 0;
bool actuatorMoving = false;
long lastCurrentCheck = 0;

const int MPU_ADDR = 0x68;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY, kalAngleX, kalAngleY;

// ===================== IMU CONFIG =====================
#define WIRE_PORT_IMU Wire1             // Use second I2C port for IMU
#define AD0_VAL 1                       // I2C address bit (1 = HIGH)
#define SERIAL_PORT Serial              // Alias for Serial output

ICM_20948_I2C myICM;                    // IMU sensor object

// ===================== SETUP =====================
void setup() {
  // Initialize serial communication
  SERIAL_PORT.begin(115200);
  delay(1000);  // Wait for serial monitor
  SERIAL_PORT.println("Starting Dual ToF + IMU Example");

  // --------- I2C Initialization ---------
  Wire.begin();               // Initialize primary I2C bus
  Wire.setClock(1000000);     // Set to 1MHz (VL53L5CX supports fast mode)

  WIRE_PORT_IMU.begin(35, 36);  // Initialize secondary I2C on pins 5/6
  WIRE_PORT_IMU.setClock(400000);  // 400kHz for IMU

  // --------- ToF Sensor Initialization ---------
  // Reset sequence for sensor 1
  pinMode(sensorReset2, OUTPUT);
  digitalWrite(sensorReset2, HIGH);  // Hold sensor 2 in reset

  pinMode(sensorReset1, OUTPUT);
  digitalWrite(sensorReset1, HIGH);  // Reset sensor 1
  delay(100);
  digitalWrite(sensorReset1, LOW);   // Release sensor 1
  delay(100);

  // Initialize sensor 1 with custom address
  SERIAL_PORT.println("Initializing ToF Sensor 1");
  if (!myImager1.begin()) {
    SERIAL_PORT.println("Sensor 1 not found. Freezing...");
    while (1);  // Halt if initialization fails
  }
  myImager1.setAddress(sensorAddress1);  // Change address

  // Release and initialize sensor 2
  digitalWrite(sensorReset2, LOW);
  delay(100);
  SERIAL_PORT.println("Initializing ToF Sensor 2");
  if (!myImager2.begin()) {
    SERIAL_PORT.println("Sensor 2 not found. Freezing...");
    while (1);
  }

  // Configure both sensors
  myImager1.setResolution(8 * 8);       // 8x8 grid (64 zones)
  myImager2.setResolution(8 * 8);
  imageResolution = myImager1.getResolution();
  imageWidth = sqrt(imageResolution);   // Calculate grid width (8)

  myImager1.setRangingFrequency(15);    // 15Hz update rate
  myImager2.setRangingFrequency(15);

  myImager1.startRanging();             // Start continuous measurements
  myImager2.startRanging();

  // --------- IMU Initialization ---------
  bool imuInitialized = false;
  while (!imuInitialized) {
    myICM.begin(WIRE_PORT_IMU, AD0_VAL);  // Initialize IMU
    SERIAL_PORT.print("IMU Init: ");
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Retrying IMU...");
      delay(500);
    } else {
      imuInitialized = true;
    }
  }

  // --------- Set Initial Reference Angle for Fall Detection ---------
  SERIAL_PORT.println("Place robot on panel. Calculating initial tilt in 3 seconds...");
  delay(3000); // Give time to position the robot

  // Take a reading to set the initial angle
  if (myICM.dataReady()) {
    myICM.getAGMT();
    calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &initialPitch, &initialRoll);
    initialYaw = 0.0; // Start with yaw at 0 degrees
    currentYaw = 0.0;
    isInitialAngleSet = true;
    SERIAL_PORT.print("Reference Angles Set - Pitch: ");
    SERIAL_PORT.print(initialPitch);
    SERIAL_PORT.print("°, Roll: ");
    SERIAL_PORT.print(initialRoll);
    SERIAL_PORT.print("°, Yaw: ");
    SERIAL_PORT.print(initialYaw);
    SERIAL_PORT.println("°");
  }

  // Pin Modes
  pinMode(dir_drive, OUTPUT);
  pinMode(pwm_drive, OUTPUT);
  pinMode(dir_brush, OUTPUT);
  pinMode(pwm_brush, OUTPUT);
  pinMode(lin_act_1_pwm, OUTPUT);
  pinMode(lin_act_1_dir, OUTPUT);
 

  // WiFi Setup
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 Local IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

// ===================== MAIN LOOP =====================
void loop() {
  // -------- ToF Sensor 1 --------
  if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {
    SERIAL_PORT.println("ToF Sensor 1:");
    printToFData(measurementData1);  // Print center 4 pixels
  }

  // -------- ToF Sensor 2 --------
  if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {
    SERIAL_PORT.println("ToF Sensor 2:");
    printToFData(measurementData2);
  }

  // -------- ToF Obstacle Check --------
  checkToF();

  // -------- IMU Data Reading --------
  if (myICM.dataReady()) {
    myICM.getAGMT();               // Read accelerometer/gyro/mag/temp
    printScaledAGMT(&myICM);       // Print formatted data
  }

  // -------- Fall Detection Check --------
  checkIMU();

  // -------- Handle Actuator Movement --------
  handleActuator();

  // -------- Handle WiFi Commands --------
  handleWiFiClient();

  delay(100);  // Short delay to prevent serial flooding
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Prints average distance of center 4 pixels in centimeters
 * @param data VL53L5CX measurement data structure
 */
void printToFData(VL53L5CX_ResultsData &data) {
  const int centerIndices[] = {27, 28, 35, 36};  // Center pixel indices (8x8 grid)
  float sum = 0;
  
  // Sum distances from center pixels
  for (int i = 0; i < 4; i++) {
    sum += data.distance_mm[centerIndices[i]];  // Distance in mm
  }
  
  // Convert to cm and print average
  SERIAL_PORT.print("Center avg: ");
  SERIAL_PORT.print(sum / 40.0f, 1);  // mm→cm (divide by 10), average 4 values
  SERIAL_PORT.println(" cm");
}

// === obstacle check function  ===
void checkToF() {
  if (millis() - lastToFCheck >= 100) {
    lastToFCheck = millis();

    int dist1 = -1;
    int dist2 = -1;

    // --- Sensor 1 ---
    if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {
      const int centerIndices[] = {27, 28, 35, 36};
      float sum = 0;
      for (int i = 0; i < 4; i++) sum += measurementData1.distance_mm[centerIndices[i]];
      dist1 = sum / 4.0f;
    }

    // --- Sensor 2 ---
    if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {
      const int centerIndices[] = {27, 28, 35, 36};
      float sum = 0;
      for (int i = 0; i < 4; i++) sum += measurementData2.distance_mm[centerIndices[i]];
      dist2 = sum / 4.0f;
    }

    // --- Print results ---
    Serial.print("ToF1: ");
    if (dist1 > 0) Serial.print(dist1 / 10.0f); else Serial.print("N/A");
    Serial.print(" cm | ToF2: ");
    if (dist2 > 0) Serial.print(dist2 / 10.0f); else Serial.print("N/A");
    Serial.println(" cm");

    // --- Obstacle detection ---
    if ((dist1 > 0 && dist1 < OBSTACLE_THRESHOLD) ||
        (dist2 > 0 && dist2 < OBSTACLE_THRESHOLD)) {
      Serial.println("Obstacle detected by ToF! Stopping motors...");
      stop_brush();
      brake();
    }
  }
}

/**
 * Updates yaw angle by integrating gyroscope Z-axis data
 */
void updateYawAngle() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  if (lastUpdate == 0) {
    lastUpdate = now;
    return;
  }
  
  float dt = (now - lastUpdate) / 1000.0; // Convert to seconds
  lastUpdate = now;
  
  // Integrate gyroscope Z-axis data (dps to degrees)
  float gyroZ_dps = myICM.gyrZ(); // degrees per second
  currentYaw += gyroZ_dps * dt;
  
  // Keep yaw between -180 and 180 degrees
  if (currentYaw > 180.0) currentYaw -= 360.0;
  if (currentYaw < -180.0) currentYaw += 360.0;
}

// === Fall Detection Function ===
void checkIMU() {
  if (millis() - lastIMUCheck >= IMU_CHECK_INTERVAL && isInitialAngleSet) {
    lastIMUCheck = millis();

    if (myICM.dataReady()) {
      myICM.getAGMT(); // Read new data
      
      // Update yaw angle from gyroscope
      updateYawAngle();

      float currentPitch, currentRoll;
      // Calculate current tilt from raw accelerometer data
      calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &currentPitch, &currentRoll);

      // Calculate how much we've tilted from the start position
      float deltaPitch = abs(currentPitch - initialPitch);
      float deltaRoll = abs(currentRoll - initialRoll);
      
      // Handle yaw wrap-around (0-360 degrees)
      float deltaYaw = abs(currentYaw - initialYaw);
      if (deltaYaw > 180.0) {
        deltaYaw = 360.0 - deltaYaw;
      }

      // Print for monitoring
      Serial.print("IMU Tilt Δ - Pitch: ");
      Serial.print(deltaPitch);
      Serial.print("°, Roll: ");
      Serial.print(deltaRoll);
      Serial.print("°, Yaw: ");
      Serial.print(deltaYaw);
      Serial.println("°");

      // Check if any orientation change is too large
      if (deltaPitch > MAX_DELTA_PITCH_ROLL || 
          deltaRoll > MAX_DELTA_PITCH_ROLL || 
          deltaYaw > MAX_DELTA_YAW) {
        triggerFallProtocol(deltaPitch, deltaRoll, deltaYaw);
      }
    }
  }
}

/**
 * Calculates Pitch and Roll angles in degrees from raw accelerometer data
 * @param accX Raw accelerometer X value
 * @param accY Raw accelerometer Y value
 * @param accZ Raw accelerometer Z value
 * @param pitch Calculated pitch angle (output)
 * @param roll Calculated roll angle (output)
 */
void calculateTilt(int16_t accX, int16_t accY, int16_t accZ, float *pitch, float *roll) {
  // Convert raw sensor readings to G-Force (approx. 1000 LSB per G for many sensors)
  float accX_g = accX / 1000.0;
  float accY_g = accY / 1000.0;
  float accZ_g = accZ / 1000.0;

  // Calculate Pitch (rotation around Y-axis) using arctangent
  *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;

  // Calculate Roll (rotation around X-axis) using arctangent
  *roll = atan2(accY_g, accZ_g) * 180.0 / PI;
}

// === Emergency Fall Protocol ===
void triggerFallProtocol(float dPitch, float dRoll, float dYaw) {
  Serial.println("!!! FALL DETECTED !!!");
  Serial.print("Orientation Change - Pitch: ");
  Serial.print(dPitch);
  Serial.print("°, Roll: ");
  Serial.print(dRoll);
  Serial.print("°, Yaw: ");
  Serial.print(dYaw);
  Serial.println("°");
  Serial.println("Engaging Emergency Stop...");

  // EMERGENCY STOP: Halt all motors and actuators
  brake();
  stop_brush();
  lin_stop();

  // Optional: Enter a safe state until reset
  while (true) {
    delay(1000); // Halt forever, blinking LED could be added here
  }
}

/**
 * Prints formatted floating-point numbers with leading zeros
 * @param val Value to print
 * @param leading Total number of digits (including before decimal)
 * @param decimals Number of decimal places
 */
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) SERIAL_PORT.print("-");
  else SERIAL_PORT.print(" ");

  // Print leading zeros
  for (uint8_t i = 0; i < leading; i++) {
    uint32_t tenpow = pow(10, leading - 1 - i);
    if (aval < tenpow) SERIAL_PORT.print("0");
    else break;
  }
  SERIAL_PORT.print(aval, decimals);  // Print value
}

/**
 * Prints formatted IMU data (accelerometer in mg, gyro in dps)
 * @param sensor Pointer to IMU object
 */
void printScaledAGMT(ICM_20948_I2C *sensor) {
  SERIAL_PORT.print("IMU: Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2); SERIAL_PORT.print(" ] ");

  SERIAL_PORT.print("Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2); SERIAL_PORT.print(" ]");

  SERIAL_PORT.println();
}

void handleWiFiClient() {
  client = server.available();
  
  if (!client) return;

  data = checkClient();

  if (data.length() == 0) return;

  String command = data.substring(0, 1);
  int value = data.substring(1).toInt();

  Serial.print("Received command: ");
  Serial.println(data);

  if (command == "f") forward(value);
  else if (command == "r") reverse(value);
  else if (command == "b") brake();
  else if (command == "c") run_brush(value);
  else if (command == "e") run_brush_rev(value);
  else if (command == "s") stop_brush();
  else if (command == "l" || command == "m" || command == "n" || command == "o") move_actuator(value);
  else if (command == "p") lin_stop();
  client.stop();
}

String checkClient() {
  while (!client.available()) {
    delay(1);
  }
  String request = client.readStringUntil('\r');
  request.remove(0, 5);
  request.remove(request.length() - 9, 9);
  return request;
}

void move_actuator(int target_pos) {
  if (actuatorMoving) return;
  
  new_pos = target_pos;
  dist = abs(pos - new_pos);
  
  if (dist > 0) {
    actuatorMoving = true;
    lastActionTime = millis();
    if (pos < new_pos) lin_inc();
    else lin_dec();
  }
}

void handleActuator() {
  if (actuatorMoving && millis() - lastActionTime >= calc_time(dist)) {
    lin_stop();
    actuatorMoving = false;
    pos = new_pos;
  }
}

// Motor Functions
void forward(int pwm) {
  digitalWrite(dir_drive, HIGH);
  analogWrite(pwm_drive, pwm);
}

void reverse(int pwm) {
  digitalWrite(dir_drive, LOW);
  analogWrite(pwm_drive, pwm);
}

void brake() {
  digitalWrite(dir_drive, LOW);
  analogWrite(pwm_drive, 0);
}

void run_brush(int pwm) {
  digitalWrite(dir_brush, HIGH);
  analogWrite(pwm_brush, pwm);
}

void run_brush_rev(int pwm) {
  digitalWrite(dir_brush, LOW);
  analogWrite(pwm_brush, pwm);
}

void stop_brush() {
  analogWrite(pwm_brush, 0);
}

// Actuator Functions
void lin_inc() {
  digitalWrite(lin_act_1_dir, LOW);
  analogWrite(lin_act_1_pwm, linear_actr_pwm);
}

void lin_dec() {
  digitalWrite(lin_act_1_dir, HIGH);
  analogWrite(lin_act_1_pwm, linear_actr_pwm);
}

void lin_stop() {
  analogWrite(lin_act_1_pwm, 0);
}

int calc_time(int dist) {
  return (int)(1000 * dist / 6.893);
}

int calc_pwm(float volt) {
  return (int)((11.0 / volt) * 255);
}