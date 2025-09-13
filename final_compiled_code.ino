/* 
 * Dual VL53L5CX Time-of-Flight Sensors + ICM-20948 IMU + LoRa Communication
 * Reads center 4 pixels from each ToF sensor and full IMU data
 * Includes Fall Detection using IMU with Pitch, Roll, and Yaw
 * Sends data via LoRa to receiver
 */

// ===================== LIBRARIES =====================
#include <Wire.h>                       // I2C communication library
#include <SparkFun_VL53L5CX_Library.h>  // For VL53L5CX ToF sensors
#include "ICM_20948.h"                  // For ICM-20948 IMU
#include <WiFi.h>                       // WiFi connectivity
#include <WiFiClient.h>                 // WiFi client functionality
#include <WiFiServer.h>                 // WiFi server functionality
#include <math.h>                       // Math functions (atan2, sqrt)
#include <LoRa.h>                       // LoRa wireless communication
#include <SPI.h>                        // SPI communication protocol

// WiFi network credentials
const char* ssid = "Abinow";            // WiFi network name
const char* password = "12345678";      // WiFi password

// Create a server on port 80
WiFiServer server(80);
// Client object for handling connections
WiFiClient client;
// String to store received data
String data = "";

// LoRa module pin definitions
#define LORA_SS 5                       // Slave select pin
#define LORA_RST 14                     // Reset pin
#define LORA_DIO0 2                     // Digital IO 0 pin

// Motor and actuator control pins
#define pwm_drive 16                    // Drive motor PWM pin
#define dir_drive 15                    // Drive motor direction pin
#define pwm_brush 18                    // Brush motor PWM pin
#define dir_brush 17                    // Brush motor direction pin
#define lin_act_1_pwm 3                 // Linear actuator PWM pin
#define lin_act_1_dir 6                 // Linear actuator direction pin

// ===================== TOF CONFIG =====================
SparkFun_VL53L5CX myImager1;            // First ToF sensor object
SparkFun_VL53L5CX myImager2;            // Second ToF sensor object

int sensorAddress1 = 0x44;              // Custom I2C address for sensor 1
int sensorAddress2 = 0x29;              // Default address for sensor 2

int sensorReset1 = 47;                  // GPIO pin for sensor 1 reset
int sensorReset2 = 48;                  // GPIO pin for sensor 2 reset

VL53L5CX_ResultsData measurementData1;  // Storage for sensor 1 data
VL53L5CX_ResultsData measurementData2;  // Storage for sensor 2 data

int imageResolution = 0;                // Stores resolution (e.g., 64 for 8x8)
int imageWidth = 0;                     // Grid width (e.g., 8)

// ====== Obstacle detection config ======
unsigned long lastToFCheck = 0;         // Last time ToF was checked
const uint16_t OBSTACLE_THRESHOLD = 300; // Obstacle distance threshold in mm (30 cm)

// ====== FALL DETECTION CONFIG (IMU) ======
#define MAX_DELTA_PITCH_ROLL 20.0       // Max allowed tilt change in degrees
#define MAX_DELTA_YAW 45.0              // Max allowed yaw change in degrees

unsigned long lastIMUCheck = 0;         // Last time IMU was checked
const uint16_t IMU_CHECK_INTERVAL = 100; // Check every 100ms
unsigned long lastIMUUpdate = 0;        // Last time IMU was updated

// IMU angle variables
float initialPitch = 0.0;               // Initial pitch angle reference
float initialRoll = 0.0;                // Initial roll angle reference
float initialYaw = 0.0;                 // Initial yaw angle reference
float currentPitch = 0.0;               // Current pitch angle
float currentRoll = 0.0;                // Current roll angle
float currentYaw = 0.0;                 // Current yaw angle

bool isInitialAngleSet = false;         // Flag if initial angles are set

// Motor control variables
int rpm = 0;                            // Motor RPM value
int pos = 0;                            // Current actuator position
int new_pos = 0;                        // Target actuator position
int dist = 0;                           // Distance to move actuator
int linear_actr_pwm = 255;              // PWM value for linear actuator
long lastMPUCheck = 0;                  // Last time MPU was checked
long lastActionTime = 0;                // Last time an action was performed
long lastCurrentCheck = 0;              // Last time current was checked

// ====== ROBOT STATUS FLAGS ======
bool driveMoving = false;               // Flag if drive motor is moving
bool brushMoving = false;               // Flag if brush motor is moving
bool actuatorMoving = false;            // Flag if linear actuator is moving
bool botDisoriented = false;            // Flag if robot is disoriented/fallen
bool obstacleDetected = false;          // Flag if obstacle is detected
bool emergencyStop = false;             // Flag if emergency stop is activated

// Sensor values for LoRa transmission
int tof1Distance = 0;                   // Distance from ToF sensor 1
int tof2Distance = 0;                   // Distance from ToF sensor 2
float imuPitch = 0.0;                   // Pitch angle from IMU
float imuRoll = 0.0;                    // Roll angle from IMU
float imuYaw = 0.0;                     // Yaw angle from IMU

// IMU raw data values
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float magX = 0.0, magY = 0.0, magZ = 0.0;
float temperature = 0.0;

// LoRa transmission timing
unsigned long lastLoRaTransmission = 0; // Last time data was sent via LoRa
const uint16_t LORA_INTERVAL = 2000;    // Send data every 2 seconds

// ===================== IMU CONFIG =====================
#define WIRE_PORT_IMU Wire1             // Use second I2C port for IMU
#define AD0_VAL 1                       // I2C address bit (1 = HIGH)
#define SERIAL_PORT Serial              // Alias for Serial output

ICM_20948_I2C myICM;                    // IMU sensor object

// ===================== SETUP =====================
void setup() {
  // Initialize serial communication
  SERIAL_PORT.begin(115200);            // Start serial at 115200 baud
  delay(1000);                          // Wait for serial monitor
  SERIAL_PORT.println("Starting Dual ToF + IMU + LoRa Example");

  // --------- LoRa Initialization ---------
  SERIAL_PORT.println("Initializing LoRa...");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);  // Set LoRa module pins
  
  while (!LoRa.begin(433E6)) {          // Initialize LoRa at 433MHz
    SERIAL_PORT.println(".");           // Print dots while initializing
    delay(500);                         // Wait 500ms between attempts
  }
  LoRa.setSyncWord(0xA5);               // Set LoRa sync word
  SERIAL_PORT.println("LoRa Initializing OK!");

  // --------- I2C Initialization ---------
  Wire.begin();                         // Initialize primary I2C bus
  Wire.setClock(1000000);               // Set to 1MHz for ToF sensors

  WIRE_PORT_IMU.begin(35,36);           // Initialize secondary I2C for IMU
  WIRE_PORT_IMU.setClock(400000);       // 400kHz for IMU

  // --------- ToF Sensor Initialization ---------
  // Reset sequence for sensor 1
  pinMode(sensorReset2, OUTPUT);        // Set sensor 2 reset pin as output
  digitalWrite(sensorReset2, HIGH);     // Hold sensor 2 in reset

  pinMode(sensorReset1, OUTPUT);        // Set sensor 1 reset pin as output
  digitalWrite(sensorReset1, HIGH);     // Reset sensor 1
  delay(100);                           // Wait 100ms
  digitalWrite(sensorReset1, LOW);      // Release sensor 1
  delay(100);                           // Wait 100ms

  // Initialize sensor 1 with custom address
  SERIAL_PORT.println("Initializing ToF Sensor 1");
  if (!myImager1.begin()) {             // Try to initialize sensor 1
    SERIAL_PORT.println("Sensor 1 not found. Freezing...");
    while (1);                          // Halt if initialization fails
  }
  myImager1.setAddress(sensorAddress1); // Change sensor 1 address

  // Release and initialize sensor 2
  digitalWrite(sensorReset2, LOW);      // Release sensor 2 from reset
  delay(100);                           // Wait 100ms
  SERIAL_PORT.println("Initializing ToF Sensor 2");
  if (!myImager2.begin()) {             // Try to initialize sensor 2
    SERIAL_PORT.println("Sensor 2 not found. Freezing...");
    while (1);                          // Halt if initialization fails
  }

  // Configure both sensors
  myImager1.setResolution(8 * 8);       // Set 8x8 grid (64 zones)
  myImager2.setResolution(8 * 8);       // Set 8x8 grid (64 zones)
  imageResolution = myImager1.getResolution();  // Get resolution value
  imageWidth = sqrt(imageResolution);   // Calculate grid width (8)

  myImager1.setRangingFrequency(15);    // Set 15Hz update rate
  myImager2.setRangingFrequency(15);    // Set 15Hz update rate

  myImager1.startRanging();             // Start continuous measurements
  myImager2.startRanging();             // Start continuous measurements

  // --------- IMU Initialization ---------
  bool imuInitialized = false;          // IMU initialization flag
  while (!imuInitialized) {             // Keep trying until IMU is initialized
    myICM.begin(WIRE_PORT_IMU, AD0_VAL);  // Initialize IMU
    SERIAL_PORT.print("IMU Init: ");    // Print initialization status
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok) {  // If initialization failed
      SERIAL_PORT.println("Retrying IMU..."); // Print retry message
      delay(500);                       // Wait 500ms before retry
    } else {
      imuInitialized = true;            // Set flag if initialization succeeded
    }
  }

  // --------- Set Initial Reference Angle for Fall Detection ---------
  SERIAL_PORT.println("Place robot on panel. Calculating initial tilt in 3 seconds...");
  delay(3000);                          // Give time to position the robot

  // Take a reading to set the initial angle
  if (myICM.dataReady()) {              // If IMU data is ready
    myICM.getAGMT();                    // Read accelerometer/gyro/mag/temp
    calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &initialPitch, &initialRoll);  // Calculate tilt
    initialYaw = 0.0;                   // Start with yaw at 0 degrees
    currentYaw = 0.0;                   // Set current yaw to 0
    isInitialAngleSet = true;           // Mark initial angles as set
    SERIAL_PORT.print("Reference Angles Set - Pitch: ");  // Print reference angles
    SERIAL_PORT.print(initialPitch);
    SERIAL_PORT.print("°, Roll: ");
    SERIAL_PORT.print(initialRoll);
    SERIAL_PORT.print("°, Yaw: ");
    SERIAL_PORT.print(initialYaw);
    SERIAL_PORT.println("°");
  }

  // Pin Modes for motor control
  pinMode(dir_drive, OUTPUT);           // Set drive direction pin as output
  pinMode(pwm_drive, OUTPUT);           // Set drive PWM pin as output
  pinMode(dir_brush, OUTPUT);           // Set brush direction pin as output
  pinMode(pwm_brush, OUTPUT);           // Set brush PWM pin as output
  pinMode(lin_act_1_pwm, OUTPUT);       // Set linear actuator PWM pin as output
  pinMode(lin_act_1_dir, OUTPUT);       // Set linear actuator direction pin as output

  // Reset all status flags
  resetStatusFlags();

  // WiFi Setup
  Serial.println("Connecting to WIFI"); // Print WiFi connection message
  WiFi.begin(ssid, password);           // Start WiFi connection
  while (WiFi.status() != WL_CONNECTED) {  // While not connected to WiFi
    Serial.print(".");                   // Print dots while connecting
    delay(500);                         // Wait 500ms between attempts
  }
  Serial.println("\nWiFi connected");   // Print connection success
  Serial.print("ESP32 Local IP: ");     // Print local IP address
  Serial.println(WiFi.localIP());
  server.begin();                       // Start the web server
}

// ===================== MAIN LOOP =====================
void loop() {
  // -------- ToF Sensor 1 --------
  if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {  // If sensor 1 has data
    SERIAL_PORT.println("ToF Sensor 1:");  // Print sensor label
    tof1Distance = printToFData(measurementData1);  // Get center 4 pixels distance
  }

  // -------- ToF Sensor 2 --------
  if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {  // If sensor 2 has data
    SERIAL_PORT.println("ToF Sensor 2:");  // Print sensor label
    tof2Distance = printToFData(measurementData2);  // Get center 4 pixels distance
  }

  // -------- ToF Obstacle Check --------
  checkToF();                         // Check for obstacles using ToF sensors

  // -------- IMU Data Reading --------
  if (myICM.dataReady()) {            // If IMU data is ready
    myICM.getAGMT();                   // Read accelerometer/gyro/mag/temp
    
    // Store raw IMU data for transmission
    accelX = myICM.accX() / 1000.0;   // Convert to G
    accelY = myICM.accY() / 1000.0;
    accelZ = myICM.accZ() / 1000.0;
    gyroX = myICM.gyrX();             // Degrees per second
    gyroY = myICM.gyrY();
    gyroZ = myICM.gyrZ();
    magX = myICM.magX();              // Raw magnetometer data
    magY = myICM.magY();
    magZ = myICM.magZ();
    temperature = myICM.temp();       // Temperature in degrees C
    
    printScaledAGMT(&myICM);           // Print formatted data
    
    // Update IMU values for transmission
    calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &imuPitch, &imuRoll);  // Calculate tilt
    updateYawAngle();                  // Update yaw from gyro
    imuYaw = currentYaw;               // Set IMU yaw to current yaw
  }

  // -------- Fall Detection Check --------
  checkIMU();                         // Check for falls using IMU

  // -------- Handle Actuator Movement --------
  handleActuator();                   // Handle linear actuator movement

  // -------- Handle WiFi Commands --------
  handleWiFiClient();                 // Handle incoming WiFi commands

  // -------- Send Data via LoRa --------
  if (millis() - lastLoRaTransmission >= LORA_INTERVAL) {  // If it's time to send LoRa data
    sendLoRaData();                    // Send data via LoRa
    lastLoRaTransmission = millis();   // Update last transmission time
  }

  delay(100);                          // Short delay to prevent serial flooding
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Prints average distance of center 4 pixels in centimeters and returns value
 * @param data VL53L5CX measurement data structure
 * @return Average distance in mm
 */
int printToFData(VL53L5CX_ResultsData &data) {
  const int centerIndices[] = {27, 28, 35, 36};  // Center pixel indices (8x8 grid)
  float sum = 0;                    // Initialize sum variable
  
  // Sum distances from center pixels
  for (int i = 0; i < 4; i++) {    // Loop through 4 center pixels
    sum += data.distance_mm[centerIndices[i]];  // Add distance to sum
  }
  
  int avgDistance = sum / 4.0f;     // Calculate average distance
  
  // Convert to cm and print average
  SERIAL_PORT.print("Center avg: ");  // Print label
  SERIAL_PORT.print(avgDistance / 10.0f, 1);  // mm→cm (divide by 10)
  SERIAL_PORT.println(" cm");        // Print unit

  return avgDistance;               // Return average distance
}

// === obstacle check function  ===
void checkToF() {
  if (millis() - lastToFCheck >= 100) {  // If 100ms has passed since last check
    lastToFCheck = millis();            // Update last check time

    int dist1 = -1;                     // Initialize sensor 1 distance
    int dist2 = -1;                     // Initialize sensor 2 distance
    bool newObstacleDetected = false;   // Temporary obstacle detection flag

    // --- Sensor 1 ---
    if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {  // If sensor 1 has data
      const int centerIndices[] = {27, 28, 35, 36};  // Center pixel indices
      float sum = 0;                  // Initialize sum
      for (int i = 0; i < 4; i++) sum += measurementData1.distance_mm[centerIndices[i]];  // Sum center distances
      dist1 = sum / 4.0f;             // Calculate average distance
    }

    // --- Sensor 2 ---
    if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {  // If sensor 2 has data
      const int centerIndices[] = {27, 28, 35, 36};  // Center pixel indices
      float sum = 0;                  // Initialize sum
      for (int i = 0; i < 4; i++) sum += measurementData2.distance_mm[centerIndices[i]];  // Sum center distances
      dist2 = sum / 4.0f;             // Calculate average distance
    }

    // --- Obstacle detection ---
    if ((dist1 > 0 && dist1 < OBSTACLE_THRESHOLD) ||  // If sensor 1 detects obstacle
        (dist2 > 0 && dist2 < OBSTACLE_THRESHOLD)) {  // Or sensor 2 detects obstacle
      newObstacleDetected = true;     // Set temporary obstacle flag
    }

    // Only print message and stop motors if obstacle status changed
    if (newObstacleDetected && !obstacleDetected) {
      obstacleDetected = true;        // Set obstacle flag
      Serial.println("Obstacle detected by ToF! Stopping motors...");
      stop_brush();                   // Stop brush motor
      brake();                        // Stop drive motor
    } 
    else if (!newObstacleDetected && obstacleDetected) {
      obstacleDetected = false;       // Clear obstacle flag
      Serial.println("Obstacle cleared.");
    }
  }
}

/**
 * Updates yaw angle by integrating gyroscope Z-axis data
 */
void updateYawAngle() {
  static unsigned long lastUpdate = 0;  // Last update time
  unsigned long now = millis();         // Current time
  
  if (lastUpdate == 0) {               // If first update
    lastUpdate = now;                  // Set last update time
    return;                            // Exit function
  }
  
  float dt = (now - lastUpdate) / 1000.0; // Convert time difference to seconds
  lastUpdate = now;                    // Update last update time
  
  // Integrate gyroscope Z-axis data (dps to degrees)
  float gyroZ_dps = myICM.gyrZ();      // Get gyro Z value in degrees per second
  currentYaw += gyroZ_dps * dt;        // Integrate to get yaw angle
  
  // Keep yaw between -180 and 180 degrees
  if (currentYaw > 180.0) currentYaw -= 360.0;  // Normalize yaw
  if (currentYaw < -180.0) currentYaw += 360.0; // Normalize yaw
}

// === Fall Detection Function ===
void checkIMU() {
  if (millis() - lastIMUCheck >= IMU_CHECK_INTERVAL && isInitialAngleSet) {  // If time to check and angles are set
    lastIMUCheck = millis();           // Update last check time

    if (myICM.dataReady()) {           // If IMU data is ready
      myICM.getAGMT();                  // Read new data
      
      // Update yaw angle from gyroscope
      updateYawAngle();                // Update yaw angle

      float currentPitch, currentRoll; // Variables for current angles
      // Calculate current tilt from raw accelerometer data
      calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &currentPitch, &currentRoll);

      // Calculate how much we've tilted from the start position
      float deltaPitch = abs(currentPitch - initialPitch);  // Pitch change
      float deltaRoll = abs(currentRoll - initialRoll);     // Roll change
      
      // Handle yaw wrap-around (0-360 degrees)
      float deltaYaw = abs(currentYaw - initialYaw);       // Yaw change
      if (deltaYaw > 180.0) {          // If change is more than 180 degrees
        deltaYaw = 360.0 - deltaYaw;   // Adjust for wrap-around
      }

      // Print for monitoring
      Serial.print("IMU Tilt Δ - Pitch: ");  // Print pitch change
      Serial.print(deltaPitch);
      Serial.print("°, Roll: ");        // Print roll change
      Serial.print(deltaRoll);
      Serial.print("°, Yaw: ");         // Print yaw change
      Serial.print(deltaYaw);
      Serial.println("°");

      // Check if any orientation change is too large
      if (deltaPitch > MAX_DELTA_PITCH_ROLL ||  // If pitch change too large
          deltaRoll > MAX_DELTA_PITCH_ROLL ||   // Or roll change too large
          deltaYaw > MAX_DELTA_YAW) {           // Or yaw change too large
            
        triggerFallProtocol(deltaPitch, deltaRoll, deltaYaw);  // Trigger fall protocol
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
  float accX_g = accX / 1000.0;       // Convert X acceleration to G
  float accY_g = accY / 1000.0;       // Convert Y acceleration to G
  float accZ_g = accZ / 1000.0;       // Convert Z acceleration to G

  // Calculate Pitch (rotation around Y-axis) using arctangent
  *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;

  // Calculate Roll (rotation around X-axis) using arctangent
  *roll = atan2(accY_g, accZ_g) * 180.0 / PI;
}

// === Emergency Fall Protocol ===
void triggerFallProtocol(float dPitch, float dRoll, float dYaw) {
  botDisoriented = true;               // Set disoriented flag
  emergencyStop = true;                // Set emergency stop flag
  Serial.println("!!! FALL DETECTED !!!");  // Print fall detection message
  Serial.print("Orientation Change - Pitch: ");  // Print orientation changes
  Serial.print(dPitch);
  Serial.print("°, Roll: ");
  Serial.print(dRoll);
  Serial.print("°, Yaw: ");
  Serial.print(dYaw);
  Serial.println("°");
  Serial.println("Engaging Emergency Stop...");  // Print emergency stop message

  // EMERGENCY STOP: Halt all motors and actuators
  brake();                            // Stop drive motor
  stop_brush();                       // Stop brush motor
  lin_stop();                         // Stop linear actuator

  // Send emergency status via LoRa
  sendLoRaData();                     // Send data via LoRa

  // Optional: Enter a safe state until reset
  while (true) {                      // Infinite loop
    delay(1000);                      // Wait 1 second
  }
}

/**
 * Prints formatted floating-point numbers with leading zeros
 * @param val Value to print
 * @param leading Total number of digits (including before decimal)
 * @param decimals Number of decimal places
 */
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);              // Get absolute value
  if (val < 0) SERIAL_PORT.print("-");  // Print minus sign if negative
  else SERIAL_PORT.print(" ");        // Print space if positive

  // Print leading zeros
  for (uint8_t i = 0; i < leading; i++) {  // Loop through leading digits
    uint32_t tenpow = pow(10, leading - 1 - i);  // Calculate power of 10
    if (aval < tenpow) SERIAL_PORT.print("0");  // Print zero if needed
    else break;                     // Exit loop if no more zeros needed
  }
  SERIAL_PORT.print(aval, decimals);  // Print value with specified decimals
}

/**
 * Prints formatted IMU data (accelerometer in mg, gyro in dps)
 * @param sensor Pointer to IMU object
 */
void printScaledAGMT(ICM_20948_I2C *sensor) {
  SERIAL_PORT.print("IMU: Acc (mg) [ ");  // Print accelerometer label
  printFormattedFloat(sensor->accX(), 5, 2); SERIAL_PORT.print(", ");  // Print X acceleration
  printFormattedFloat(sensor->accY(), 5, 2); SERIAL_PORT.print(", ");  // Print Y acceleration
  printFormattedFloat(sensor->accZ(), 5, 2); SERIAL_PORT.print(" ] "); // Print Z acceleration

  SERIAL_PORT.print("Gyr (DPS) [ ");  // Print gyroscope label
  printFormattedFloat(sensor->gyrX(), 5, 2); SERIAL_PORT.print(", ");  // Print X rotation
  printFormattedFloat(sensor->gyrY(), 5, 2); SERIAL_PORT.print(", ");  // Print Y rotation
  printFormattedFloat(sensor->gyrZ(), 5, 2); SERIAL_PORT.print(" ]");  // Print Z rotation

  SERIAL_PORT.println();              // Print newline
}

// ===================== LoRa COMMUNICATION =====================

/**
 * Sends ALL sensor data and status flags via LoRa
 * Format: "TOF1:xxx,TOF2:xxx,PITCH:xx.x,ROLL:xx.x,YAW:xx.x,
 *          ACCX:x.xx,ACCY:x.xx,ACCZ:x.xx,GYRX:x.xx,GYRY:x.xx,GYRZ:x.xx,
 *          MAGX:x.xx,MAGY:x.xx,MAGZ:x.xx,TEMP:xx.x,
 *          DRIVE:x,BRUSH:x,ACTUATOR:x,OBSTACLE:x,DISORIENTED:x,EMERGENCY:x"
 */
void sendLoRaData() {
  String loraData = "";               // Initialize data string
  
  // ToF Sensor data
  loraData += "TOF1:" + String(tof1Distance);  // Add ToF sensor 1 data
  loraData += ",TOF2:" + String(tof2Distance); // Add ToF sensor 2 data
  
  // IMU Orientation data
 // loraData += ",PITCH:" + String(imuPitch, 1); // Add pitch angle
 // loraData += ",ROLL:" + String(imuRoll, 1);   // Add roll angle
  //loraData += ",YAW:" + String(imuYaw, 1);     // Add yaw angle
  
  // IMU Raw Accelerometer data
  loraData += ",ACCX:" + String(accelX, 2);    // Add X acceleration
  loraData += ",ACCY:" + String(accelY, 2);    // Add Y acceleration
  loraData += ",ACCZ:" + String(accelZ, 2);    // Add Z acceleration
  
  // IMU Raw Gyroscope data
  loraData += ",GYRX:" + String(gyroX, 2);     // Add X rotation
  loraData += ",GYRY:" + String(gyroY, 2);     // Add Y rotation
  loraData += ",GYRZ:" + String(gyroZ, 2);     // Add Z rotation
  
  // IMU Raw Magnetometer data
  //loraData += ",MAGX:" + String(magX, 2);      // Add X magnetic field
 // loraData += ",MAGY:" + String(magY, 2);      // Add Y magnetic field
  //loraData += ",MAGZ:" + String(magZ, 2);      // Add Z magnetic field
  
  // IMU Temperature data
  //loraData += ",TEMP:" + String(temperature, 1); // Add temperature
  
  // Status flags (1 = true/active, 0 = false/inactive)
  loraData += ",DRIVE:" + String(driveMoving ? 1 : 0);      // Add drive motor status
  loraData += ",BRUSH:" + String(brushMoving ? 1 : 0);      // Add brush motor status
 loraData += ",ACTUATOR:" + String(actuatorMoving ? 1 : 0); // Add actuator status
  loraData += ",OBSTACLE:" + String(obstacleDetected ? 1 : 0);  // Add obstacle status
  loraData += ",DISORIENTED:" + String(botDisoriented ? 1 : 0); // Add disoriented status
  //loraData += ",EMERGENCY:" + String(emergencyStop ? 1 : 0);   // Add emergency stop status

  // Send via LoRa
  LoRa.beginPacket();                 // Start LoRa packet
  LoRa.print(loraData);               // Add data to packet
  LoRa.endPacket();                   // Send packet

  SERIAL_PORT.println("LoRa Sent: " + loraData);  // Print sent data
}

// ===================== WiFi AND MOTOR FUNCTIONS =====================

void handleWiFiClient() {
  client = server.available();        // Check for client connection
  
  if (!client) return;                // Exit if no client

  data = checkClient();               // Get data from client

  if (data.length() == 0) return;     // Exit if no data

  String command = data.substring(0, 1);  // Extract command character
  int value = data.substring(1).toInt();  // Extract value

  Serial.print("Received command: "); // Print received command
  Serial.println(data);

  // Execute command based on received character
  if (command == "f") forward(value); // Move forward
  else if (command == "r") reverse(value); // Move reverse
  else if (command == "b") brake();   // Brake
  else if (command == "c") run_brush(value); // Run brush
  else if (command == "e") run_brush_rev(value); // Run brush reverse
  else if (command == "s") stop_brush(); // Stop brush
  else if (command == "l" || command == "m" || command == "n" || command == "o") move_actuator(value); // Move actuator
  else if (command == "p") lin_stop(); // Stop actuator
  else if (command == "x") resetStatusFlags(); // Reset status flags
  client.stop();                      // Close client connection
}

String checkClient() {
  while (!client.available()) {       // While no data available
    delay(1);                         // Wait briefly
  }
  String request = client.readStringUntil('\r');  // Read until carriage return
  request.remove(0, 5);               // Remove HTTP header
  request.remove(request.length() - 9, 9);  // Remove HTTP footer
  return request;                     // Return processed request
}

void move_actuator(int target_pos) {
  if (actuatorMoving) return;         // Exit if actuator is already moving
  
  new_pos = target_pos;               // Set target position
  dist = abs(pos - new_pos);          // Calculate distance to move
  
  if (dist > 0) {                     // If movement needed
    actuatorMoving = true;            // Set moving flag
    lastActionTime = millis();        // Record start time
    if (pos < new_pos) lin_inc();     // Move increment if target is higher
    else lin_dec();                   // Move decrement if target is lower
  }
}

void handleActuator() {
  if (actuatorMoving && millis() - lastActionTime >= calc_time(dist)) {  // If moving and time elapsed
    lin_stop();                         // Stop actuator
    actuatorMoving = false;             // Clear moving flag
    pos = new_pos;                      // Update current position
  }
}

// Motor Functions
void forward(int pwm) {
  driveMoving = true;                  // Set drive moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(dir_drive, HIGH);       // Set forward direction
  analogWrite(pwm_drive, pwm);         // Set PWM speed
}

void reverse(int pwm) {
  driveMoving = true;                  // Set drive moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(dir_drive, LOW);        // Set reverse direction
  analogWrite(pwm_drive, pwm);         // Set PWM speed
}

void brake() {
  driveMoving = false;                 // Clear drive moving flag
  digitalWrite(dir_drive, LOW);        // Set direction (doesn't matter for brake)
  analogWrite(pwm_drive, 0);           // Set PWM to 0 (brake)
}

void run_brush(int pwm) {
  brushMoving = true;                  // Set brush moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(dir_brush, HIGH);       // Set forward direction
  analogWrite(pwm_brush, pwm);         // Set PWM speed
}

void run_brush_rev(int pwm) {
  brushMoving = true;                  // Set brush moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(dir_brush, LOW);        // Set reverse direction
  analogWrite(pwm_brush, pwm);         // Set PWM speed
}

void stop_brush() {
  brushMoving = false;                 // Clear brush moving flag
  analogWrite(pwm_brush, 0);           // Set PWM to 0 (stop)
}

// Actuator Functions
void lin_inc() {
  actuatorMoving = true;               // Set actuator moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(lin_act_1_dir, LOW);    // Set increment direction
  analogWrite(lin_act_1_pwm, linear_actr_pwm);  // Set PWM speed
}

void lin_dec() {
  actuatorMoving = true;               // Set actuator moving flag
  emergencyStop = false;               // Clear emergency stop if moving
  digitalWrite(lin_act_1_dir, HIGH);   // Set decrement direction
  analogWrite(lin_act_1_pwm, linear_actr_pwm);  // Set PWM speed
}

void lin_stop() {
  actuatorMoving = false;              // Clear actuator moving flag
  analogWrite(lin_act_1_pwm, 0);       // Set PWM to 0 (stop)
}

// === Reset Status Flags ===
void resetStatusFlags() {
  driveMoving = false;
  brushMoving = false;
  actuatorMoving = false;
  obstacleDetected = false;
  // Don't reset botDisoriented and emergencyStop as they need manual reset
  Serial.println("Status flags reset");
}

int calc_time(int dist) {
  return (int)(1000 * dist / 6.893);   // Calculate time needed for movement
}

int calc_pwm(float volt) {
  return (int)((11.0 / volt) * 255);   // Calculate PWM from voltage
}