/* 
 * Dual VL53L5CX Time-of-Flight Sensors + ICM-20948 IMU + LoRa Communication
 * Reads center 4 pixels from each ToF sensor and full IMU data
 * Includes Fall Detection using IMU with Pitch, Roll, and Yaw
 * Sends data via LoRa to receiver
 * Uses WebServer for HTTP communication
 * Distance-based stopping (stops when distance increases significantly)
 */

// ===================== LIBRARIES =====================
#include <Wire.h>                       // I2C communication library
#include <SparkFun_VL53L5CX_Library.h>  // Distance sensor library
#include "ICM_20948.h"                  // IMU sensor library
#include <WiFi.h>                       // WiFi library for ESP32
#include <WebServer.h>                  // WebServer library
#include <math.h>                       // Math functions library
#include <LoRa.h>                       // LoRa wireless library
#include <SPI.h>                        // SPI communication library
#include "Kalman.h"                     // Kalman filter library

//---------------------------------------------------------------------------------------------------------------------------------------
// WiFi network settings
const char *ssid = "MyESP32_AP";        // WiFi network name
const char *password = "12345678";      // WiFi password

// Create a WebServer on port 80
WebServer server(80);                   // Web server on port 80

// ===================== HARDWARE CONFIG =====================
// LoRa module pin definitions
#define LORA_SS 5                       // LoRa chip select pin
#define LORA_RST 14                     // LoRa reset pin
#define LORA_DIO0 2                     // LoRa interrupt pin

// Motor and actuator control pins
#define pwm_drive 16                    // Drive motor speed pin
#define dir_drive 15                    // Drive motor direction pin
#define pwm_brush 18                    // Brush motor speed pin
#define dir_brush 17                    // Brush motor direction pin
#define lin_act_1_pwm 3                 // Actuator speed pin
#define lin_act_1_dir 6                 // Actuator direction pin

// ===================== TOF CONFIG =====================
SparkFun_VL53L5CX myImager1;            // First distance sensor object
SparkFun_VL53L5CX myImager2;            // Second distance sensor object

int sensorAddress1 = 0x44;              // Custom address for sensor 1
int sensorAddress2 = 0x29;              // Default address for sensor 2

int sensorReset1 = 47;                  // Reset pin for sensor 1
int sensorReset2 = 48;                  // Reset pin for sensor 2

VL53L5CX_ResultsData measurementData1;  // Storage for sensor 1 data
VL53L5CX_ResultsData measurementData2;  // Storage for sensor 2 data

int imageResolution = 0;                // Stores resolution value
int imageWidth = 0;                     // Grid width size

// ====== Distance tracking config ======
unsigned long lastToFCheck = 0;         // Last time distance was checked
int initialTof1Distance = 0;           // Initial distance for sensor 1
int initialTof2Distance = 0;           // Initial distance for sensor 2
bool initialDistanceSet = false;       // Flag if initial distances are set
const uint16_t DISTANCE_TOLERANCE = 100; // 10cm tolerance (100mm)

// ====== FALL DETECTION CONFIG (IMU) ======
#define MAX_DELTA_PITCH_ROLL 20.0       // Max allowed tilt change
#define MAX_DELTA_YAW 45.0              // Max allowed yaw change

unsigned long lastIMUCheck = 0;         // Last time IMU was checked
const uint16_t IMU_CHECK_INTERVAL = 100; // Check every 100ms
unsigned long lastIMUUpdate = 0;        // Last time IMU was updated

// IMU angle variables
float initialPitch = 0.0;               // Starting pitch angle
float initialRoll = 0.0;                // Starting roll angle
float initialYaw = 0.0;                 // Starting yaw angle
float currentPitch = 0.0;               // Current pitch angle
float currentRoll = 0.0;                // Current roll angle
float currentYaw = 0.0;                 // Current yaw angle

// Kalman filters for each axis
Kalman kalmanRoll;                      // Filter for roll angle
Kalman kalmanPitch;                     // Filter for pitch angle
Kalman kalmanYaw;                       // Filter for yaw angle

// Filtered IMU output
float imuPitch = 0.0;                   // Filtered pitch angle
float imuRoll = 0.0;                    // Filtered roll angle
float imuYaw = 0.0;                     // Filtered yaw angle

bool isInitialAngleSet = false;         // Flag if angles are set

// Motor control variables
int rpm = 0;                            // Motor speed value
int pos = 0;                            // Current actuator position
int new_pos = 0;                        // Target actuator position
int dist = 0;                           // Distance to move actuator
int linear_actr_pwm = 255;              // Actuator speed value
long lastMPUCheck = 0;                  // Last time MPU was checked
long lastActionTime = 0;                // Last time action was performed
long lastCurrentCheck = 0;              // Last time current was checked

// ====== ROBOT STATUS FLAGS ======
bool driveMoving = false;               // Is drive motor moving?
bool brushMoving = false;               // Is brush motor moving?
bool actuatorMoving = false;            // Is actuator moving?
bool botDisoriented = false;            // Is robot fallen?
bool obstacleDetected = false;          // Is distance alert active?
bool emergencyStop = false;             // Is emergency stop active?

// Sensor values for LoRa transmission
int tof1Distance = 0;                   // Distance from sensor 1
int tof2Distance = 0;                   // Distance from sensor 2

// IMU raw data values
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;  // Acceleration values
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;     // Rotation values
float magX = 0.0, magY = 0.0, magZ = 0.0;        // Magnetic field values

// LoRa transmission timing
unsigned long lastLoRaTransmission = 0; // Last time data was sent
const uint16_t LORA_INTERVAL = 1000;    // Send data every 1 second

// ===================== IMU CONFIG =====================
#define WIRE_PORT_IMU Wire1             // Use second I2C port for IMU
#define AD0_VAL 1                       // I2C address bit
#define SERIAL_PORT Serial              // Alias for Serial output

ICM_20948_I2C myICM;                    // IMU sensor object

// ===================== SETUP =====================
void setup() {
  // Initialize serial communication
  SERIAL_PORT.begin(115200);            // Start serial at 115200 baud
  delay(1000);                          // Wait 1 second
  SERIAL_PORT.println("Starting Dual ToF + IMU + LoRa Example");
  SERIAL_PORT.println("Distance-based stopping: Robot will stop when distance increases by 10cm");

  // --------- LoRa Initialization ---------
  SERIAL_PORT.println("Initializing LoRa...");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);  // Set LoRa pins
  
  while (!LoRa.begin(433E6)) {          // Try to start LoRa at 433MHz
    SERIAL_PORT.println(".");           // Print dot while trying
    delay(500);                         // Wait 500ms
  }
  LoRa.setSyncWord(0xA5);               // Set LoRa sync word
  SERIAL_PORT.println("LoRa Initializing OK!");

  // --------- I2C Initialization ---------
  Wire.begin();                         // Start first I2C bus
  Wire.setClock(1000000);               // Set fast speed for sensors

  WIRE_PORT_IMU.begin(35,36);           // Start second I2C for IMU
  WIRE_PORT_IMU.setClock(400000);       // Set medium speed for IMU

  // --------- ToF Sensor Initialization ---------
  // Reset sequence for sensor 1
  pinMode(sensorReset2, OUTPUT);        // Set sensor 2 reset pin
  digitalWrite(sensorReset2, HIGH);     // Hold sensor 2 in reset

  pinMode(sensorReset1, OUTPUT);        // Set sensor 1 reset pin
  digitalWrite(sensorReset1, HIGH);     // Reset sensor 1
  delay(100);                           // Wait 100ms
  digitalWrite(sensorReset1, LOW);      // Release sensor 1
  delay(100);                           // Wait 100ms

  // Initialize sensor 1 with custom address
  SERIAL_PORT.println("Initializing ToF Sensor 1");
  if (!myImager1.begin()) {             // Try to start sensor 1
    SERIAL_PORT.println("Sensor 1 not found. Freezing...");
    while (1);                          // Stop if failed
  }
  myImager1.setAddress(sensorAddress1); // Change sensor 1 address

  // Release and initialize sensor 2
  digitalWrite(sensorReset2, LOW);      // Release sensor 2
  delay(100);                           // Wait 100ms
  SERIAL_PORT.println("Initializing ToF Sensor 2");
  if (!myImager2.begin()) {             // Try to start sensor 2
    SERIAL_PORT.println("Sensor 2 not found. Freezing...");
    while (1);                          // Stop if failed
  }
  
  // Configure both sensors
  myImager1.setResolution(8 * 8);       // Set 8x8 grid
  myImager2.setResolution(8 * 8);       // Set 8x8 grid
  imageResolution = myImager1.getResolution();  // Get resolution
  imageWidth = sqrt(imageResolution);   // Calculate width

  myImager1.setRangingFrequency(15);    // Set 15Hz update rate
  myImager2.setRangingFrequency(15);    // Set 15Hz update rate

  myImager1.startRanging();             // Start measuring
  myImager2.startRanging();             // Start measuring

  // --------- IMU Initialization ---------
  bool imuInitialized = false;          // IMU start flag
  while (!imuInitialized) {             // Keep trying until IMU starts
    myICM.begin(WIRE_PORT_IMU, AD0_VAL);  // Try to start IMU
    SERIAL_PORT.print("IMU Init: ");    // Print status
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok) {  // If failed
      SERIAL_PORT.println("Retrying IMU..."); // Print retry message
      delay(500);                       // Wait 500ms
    } else {
      imuInitialized = true;            // Set flag if success
    }
  }

  // --------- Set Initial Reference Angle for Fall Detection ---------
  SERIAL_PORT.println("Place robot on panel. Calculating initial tilt in 3 seconds...");
  delay(3000);                          // Wait 3 seconds

  // Take a reading to set the initial angle
  if (myICM.dataReady()) {              // If IMU data ready
    myICM.getAGMT();                    // Read sensor data
    calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &initialPitch, &initialRoll);  // Calculate tilt
  
    // ----- Compute averaged magnetometer heading -----
    float sumHeading = 0;
    const int samples = 10;               // number of samples for averaging
    for (int i = 0; i < samples; i++) {
        myICM.getAGMT();                  // Read fresh IMU data
        float h = atan2(myICM.magY(), myICM.magX()) * 180.0f / PI;  // raw heading
        if (h < 0) h += 360.0f;           // normalize 0..360
        sumHeading += h;
        delay(50);                        // small delay between readings
    }
    float initialMagHeading = sumHeading / samples; // averaged heading

    // Set initial yaw
    initialYaw = initialMagHeading;
    currentYaw = initialMagHeading;

    // Initialize Kalman filter
    kalmanYaw.setAngle(initialYaw);
  
    isInitialAngleSet = true;           // Mark angles as set
    // initialize Kalman filters with reference angles
    kalmanRoll.setAngle(initialRoll);
    kalmanPitch.setAngle(initialPitch);

    SERIAL_PORT.print("Reference Angles Set - Pitch: ");  // Print angles
    SERIAL_PORT.print(initialPitch);
    SERIAL_PORT.print("°, Roll: ");
    SERIAL_PORT.print(initialRoll);
    SERIAL_PORT.print("°, Yaw: ");
    SERIAL_PORT.print(initialYaw);
    SERIAL_PORT.println("°");
  }

  // Pin Modes for motor control
  pinMode(dir_drive, OUTPUT);           // Set drive direction pin
  pinMode(pwm_drive, OUTPUT);           // Set drive speed pin
  pinMode(dir_brush, OUTPUT);           // Set brush direction pin
  pinMode(pwm_brush, OUTPUT);           // Set brush speed pin
  pinMode(lin_act_1_pwm, OUTPUT);       // Set actuator speed pin
  pinMode(lin_act_1_dir, OUTPUT);       // Set actuator direction pin

  // Reset all status flags
  resetStatusFlags();
  
  //----------------------------------------------------------------------------------------------------------------------------------------------
  // WiFi Setup with WebServer
  SERIAL_PORT.println("Starting Access Point...");

  // Start AP
  WiFi.softAP(ssid, password);          // Start access point

  SERIAL_PORT.println("Access Point Started");
  SERIAL_PORT.print("IP Address: ");
  SERIAL_PORT.println(WiFi.softAPIP()); // Print IP address

  // Setup server routes
  server.on("/", handleRoot);           // Root page
  server.on("/data", handleData);       // Data endpoint for commands
  server.on("/status", handleStatus);   // Status endpoint
  
  server.begin();                       // Start web server
  SERIAL_PORT.println("HTTP server started");
  SERIAL_PORT.println("Send commands to: http://" + WiFi.softAPIP().toString() + "/data?cmd=COMMAND");
  SERIAL_PORT.println("Initial distances will be set automatically from first sensor readings");
}

// ===================== MAIN LOOP =====================
void loop() {
  server.handleClient();                // Handle HTTP requests

  // -------- ToF Sensor 1 --------
  if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {  // If sensor 1 has data
    tof1Distance = printToFData(measurementData1);  // Get distance
  }

  // -------- ToF Sensor 2 --------
  if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {  // If sensor 2 has data
    tof2Distance = printToFData(measurementData2);  // Get distance
  }

  // -------- Distance Change Check --------
  checkDistanceChange();               // Check for significant distance increases

  // -------- IMU Data Reading --------
  if (myICM.dataReady()) {            // If IMU data ready
    myICM.getAGMT();                  // Read sensor data
    
    // Store raw IMU data for transmission
    accelX = myICM.accX() / 1000.0;   // Convert to G
    accelY = myICM.accY() / 1000.0;
    accelZ = myICM.accZ() / 1000.0;
    gyroX = myICM.gyrX();             // Degrees per second
    gyroY = myICM.gyrY();
    gyroZ = myICM.gyrZ();
    magX = myICM.magX();              // Raw magnetometer
    magY = myICM.magY();
    magZ = myICM.magZ();
    
    printScaledAGMT(&myICM);          // Print data
    
    // ===== Kalman + magnetometer fusion =====
    unsigned long now = millis();     // Current time
    float dt = 0.0;                   // Time difference
    if (lastIMUUpdate == 0) {
      // first iteration: fall back to IMU_CHECK_INTERVAL
      dt = IMU_CHECK_INTERVAL / 1000.0f;
    } else {
      dt = (now - lastIMUUpdate) / 1000.0f;  // Calculate time difference
    }
    lastIMUUpdate = now;              // Update time

    // 1) accelerometer-based angles (degrees)
    float accRoll  = atan2(accelY, accelZ) * 180.0f / PI;    // Calculate roll
    float accPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0f / PI;  // Calculate pitch

    // 2) fuse accel + gyro for roll & pitch via Kalman
    float fusedRoll  = kalmanRoll.getAngle(accRoll, gyroX, dt);   // Filtered roll
    float fusedPitch = kalmanPitch.getAngle(accPitch, gyroY, dt); // Filtered pitch

    // 3) tilt-compensate magnetometer and compute heading
    float rollRad  = fusedRoll  * (PI / 180.0f);             // Convert to radians
    float pitchRad = fusedPitch * (PI / 180.0f);             // Convert to radians

    // Use raw magnetometer readings
    float mx = magX;
    float my = magY;
    float mz = magZ;

    // Tilt compensation
    float Xh = mx * cos(pitchRad) + mz * sin(pitchRad);      // Compensated X
    float Yh = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);  // Compensated Y

    // heading from tilt-compensated mag (0..360)
    float magHeading = atan2(Yh, Xh) * 180.0f / PI;          // Calculate heading
    if (magHeading < 0.0f) magHeading += 360.0f;             // Normalize

    static float prevMagHeading = initialYaw;
    magHeading = 0.9 * prevMagHeading + 0.1 * magHeading;    // Low-pass filter
    prevMagHeading = magHeading;

    // 4) fuse magnetometer heading + gyroZ with Kalman for yaw
    float fusedYaw = kalmanYaw.getAngle(magHeading, gyroZ, dt);  // Filtered yaw

    // normalize yaw to -180..180
    if (fusedYaw > 180.0f) fusedYaw -= 360.0f;               // Normalize
    if (fusedYaw < -180.0f) fusedYaw += 360.0f;              // Normalize

    // 5) write fused results
    currentRoll  = fusedRoll;                                // Update roll
    currentPitch = fusedPitch;                               // Update pitch
    currentYaw   = fusedYaw;                                 // Update yaw
  }

  // -------- Fall Detection Check --------
  checkIMU();                         // Check for falls

  // -------- Handle Actuator Movement --------
  handleActuator();                   // Handle actuator

  // -------- Send Data via LoRa --------
  if (millis() - lastLoRaTransmission >= LORA_INTERVAL) {  // If time to send
    sendLoRaData();                    // Send data
    lastLoRaTransmission = millis();   // Update time
  }

  delay(100);                          // Short delay
}

// ===================== HTTP HANDLERS =====================

/**
 * Handle root page request
 */
void handleRoot() {
  String html = "<html><head><title>Robot Controller</title></head><body>";
  html += "<h1>Robot Controller</h1>";
  html += "<p>Send commands to: /data?cmd=COMMAND</p>";
  html += "<p>Available commands: fm (forward), rm (reverse), b (brake), ";
  html += "fb (brush forward), rb (brush reverse), s (stop brush), ";
  html += "fl (actuator forward), rl (actuator reverse), p (stop actuator), ";
  html += "x (reset flags), resetdist (reset initial distances)</p>";
  html += "<p>Format: command + value (e.g., fm255, rm120, fb200)</p>";
  html += "<p><strong>Distance Monitoring:</strong> Robot stops when distance increases by 10cm from initial reading</p>";
  html += "<p><a href='/status'>Check Status</a></p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

/**
 * Handle data commands
 */
void handleData() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd parameter");
    return;
  }

  String data = server.arg("cmd");    // Get command like "fm255"
  
  if (data.length() == 0) {
    server.send(400, "text/plain", "Empty command");
    return;
  }

  // Extract command letter and value
  String commandPart = "";
  String valuePart = "";
  
  for (int i = 0; i < data.length(); i++) {
    if (isAlpha(data.charAt(i))) {
      commandPart += data.charAt(i);
    } else if (isDigit(data.charAt(i))) {
      valuePart += data.charAt(i);
    }
  }
  
  int value = valuePart.toInt();

  SERIAL_PORT.print("Received HTTP command: ");
  SERIAL_PORT.print(commandPart);
  SERIAL_PORT.print(" with value: ");
  SERIAL_PORT.println(value);

  // Execute command
  if (commandPart == "fm") forward(value);
  else if (commandPart == "rm") reverse(value);
  else if (commandPart == "b") brake();
  else if (commandPart == "fb") run_brush(value);
  else if (commandPart == "rb") run_brush_rev(value);
  else if (commandPart == "s") stop_brush();
  else if (commandPart == "fl") forward_actuator(value);
  else if (commandPart == "rl") reverse_actuator(value);
  else if (commandPart == "p") lin_stop();
  else if (commandPart == "x") resetStatusFlags();
  else if (commandPart == "resetdist") {
    resetInitialDistances();
    server.send(200, "text/plain", "Initial distances reset");
    return;
  }
  else {
    server.send(400, "text/plain", "Invalid command: " + commandPart);
    return;
  }

  server.send(200, "text/plain", "OK - Command: " + commandPart + " Value: " + String(value));
}

/**
 * Handle status request
 */
void handleStatus() {
  String status = "Robot Status:\n";
  status += "Drive Moving: " + String(driveMoving ? "YES" : "NO") + "\n";
  status += "Brush Moving: " + String(brushMoving ? "YES" : "NO") + "\n";
  status += "Actuator Moving: " + String(actuatorMoving ? "YES" : "NO") + "\n";
  status += "Distance Alert: " + String(obstacleDetected ? "YES" : "NO") + "\n";
  status += "Bot Disoriented: " + String(botDisoriented ? "YES" : "NO") + "\n";
  status += "Emergency Stop: " + String(emergencyStop ? "YES" : "NO") + "\n";
  status += "ToF1 Current: " + String(tof1Distance) + "mm\n";
  status += "ToF2 Current: " + String(tof2Distance) + "mm\n";
  status += "ToF1 Initial: " + String(initialTof1Distance) + "mm\n";
  status += "ToF2 Initial: " + String(initialTof2Distance) + "mm\n";
  status += "Distance Set: " + String(initialDistanceSet ? "YES" : "NO") + "\n";
  status += "Tolerance: " + String(DISTANCE_TOLERANCE) + "mm\n";
  status += "Pitch: " + String(currentPitch, 1) + "°\n";
  status += "Roll: " + String(currentRoll, 1) + "°\n";
  status += "Yaw: " + String(currentYaw, 1) + "°\n";
  
  server.send(200, "text/plain", status);
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Prints average distance of center 4 pixels in centimeters and returns value
 */
int printToFData(VL53L5CX_ResultsData &data) {
  const int centerIndices[] = {27, 28, 35, 36};  // Center pixel positions
  float sum = 0;                    // Initialize sum
  
  // Sum distances from center pixels
  for (int i = 0; i < 4; i++) {    // Loop through 4 pixels
    sum += data.distance_mm[centerIndices[i]];  // Add distance
  }
  
  int avgDistance = sum / 4.0f;     // Calculate average
  return avgDistance;               // Return average
}

/**
 * Check for significant distance increases using ToF sensors
 * Stops robot if current distance > initial distance + tolerance
 */
void checkDistanceChange() {
  if (millis() - lastToFCheck >= 100) {  // If 100ms passed
    lastToFCheck = millis();            // Update time

    int currentDist1 = -1;              // Current sensor 1 distance
    int currentDist2 = -1;              // Current sensor 2 distance
    bool shouldStop = false;            // Flag to stop robot

    // --- Sensor 1 ---
    if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {
      const int centerIndices[] = {27, 28, 35, 36};
      float sum = 0;
      for (int i = 0; i < 4; i++) sum += measurementData1.distance_mm[centerIndices[i]];
      currentDist1 = sum / 4.0f;
      
      // Set initial distance if not set
      if (!initialDistanceSet && currentDist1 > 0) {
        initialTof1Distance = currentDist1;
        Serial.print("Initial ToF1 distance set to: ");
        Serial.print(initialTof1Distance);
        Serial.println("mm");
      }
    }

    // --- Sensor 2 ---
    if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {
      const int centerIndices[] = {27, 28, 35, 36};
      float sum = 0;
      for (int i = 0; i < 4; i++) sum += measurementData2.distance_mm[centerIndices[i]];
      currentDist2 = sum / 4.0f;
      
      // Set initial distance if not set
      if (!initialDistanceSet && currentDist2 > 0) {
        initialTof2Distance = currentDist2;
        Serial.print("Initial ToF2 distance set to: ");
        Serial.print(initialTof2Distance);
        Serial.println("mm");
        
        // Mark initial distances as set once both are read
        if (initialTof1Distance > 0 && initialTof2Distance > 0) {
          initialDistanceSet = true;
          Serial.println("Initial distances set for both sensors");
          Serial.print("Tolerance: ");
          Serial.print(DISTANCE_TOLERANCE);
          Serial.println("mm");
        }
      }
    }

    // --- Check distance changes only after initial distances are set ---
    if (initialDistanceSet) {
      // Check if current distance exceeds initial distance + tolerance
      if (currentDist1 > 0 && currentDist1 > (initialTof1Distance + DISTANCE_TOLERANCE)) {
        Serial.print("ToF1 distance increased: ");
        Serial.print(currentDist1);
        Serial.print("mm > ");
        Serial.print(initialTof1Distance + DISTANCE_TOLERANCE);
        Serial.println("mm (initial + tolerance)");
        shouldStop = true;
      }
      
      if (currentDist2 > 0 && currentDist2 > (initialTof2Distance + DISTANCE_TOLERANCE)) {
        Serial.print("ToF2 distance increased: ");
        Serial.print(currentDist2);
        Serial.print("mm > ");
        Serial.print(initialTof2Distance + DISTANCE_TOLERANCE);
        Serial.println("mm (initial + tolerance)");
        shouldStop = true;
      }

      // Only take action if status changed
      if (shouldStop && !obstacleDetected) {
        obstacleDetected = true;
        Serial.println("Distance increased significantly! Stopping motors...");
        stop_brush();                   // Stop brush
        brake();                        // Stop drive
      } 
      else if (!shouldStop && obstacleDetected) {
        obstacleDetected = false;
        Serial.println("Distance returned to normal range.");
      }
    }
  }
}

/**
 * Reset initial distances to current readings
 */
void resetInitialDistances() {
  initialDistanceSet = false;
  initialTof1Distance = 0;
  initialTof2Distance = 0;
  obstacleDetected = false;
  Serial.println("Initial distances reset. New distances will be set on next reading.");
}

/**
 * Updates yaw angle by integrating gyroscope Z-axis data
 */
void updateYawAngle() {
  static unsigned long lastUpdate = 0;  // Last update time
  unsigned long now = millis();         // Current time
  
  if (lastUpdate == 0) {               // If first time
    lastUpdate = now;                  // Set time
    return;                            // Exit
  }
  
  float dt = (now - lastUpdate) / 1000.0; // Time difference in seconds
  lastUpdate = now;                    // Update time
  
  // Integrate gyroscope data
  float gyroZ_dps = myICM.gyrZ();      // Get rotation speed
  currentYaw += gyroZ_dps * dt;        // Calculate new yaw
  
  // Keep yaw between -180 and 180
  if (currentYaw > 180.0) currentYaw -= 360.0;  // Normalize
  if (currentYaw < -180.0) currentYaw += 360.0; // Normalize
}

/**
 * Fall Detection Function
 */
void checkIMU() {
  if (millis() - lastIMUCheck >= IMU_CHECK_INTERVAL && isInitialAngleSet) {  // If time to check
    lastIMUCheck = millis();           // Update time

    if (myICM.dataReady()) {           // If data ready
      myICM.getAGMT();                  // Read data
      
      // Update yaw angle
      updateYawAngle();                // Update yaw

      float currentPitch, currentRoll; // Current angles
      // Calculate current tilt
      calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &currentPitch, &currentRoll);

      // Calculate tilt change
      float deltaPitch = abs(currentPitch - initialPitch);  // Pitch change
      float deltaRoll = abs(currentRoll - initialRoll);     // Roll change
      
      // Handle yaw wrap-around
      float deltaYaw = abs(currentYaw - initialYaw);       // Yaw change
      if (deltaYaw > 180.0) {          // If over 180
        deltaYaw = 360.0 - deltaYaw;   // Adjust
      }

      // Check if change too large
      if (deltaPitch > MAX_DELTA_PITCH_ROLL ||  // If pitch too much
          deltaRoll > MAX_DELTA_PITCH_ROLL ||   // Or roll too much
          deltaYaw > MAX_DELTA_YAW) {           // Or yaw too much
            
        triggerFallProtocol(deltaPitch, deltaRoll, deltaYaw);  // Trigger fall
      }
    }
  }
}

/**
 * Calculates Pitch and Roll angles in degrees from raw accelerometer data
 */
void calculateTilt(int16_t accX, int16_t accY, int16_t accZ, float *pitch, float *roll) {
  // Convert to G-Force
  float accX_g = accX / 1000.0;       // X acceleration in G
  float accY_g = accY / 1000.0;       // Y acceleration in G
  float accZ_g = accZ / 1000.0;       // Z acceleration in G

  // Calculate Pitch
  *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;

  // Calculate Roll
  *roll = atan2(accY_g, accZ_g) * 180.0 / PI;
}

/**
 * Emergency Fall Protocol
 */
void triggerFallProtocol(float dPitch, float dRoll, float dYaw) {
  botDisoriented = true;               // Set fallen flag
  emergencyStop = true;                // Set emergency flag
  Serial.println("!!! FALL DETECTED !!!");  // Print message
  Serial.print("Orientation Change - Pitch: ");  // Print changes
  Serial.print(dPitch);
  Serial.print("°, Roll: ");
  Serial.print(dRoll);
  Serial.print("°, Yaw: ");
  Serial.print(dYaw);
  Serial.println("°");
  Serial.println("Engaging Emergency Stop...");  // Print stop message

  // Stop all motors
  brake();                            // Stop drive
  stop_brush();                       // Stop brush
  lin_stop();                         // Stop actuator

  // Send emergency status
  sendLoRaData();                     // Send data

  // Stay in safe state
  while (true) {                      // Infinite loop
    delay(1000);                      // Wait 1 second
  }
}

/**
 * Prints formatted IMU data
 */
void printScaledAGMT(ICM_20948_I2C *sensor) {
  /*Serial.print("Scaled. Acc (G) [ ");
  Serial.print(sensor->accX() / 1000.0, 2);
  Serial.print(", ");
  Serial.print(sensor->accY() / 1000.0, 2);
  Serial.print(", ");
  Serial.print(sensor->accZ() / 1000.0, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  Serial.print(sensor->gyrX(), 2);
  Serial.print(", ");
  Serial.print(sensor->gyrY(), 2);
  Serial.print(", ");
  Serial.print(sensor->gyrZ(), 2);
  Serial.print(" ], Mag (uT) [ ");
  Serial.print(sensor->magX(), 2);
  Serial.print(", ");
  Serial.print(sensor->magY(), 2);
  Serial.print(", ");
  Serial.print(sensor->magZ(), 2);
  Serial.println(" ]");*/
}

/**
 * Sends ALL sensor data and status flags via LoRa
 */
void sendLoRaData() {
  String loraData = "";               // Initialize data string
  
  // ToF Sensor data
  loraData += "TOF1:" + String(tof1Distance);  // Add sensor 1
  loraData += ",TOF2:" + String(tof2Distance); // Add sensor 2
  
  // IMU Orientation data
  loraData += ",PITCH:" + String(currentPitch, 1); // Add pitch
  loraData += ",ROLL:" + String(currentRoll, 1);   // Add roll
  loraData += ",YAW:" + String(currentYaw, 1);     // Add yaw
  
  // IMU Raw Accelerometer data
  loraData += ",ACCX:" + String(accelX, 2);    // Add X acceleration
  loraData += ",ACCY:" + String(accelY, 2);    // Add Y acceleration
  loraData += ",ACCZ:" + String(accelZ, 2);    // Add Z acceleration
  
  // IMU Raw Gyroscope data
  loraData += ",GYRX:" + String(gyroX, 2);     // Add X rotation
  loraData += ",GYRY:" + String(gyroY, 2);     // Add Y rotation
  loraData += ",GYRZ:" + String(gyroZ, 2);     // Add Z rotation
  
  // IMU Raw Magnetometer data
  loraData += ",MAGX:" + String(magX, 2);      // Add X magnetic
  loraData += ",MAGY:" + String(magY, 2);      // Add Y magnetic
  loraData += ",MAGZ:" + String(magZ, 2);      // Add Z magnetic
  
  // Status flags
  loraData += ",DRIVE:" + String(driveMoving ? 1 : 0);      // Drive status
  loraData += ",BRUSH:" + String(brushMoving ? 1 : 0);      // Brush status
  loraData += ",ACTUATOR:" + String(actuatorMoving ? 1 : 0); // Actuator status
  loraData += ",DIST_ALERT:" + String(obstacleDetected ? 1 : 0);  // Distance alert status
  loraData += ",DISORIENTED:" + String(botDisoriented ? 1 : 0); // Fallen status
  loraData += ",EMERGENCY:" + String(emergencyStop ? 1 : 0);   // Emergency status

  // Send via LoRa
  LoRa.beginPacket();                 // Start packet
  LoRa.print(loraData);               // Add data
  LoRa.endPacket();                   // Send packet

  SERIAL_PORT.println("LoRa Sent: " + loraData);  // Print data
}

// ===================== MOTOR CONTROL FUNCTIONS =====================

void forward(int pwm) {
  driveMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_drive, HIGH);       // Set forward
  analogWrite(pwm_drive, pwm);         // Set speed
}

void reverse(int pwm) {
  driveMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_drive, LOW);        // Set reverse
  analogWrite(pwm_drive, pwm);         // Set speed
}

void brake() {
  driveMoving = false;                 // Clear flag
  digitalWrite(dir_drive, LOW);        // Set direction
  analogWrite(pwm_drive, 0);           // Stop
}

void run_brush(int pwm) {
  brushMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_brush, HIGH);       // Set forward
  analogWrite(pwm_brush, pwm);         // Set speed
}

void run_brush_rev(int pwm) {
  brushMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_brush, LOW);        // Set reverse
  analogWrite(pwm_brush, pwm);         // Set speed
}

void stop_brush() {
  brushMoving = false;                 // Clear flag
  analogWrite(pwm_brush, 0);           // Stop
}

void handleActuator() {
  if (actuatorMoving && millis() - lastActionTime >= calc_time(dist)) {  // If time elapsed
    lin_stop();                         // Stop
    actuatorMoving = false;             // Clear flag
    pos = new_pos;                      // Update position
  }
}

void forward_actuator(int pwm) {
  actuatorMoving = true;               // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(lin_act_1_dir, LOW);    // Set up
  analogWrite(lin_act_1_pwm, pwm);     // Set speed
}

void reverse_actuator(int pwm) {
  actuatorMoving = true;               // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(lin_act_1_dir, HIGH);   // Set down
  analogWrite(lin_act_1_pwm, pwm);     // Set speed
}

void lin_stop() {
  actuatorMoving = false;              // Clear flag
  analogWrite(lin_act_1_pwm, 0);       // Stop
}

/**
 * Reset Status Flags
 */
void resetStatusFlags() {
  driveMoving = false;                 // Clear drive flag
  brushMoving = false;                 // Clear brush flag
  actuatorMoving = false;              // Clear actuator flag
  obstacleDetected = false;            // Clear distance alert flag
  // Don't reset fallen and emergency flags
  Serial.println("Status flags reset"); // Print message
}

/**
 * Calculate time needed for actuator movement
 */
int calc_time(int dist) {
  return (int)(1000 * dist / 6.893);   // Calculate time needed
}

/**
 * Calculate PWM from voltage
 */
int calc_pwm(float volt) {
  return (int)((11.0 / volt) * 255);   // Calculate PWM from voltage
}