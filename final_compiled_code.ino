/* 
 * Dual VL53L5CX Time-of-Flight Sensors + ICM-20948 IMU
 * Reads center 4 pixels from each ToF sensor and full IMU data
 */

// ===================== LIBRARIES =====================
#include <Wire.h>                       // I2C communication library
#include <SparkFun_VL53L5CX_Library.h>  // For VL53L5CX ToF sensors
#include "ICM_20948.h"                  // For ICM-20948 IMU

// ===================== TOF CONFIG =====================
SparkFun_VL53L5CX myImager1;            // First ToF sensor object
SparkFun_VL53L5CX myImager2;            // Second ToF sensor object

int sensorAddress1 = 0x44;              // Custom I2C address for sensor 1
int sensorAddress2 = 0x29;              // Default address for sensor 2

int sensorReset1 = 14;                  // GPIO pin for sensor 1 reset
int sensorReset2 = 13;                  // GPIO pin for sensor 2 reset

VL53L5CX_ResultsData measurementData1;  // Storage for sensor 1 data
VL53L5CX_ResultsData measurementData2;  // Storage for sensor 2 data

int imageResolution = 0;                // Stores resolution (e.g., 64 for 8x8)
int imageWidth = 0;                     // Grid width (e.g., 8)

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

  WIRE_PORT_IMU.begin(5, 6);  // Initialize secondary I2C on pins 5/6
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

  // -------- IMU --------
  if (myICM.dataReady()) {
    myICM.getAGMT();               // Read accelerometer/gyro/mag/temp
    printScaledAGMT(&myICM);       // Print formatted data
  }

  delay(30);  // Short delay to prevent serial flooding
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