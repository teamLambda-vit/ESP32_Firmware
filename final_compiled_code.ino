#include "ICM_20948.h" 

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port
#define AD0_VAL 1     // I2C address bit (1 = default, 0 if ADR jumper is closed)

ICM_20948_I2C myICM;  // Create I2C object
#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {}; // Wait for serial port

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000); // Set I2C to fast mode (400kHz)

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL); // Initialize I2C communication

    SERIAL_PORT.print(F("Initialization returned: "));
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
  
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.startRanging();
}

void loop()
{
  if (myICM.dataReady())
  {
    myICM.getAGMT(); // Get all sensor data
    
    // Print formatted data with units
    SERIAL_PORT.print("Acc (mg) [ ");
    SERIAL_PORT.print(myICM.accX()); SERIAL_PORT.print(", ");
    SERIAL_PORT.print(myICM.accY()); SERIAL_PORT.print(", ");
    SERIAL_PORT.print(myICM.accZ()); SERIAL_PORT.print(" ] | ");
    
    SERIAL_PORT.print("Gyr (DPS) [ ");
    SERIAL_PORT.print(myICM.gyrX()); SERIAL_PORT.print(", ");
    SERIAL_PORT.print(myICM.gyrY()); SERIAL_PORT.print(", ");
    SERIAL_PORT.print(myICM.gyrZ()); SERIAL_PORT.print(" ] | ");
    
    delay(30);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
  
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      {
      uint16_t minDistance = 65535;

      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
      {
        for (int x = imageWidth - 1; x >= 0; x--)
        {
          uint16_t currentDistance = measurementData.distance_mm[x + y];
          SERIAL_PORT.print("\t");
          SERIAL_PORT.print(currentDistance);

          if (currentDistance > 0 && currentDistance < minDistance)
          {
            minDistance = currentDistance;
          }
        }
        SERIAL_PORT.println();
      }

      SERIAL_PORT.print("Nearest object (min distance): ");
      SERIAL_PORT.print(minDistance);
      SERIAL_PORT.println(" mm\n");
    }

      
    }
  }

  delay(5); //Small delay between polling
}
