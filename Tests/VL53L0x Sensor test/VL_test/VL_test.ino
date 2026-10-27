#include <Adafruit_VL53L0X.h>


// Create an instance of the sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // Initialize serial communication at 115200 baud rate
  delay(100);
  Serial.begin(9600);

  // Wait for native USB port to open (needed for boards like Leonardo/Micro/Zero)
  while (!Serial) {      
    delay(1);
  }
  
  Serial.println("Initializing VL53L0X ToF Sensor...");
  
  // Attempt to boot the sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X. Check your wiring!"));
    while(1); // Halt execution if initialization fails
  }

  Serial.println(F("VL53L0X ready! Ready to measure distance.\n"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure; 
    
  Serial.print("Reading sensor... ");
  
  // Perform a distance reading test
  lox.rangingTest(&measure, false); // Pass 'true' to print raw debug metrics instead

  // Phase status '4' represents out of bounds or phase failures
  if (measure.RangeStatus != 4) {  
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" Target out of range ");
  }
    
  // Fetch new data every 200 milliseconds
  delay(200);
}

