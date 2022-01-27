// #include <Wire.h>
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*  Get IMUs sensor data.
   IMU 1: on the cello
     It gets Euler angles of the position and orientation of the cello
     
   IMU 2: on the cello bow
     It gets Euler angles of the position and orietation of the bow
   
   Data from both sensors is collected. The program compares the different orientations
   and checks whether the bow is perpendicular to the cello neck.
   
   ....TO BE FINISHED ONCE THE CODE IS MORE PRECISE
   
   
   
   ---------CONNECTIONS-----------
   IMU 1 (CELLO)
      Connect SCL to analog A5
      Connect SDA to analog A4
      Connect VDD to 3.3-5V DC
      Connect GROUND to common ground

   IMU 2 (BOW)
      Connect SCL to analog A18
      Connect SDA to analog A19
      Connect VDD to 3.3-5V DC
      Connect GROUND to common ground

   MOTORs
      MotorHigher Connect to digital pin 9
      MotorLower Connect to digital pin 8

*/

/*
DO WE NEED THIS???
Set the correction factors for the three Euler angles according to the wanted orientation
float  correction_x = 0; // -177.19;
float  correction_y = 0; // 0.5;
float  correction_z = 0; // 1.25;
*/

/* set the two Wires pins - get data from pins 18 and 19 (Cello IMU) and from pins 37 and 38 (Bow IMU) */
#define WIRE_PINS   I2C_PINS_18_19
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)  // Teensy 3.5/3.6
  #define WIRE1_PINS   I2C_PINS_37_38
#endif

/* set motors pins */

int motorHigher_pin = 9;    // motorHigher activates if the orientation values are too high
int motorLower_pin = 8;     // motorLower activates if the orientation values are too low

int relative_orientation = 0;
int accepted_offset = 15;   // accepted "deviance" angle from perfect orientation


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;


bool reset_calibration = false;  // set to true if you want to redo the calibration rather than using the values stored in the EEPROM
bool display_BNO055_info = false; // set to true if you want to print on the serial port the infromation about the status and calibration of the IMU


/* define 2 IMU sensors */
Adafruit_BNO055 imu_cello = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);
Adafruit_BNO055 imu_bow = Adafruit_BNO055(WIRE1_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_37_38, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);



/**************************************************************************/
/* Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information) */
/**************************************************************************/
void displaySensorDetails(Adafruit_BNO055 imu)
{
    sensor_t sensor;
    imu.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/* Display some basic info about the sensor status */
/**************************************************************************/
void displaySensorStatus(Adafruit_BNO055 imu)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    imu.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/* Display sensor calibration status */
/**************************************************************************/
void displayCalStatus(Adafruit_BNO055 imu)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    imu.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/* Display the raw calibration offset and radius data */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}




/**************************************************************************/
/* Magnetometer calibration */
/**************************************************************************/
void performMagCal(Adafruit_BNO055 imu) {
  
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
 
  while (mag != 3) {
    
    imu.getCalibration(&system, &gyro, &accel, &mag);
    if(display_BNO055_info){
      
      displayCalStatus(imu);
      Serial.println("");
    }
  }
  
  if(display_BNO055_info){

    Serial.println("\nMagnetometer calibrated!");
  }
}  





void setup() {
  /* don't know if this is needed, depends on which processing unit we'll use */
  Serial.begin(115200);
  pinMode(motorHigher_pin, OUTPUT);
  pinMode(motorLower_pin, OUTPUT);

  /* Wire setup -> handle 2 SDAs and SCLs */
  /* Setup for Master mode, all buses, external pullups, 400kHz, 10ms default timeout */  
  Wire.begin(I2C_MASTER, 0x00, WIRE_PINS, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 10ms
  #if I2C_BUS_NUM >= 2
    Wire1.begin(I2C_MASTER, 0x00, WIRE1_PINS, I2C_PULLUP_EXT, 400000);
    Wire1.setDefaultTimeout(10000); // 10ms
  #endif

  
  /* Initialize sensors */
  if(!imu_cello.begin()){
    /* cello IMU was not detected */
    Serial.print("Ooops, no BNO055 on the CELLO detected... Check your wiring or I2C ADDR!");
    while(1);
  }

  if(!imu_bow.begin()){
    /*bow IMU was not detected*/
    Serial.print("Ooops, no BNO055 on the BOW detected... Check your wiring or I2C ADDR!");
    while(1);
  }

  displayCalStatus(imu_cello);
  displayCalStatus(imu_bow);

/*---------------------------------------------------------------------------------------------------------------------------------------*/
  /* re-calibrate if needed */
  int eeAddress_cello = 0;
  int eeAddress_bow = 0;
  long eeIMU_ID_cello;
  long eeIMU_ID_bow;
  long imu_ID_cello;
  long imu_ID_bow;
  bool foundCalib_cello = false;
  bool foundCalib_bow = false;

    
  if(reset_calibration){// Then reset the EEPROM so a new calibration can be made
    
    EEPROM.put(eeAddress_bow, 0);
    eeAddress_bow += sizeof(long);
    EEPROM.put(eeAddress_bow, 0);
    eeAddress_bow = 0;
    if(display_BNO055_info){
      Serial.println("/nEEPROM reset.");
      delay(10000);
    }
  }
  
  EEPROM.get(eeAddress_bow, eeIMU_ID_bow);
  
  adafruit_bno055_offsets_t calibrationData_bow;
  sensor_t sensor_bow;  

    /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  imu_bow.getSensor(&sensor_bow);
  imu_ID_bow = sensor_bow.sensor_id;
    
  if (eeIMU_ID_bow != imu_ID_bow) {
  
    if(display_BNO055_info){
      
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(2000);
    }
  }
  else{

    if(display_BNO055_info){  
       
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
    }
    
    eeAddress_bow += sizeof(long);
    EEPROM.get(eeAddress_bow, calibrationData_bow);

    if(display_BNO055_info){
      
      displaySensorOffsets(calibrationData_bow);
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
    }

    imu_bow.setSensorOffsets(calibrationData_bow);

    if(display_BNO055_info){
      
      Serial.println("\n\nCalibration data loaded into BNO055");
      delay(2000);
    }
    
    foundCalib_bow = true;
  }

  if(display_BNO055_info){
    
    /* Display some basic information on this sensor */
    displaySensorDetails(imu_bow);

    /* Optional: Display current status */
    displaySensorStatus(imu_bow);

  }

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  imu_bow.setExtCrystalUse(true);
      
  
  if (foundCalib_bow){
    
    performMagCal(imu_bow); /* always recalibrate the magnetometers as it goes out of calibration very often */
  }
  else {
    
    if(display_BNO055_info){
      
      Serial.println("Please Calibrate Sensor: ");
      delay(2000); 
    }
        
    while (!imu_bow.isFullyCalibrated()){

      if(display_BNO055_info){
        
            displayCalStatus(imu_bow);
            Serial.println("");
            delay(BNO055_SAMPLERATE_DELAY_MS); // Wait for the specified delay before requesting new data            
        }
    }

    adafruit_bno055_offsets_t newCalib_bow;
    imu_bow.getSensorOffsets(newCalib_bow);
    
    if(display_BNO055_info){

      Serial.println("\nFully calibrated!");
      delay(3000);
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
    
      displaySensorOffsets(newCalib_bow);

      Serial.println("\n\nStoring calibration data to EEPROM...");
    }


    eeAddress_bow = 0;
    EEPROM.put(eeAddress_bow, imu_ID_bow);
    eeAddress_bow += sizeof(long);
    EEPROM.put(eeAddress_bow, newCalib_bow);


    if(display_BNO055_info){
      Serial.println("Data stored to EEPROM.");
      Serial.println("\n--------------------------------\n");
      delay(3000);
      }
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

  delay(1000);
  
  }
}

void loop() {
  /* get cello data */
  sensors_event_t orientation_cello;
  imu_cello.getEvent(&orientation_cello);



  /* print data from cello */
  Serial.print("=========== CELLO ===========\n");
  printEvent(&orientation_cello);
  Serial.print("");

  


  /* get bow data (Euler + gyroscope? Cause it moves a lot) */
  sensors_event_t orientation_bow;
  imu_bow.getEvent(&orientation_bow);

  /* print data from bow */
  Serial.print("=========== BOW ===========\n");
  printEvent(&orientation_bow);
  Serial.print("");


  /* check if the ratio between orientations is correct */
  check_orientation(&relative_orientation, orientation_cello.orientation.x, orientation_bow.orientation.x);
  if(relative_orientation > 0){
      // activate motorHigher
      digitalWrite(motorLower_pin, LOW);
      digitalWrite(motorHigher_pin, HIGH);

  } else if(relative_orientation < 0){
      // activate motorLower
      digitalWrite(motorLower_pin, HIGH);
      digitalWrite(motorHigher_pin, LOW);
  } else {
      // turn off motors
      digitalWrite(motorLower_pin, LOW);
      digitalWrite(motorHigher_pin, LOW);
  }


  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}




/* function to check if bow and cello are perpendicular 
    return
      -1 if COMPLETE AFTER SOME TESTS
      1 
      0
*/
void check_orientation(int* result, float x_orientation_cello, float x_orientation_bow){
  if(x_orientation_bow > (x_orientation_cello + accepted_offset)){
    *result = 1;
  } else if(x_orientation_bow < (x_orientation_cello - accepted_offset)){
    *result = -1;
  } else {
    *result = 0;
  }
  return;
}




/* generic function to print IMU data */
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
