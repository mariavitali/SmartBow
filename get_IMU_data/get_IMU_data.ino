// #include <Wire.h>
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* Get IMUs sensor data.
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
      MotorHigher Connect to digital pin 30
      MotorLower Connect to digital pin 29

   BUTTONs
      onOffButton Connect to digital pin 4
      recalibrationButton Connect to digital pin 8
      setZeroButton Connect to digital pin 10

   LED
      greenLED (OnOffButton) Connect to digital pin 3
      redLED (RecalibrationButton) Connect to digital pin 7
      blueLED (SetZeroButton) Connect to digital pin 9

*/


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

/*------------------------------MOTORs------------------------------------------------------------------------------------------------------------*/

/* set motors pins */
int motorHigher_pin = 30;    // motorHigher activates if the orientation values are too high
int motorLower_pin = 29;     // motorLower activates if the orientation values are too low


/*------------------------------LED------------------------------------------------------------------------------------------------------------*/

int greenLED_pin = 3;
int redLED_pin = 7;
int blueLED_pin = 9;

int greenLEDState = LOW;      // the current state of the output pin
int redLEDState = LOW;        // the current state of the output pin
int blueLEDState = LOW;       // the current state of the output pin


/*------------------------------BUTTONs------------------------------------------------------------------------------------------------------------*/

int onOffButton_pin = 4;
int recalibrationButton_pin = 8;
int setZeroButton_pin = 10;

int onOffButtonState;                     // the current reading from the input pin
int recalibrationButtonState;             // the current reading from the input pin
int setZeroButtonState;                   // the current reading from the input pin

int lastOnOffButtonState = LOW;           // the previous reading from the input pin
int lastRecalibrationButtonState = LOW;   // the previous reading from the input pin
int lastSetZeroButtonState = LOW;         // the previous reading from the input pin

unsigned long lastOnOffButtonDebounceTime = 0;          // the last time the output pin was toggled
unsigned long lastRecalibrationButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastSetZeroButtonDebounceTime = 0;        // the last time the output pin was toggled

/*------------------------------IMUs---------------------------------------------------------------------------------------------------------------*/

/* set the two Wires pins - get data from pins 18 and 19 (Cello IMU) and from pins 37 and 38 (Bow IMU) */
#define WIRE_PINS   I2C_PINS_18_19
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)  // Teensy 3.5/3.6
#define WIRE1_PINS   I2C_PINS_37_38
#endif

/* define 2 IMU sensors */
#define N_CELLO 0
#define N_BOW 1
Adafruit_BNO055 imu_cello = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);
Adafruit_BNO055 imu_bow = Adafruit_BNO055(WIRE1_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_37_38, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_ISR);


/*------------------------------CALIBRATION------------------------------------------------------------------------------------------------------*/

/* re-calibration */
bool reset_calibration = false;  // set to true if you want to redo the calibration rather than using the values stored in the EEPROM
bool display_BNO055_info = true; // set to true if you want to print on the serial port the infromation about the status and calibration of the IMU


/*--------------------------------------------------------------------------------------------------------------------------------------------------*/



/* zero value of bow and cello */
float zero_offset_cello = 0;
float zero_offset_bow = 0;

int accepted_offset = 15;   // accepted "deviance" angle from perfect orientation
float angle_difference = 0;



void setup() {
  // Serial.begin(115200);
  pinMode(motorHigher_pin, OUTPUT);
  pinMode(motorLower_pin, OUTPUT);

  pinMode(onOffButton_pin, INPUT);
  pinMode(recalibrationButton_pin, INPUT);
  pinMode(setZeroButton_pin, INPUT);

  pinMode(greenLED_pin, OUTPUT);
  pinMode(redLED_pin, OUTPUT);
  pinMode(blueLED_pin, OUTPUT);

  // set initial LED state
  digitalWrite(greenLED_pin, greenLEDState);
  digitalWrite(redLED_pin, redLEDState);
  digitalWrite(blueLED_pin, blueLEDState);


  /* Wire setup -> handle 2 SDAs and SCLs */
  /* Setup for Master mode, all buses, external pullups, 400kHz, 10ms default timeout */
  Wire.begin(I2C_MASTER, 0x00, WIRE_PINS, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 10ms
#if I2C_BUS_NUM >= 2
  Wire1.begin(I2C_MASTER, 0x00, WIRE1_PINS, I2C_PULLUP_EXT, 400000);
  Wire1.setDefaultTimeout(10000); // 10ms
#endif


  /* Initialize sensors */
  if (!imu_cello.begin()) {
    /* cello IMU was not detected */
    Serial.print("Ooops, no BNO055 on the CELLO detected... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!imu_bow.begin()) {
    /*bow IMU was not detected*/
    Serial.print("Ooops, no BNO055 on the BOW detected... Check your wiring or I2C ADDR!");
    while (1);
  }

  displayCalStatus(imu_cello);
  displayCalStatus(imu_bow);


  imu_calibration(imu_cello, N_CELLO, reset_calibration);
  imu_calibration(imu_bow, N_BOW, reset_calibration);

  delay(1000);

}


void loop() {
  /* check if buttons are pressed */
  button_pressed(onOffButton_pin, &onOffButtonState, &lastOnOffButtonState, &lastOnOffButtonDebounceTime, greenLED_pin, &greenLEDState);
  button_pressed(recalibrationButton_pin, &recalibrationButtonState, &lastRecalibrationButtonState, &lastRecalibrationButtonDebounceTime, redLED_pin, &redLEDState);
  button_pressed(setZeroButton_pin, &setZeroButtonState, &lastSetZeroButtonState, &lastSetZeroButtonDebounceTime, blueLED_pin, &blueLEDState);


  /* if recalibrationButton is pressed, recalibrate sensors */
  if (digitalRead(redLED_pin) == HIGH) {
    reset_calibration = true;
    imu_calibration(imu_cello, N_CELLO, reset_calibration);
    imu_calibration(imu_bow, N_BOW, reset_calibration);

    reset_calibration = false;

    // turn off led after calibration is over
    digitalWrite(redLED_pin, LOW);
    redLEDState = !redLEDState;
  }


  /* get cello data */
  sensors_event_t orientation_cello;
  imu_cello.getEvent(&orientation_cello);


  /* get bow data */
  sensors_event_t orientation_bow;
  imu_bow.getEvent(&orientation_bow);

  /* check if setZeroButton is pressed. If pressed, reset zero */
  if (digitalRead(blueLED_pin) == HIGH) {
    zero_offset_cello = orientation_cello.orientation.x;
    zero_offset_bow = orientation_bow.orientation.x;
    delay (1000);
    // turn off led
    digitalWrite(blueLED_pin, LOW);
    blueLEDState = !blueLEDState;
  }

  /* set "zero" value */
  // fix cello value
  orientation_cello.orientation.x = (orientation_cello.orientation.x - zero_offset_cello);
  
  // compute bow value, taking into consideration the "jump" from 359 to 0 degree value
  float new_orientation_bow = orientation_bow.orientation.x - zero_offset_bow;
  
  if(zero_offset_bow >= 180){
    if((new_orientation_bow >= ((-1) * zero_offset_bow)) && (new_orientation_bow <= (-180))){
      orientation_bow.orientation.x = new_orientation_bow + 360;
    } 
    else {
      orientation_bow.orientation.x = new_orientation_bow;
    }
  }
  else {                                    // zero_offset_bow < 180
      if(new_orientation_bow > 180){
        orientation_bow.orientation.x = new_orientation_bow - 360;
      }
      else {
        orientation_bow.orientation.x = new_orientation_bow;
      }
  }


  /* 
   REMOVE COMMENT IF ORIENTATION DATA PRINT NEEDED
  // print data from cello
  Serial.println("=========== CELLO ===========");
  printEvent(&orientation_cello);
  Serial.print("");


  // print data from bow
  Serial.println("=========== BOW ===========");
  printEvent(&orientation_bow);
  Serial.print("");
  */



  angle_difference = orientation_bow.orientation.x - orientation_cello.orientation.x;
  
  if (angle_difference > accepted_offset) {
      // activate motorHigher
      digitalWrite(motorLower_pin, LOW);
      digitalWrite(motorHigher_pin, digitalRead(greenLED_pin));
    } else if (angle_difference < ((-1) * accepted_offset)) {
      // activate motorLower
      digitalWrite(motorHigher_pin, LOW);
      digitalWrite(motorLower_pin, digitalRead(greenLED_pin));
    } else {
      // turn off motors
      digitalWrite(motorLower_pin, LOW);
      digitalWrite(motorHigher_pin, LOW);
    }


  Serial.print("angleDifference ");
  Serial.println(angle_difference);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}


/* modulo operator - always returns positive remainder */
int mod( int x, int y ) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
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
