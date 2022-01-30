/* re-calibrate if needed */
void imu_calibration(Adafruit_BNO055 imu, int n_sensor, bool reset_calibration) {
  int eeAddress = n_sensor * 2 * sizeof(long);
  long eeIMU_ID;
  long imu_ID;
  bool foundCalib = false;

  if (reset_calibration) { // Then reset the EEPROM so a new calibration can be made

    EEPROM.put(eeAddress, 0);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, 0);
    eeAddress = n_sensor * 2 * sizeof(long);
    if (display_BNO055_info) {
      Serial.println("\nEEPROM reset.");
      delay(10000);
    }
  }


  EEPROM.get(eeAddress, eeIMU_ID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */
  imu.getSensor(&sensor);
  imu_ID = sensor.sensor_id;

  if (eeIMU_ID != imu_ID) {

    if (display_BNO055_info) {

      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(2000);
    }
  }
  else {

    if (display_BNO055_info) {

      Serial.println("\nFound Calibration for this sensor in EEPROM.");
    }

    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    if (display_BNO055_info) {

      displaySensorOffsets(calibrationData);
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
    }

    imu.setSensorOffsets(calibrationData);

    if (display_BNO055_info) {

      Serial.println("\n\nCalibration data loaded into BNO055");
      delay(2000);
    }

    foundCalib = true;
  }

  if (display_BNO055_info) {

    /* Display some basic information on this sensor */
    displaySensorDetails(imu);

    /* Optional: Display current status */
    displaySensorStatus(imu);

  }

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  imu.setExtCrystalUse(true);


  if (foundCalib) {

    performMagCal(imu); /* always recalibrate the magnetometers as it goes out of calibration very often */
  }
  else {

    if (display_BNO055_info) {

      Serial.println("Please Calibrate Sensor: ");
      delay(2000);
    }

    while (!imu.isFullyCalibrated()) {

      if (display_BNO055_info) {

        displayCalStatus(imu);
        Serial.println("");
        delay(BNO055_SAMPLERATE_DELAY_MS); // Wait for the specified delay before requesting new data
      }
    }

    adafruit_bno055_offsets_t newCalib;
    imu.getSensorOffsets(newCalib);

    if (display_BNO055_info) {

      Serial.println("\nFully calibrated!");
      delay(3000);
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");

      displaySensorOffsets(newCalib);

      Serial.println("\n\nStoring calibration data to EEPROM...");
    }


    eeAddress = n_sensor * 2 * sizeof(long);
    EEPROM.put(eeAddress, imu_ID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);


    if (display_BNO055_info) {
      Serial.println("Data stored to EEPROM.");
      Serial.println("\n--------------------------------\n");
      delay(3000);
    }
  }
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
    if (display_BNO055_info) {

      displayCalStatus(imu);
      Serial.println("");
    }
  }

  if (display_BNO055_info) {

    Serial.println("\nMagnetometer calibrated!");
  }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------*/
