

/**************************************************************************
 
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels

#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   // < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 >
#define SDA_PIN 18
#define SCL_PIN 19

/*
  A337F33D-96B4-4FDD-86D1-1237AF8C59A9
  C4F3FDAF-EAB5-42DE-8F17-2C84503F4AF0
  A8425BD8-0271-47BB-BF52-63F950AC9E3F
  4FAF0DFA-CDCA-4826-8091-88F53ACC1763
  7B8F7EBC-DA64-4105-9EBA-04526091DAD6
  FC98DA0E-EEDB-4814-9215-0C05E3FB5389
*/

const char *serviceUUID = "A337F33D-96B4-4FDD-86D1-1237AF8C59A9";
const char *telemetryPitchUUID = "C4F3FDAF-EAB5-42DE-8F17-2C84503F4AF0";
const char *telemetryRollUUID = "A8425BD8-0271-47BB-BF52-63F950AC9E3F";
const char *telemetryYawUUID = "4FAF0DFA-CDCA-4826-8091-88F53ACC1763";
const char *telemetryRssiUUID = "7B8F7EBC-DA64-4105-9EBA-04526091DAD6";
const char *telemetryResetUUID = "FC98DA0E-EEDB-4814-9215-0C05E3FB5389";
const String companyName = "____Witness Equipment";                     // Bluetooth LE 1.4 Manufacturer Specific Data
const char *unitName = "Axis";
const char *productName = "Inclinometer";
const char *broadcastName = "Axis Inclinometer";
const char *gadgetName = "Axis Inclinometer";

const float radiansXPositiveThreshold = 0.01;
const float radiansXNegativeThreshold = 0.01;
const float radiansYPositiveThreshold = 0.01;
const float radiansYNegativeThreshold = 0.01;
const float radiansZPositiveThreshold = 0.00;
const float radiansZNegativeThreshold = 0.00;

const int warningPositivePitchThreshold = 65;
const int warningNegativePitchThreshold = -65;
const int warningPositiveRollThreshold = 45;
const int warningNegativeRollThreshold = -45;
const int warningPositiveYawThreshold = 0;
const int warningNegativeYawThreshold = 0;

const boolean reversePitch = false;
const boolean reverseRoll = true;
const boolean reverseYaw = false;

const int pitchPin = 2;
const int rollPin = 3;
const int yawPin = 4;
const int resetPin = 5;
const int distressPin = 14;

const int longDelay = 100;
const int shortDelay = 50;
const int notifyDelayMultiplier = 3;
const int calibrationDelay = 1500;

const boolean logIMUTelemetry = false;
const boolean logBLETelemetry = false;

int loopCounter = 0;
int notifyDelayCounter = 0;
int currentPitchAngle = 0;
int currentRollAngle = 0;
int currentYawAngle = 0;
int currentRssiLevel = 0;
int maximumPositivePitchAngle = 0;
int maximumNegativePitchAngle = 0;
int maximumPositiveRollAngle = 0;
int maximumNegativeRollAngle = 0;
int maximumPositiveYawAngle = 0;
int maximumNegativeYawAngle = 0;
int resetState = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BLEService axisInclinometerService(serviceUUID);
BLEIntCharacteristic telemetryPitchCharacteristic(telemetryPitchUUID, BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic telemetryRollCharacteristic(telemetryRollUUID, BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic telemetryYawCharacteristic(telemetryYawUUID, BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic telemetryRssiCharacteristic(telemetryRssiUUID, BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic telemetryResetCharacteristic(telemetryResetUUID, BLERead | BLEWrite | BLENotify);


// *************** SETUP ROUTINE ***************

void setup() {

  while (!Serial);  
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Failed to initialize SSD1306!"));
    while (1);
  }

  if(!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Initial display buffer
  display.clearDisplay();

  // Prepare pins
  pinMode(pitchPin, OUTPUT);
  pinMode(rollPin, OUTPUT);
  pinMode(yawPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(distressPin, OUTPUT);  

  // Serial/SD info
  if(logIMUTelemetry) {
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");
  }

  // Draw and wait for calibration
  drawCalibrationInterface();

  // Initialize Bluetooth LE
  byte companyNameBytes[companyName.length() + 1];
  companyName.getBytes(companyNameBytes, companyName.length() + 1);
  BLE.setLocalName(broadcastName);
  BLE.setDeviceName(gadgetName);
  BLE.setManufacturerData(companyNameBytes, companyName.length() + 1);
  BLE.setAdvertisedService(axisInclinometerService);
  axisInclinometerService.addCharacteristic(telemetryPitchCharacteristic);
  axisInclinometerService.addCharacteristic(telemetryRollCharacteristic);
  axisInclinometerService.addCharacteristic(telemetryYawCharacteristic);
  axisInclinometerService.addCharacteristic(telemetryRssiCharacteristic);
  axisInclinometerService.addCharacteristic(telemetryResetCharacteristic);
  BLE.addService(axisInclinometerService);
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  telemetryPitchCharacteristic.setEventHandler(BLERead, telemetryPitchCharacteristicReadHandler);
  telemetryPitchCharacteristic.setEventHandler(BLEWritten, telemetryPitchCharacteristicWriteHandler);
  telemetryPitchCharacteristic.setEventHandler(BLESubscribed, telemetryPitchCharacteristicSubscribedNotifyHandler);
  telemetryPitchCharacteristic.setEventHandler(BLEUnsubscribed, telemetryPitchCharacteristicUnsubscribedNotifyHandler);
  telemetryRollCharacteristic.setEventHandler(BLERead, telemetryRollCharacteristicReadHandler);
  telemetryRollCharacteristic.setEventHandler(BLEWritten, telemetryRollCharacteristicWriteHandler);
  telemetryRollCharacteristic.setEventHandler(BLESubscribed, telemetryRollCharacteristicSubscribedNotifyHandler);
  telemetryRollCharacteristic.setEventHandler(BLEUnsubscribed, telemetryRollCharacteristicUnsubscribedNotifyHandler);
  telemetryYawCharacteristic.setEventHandler(BLERead, telemetryYawCharacteristicReadHandler);
  telemetryYawCharacteristic.setEventHandler(BLEWritten, telemetryYawCharacteristicWriteHandler);
  telemetryYawCharacteristic.setEventHandler(BLESubscribed, telemetryYawCharacteristicSubscribedNotifyHandler);
  telemetryYawCharacteristic.setEventHandler(BLEUnsubscribed, telemetryYawCharacteristicUnsubscribedNotifyHandler);
  telemetryRssiCharacteristic.setEventHandler(BLERead, telemetryRssiCharacteristicReadHandler);
  telemetryRssiCharacteristic.setEventHandler(BLEWritten, telemetryRssiCharacteristicWriteHandler);
  telemetryRssiCharacteristic.setEventHandler(BLESubscribed, telemetryRssiCharacteristicSubscribedNotifyHandler);
  telemetryRssiCharacteristic.setEventHandler(BLEUnsubscribed, telemetryRssiCharacteristicUnsubscribedNotifyHandler);
  telemetryResetCharacteristic.setEventHandler(BLERead, telemetryResetCharacteristicReadHandler);
  telemetryResetCharacteristic.setEventHandler(BLEWritten, telemetryResetCharacteristicWriteHandler);
  telemetryResetCharacteristic.setEventHandler(BLESubscribed, telemetryResetCharacteristicSubscribedNotifyHandler);
  telemetryResetCharacteristic.setEventHandler(BLEUnsubscribed, telemetryResetCharacteristicUnsubscribedNotifyHandler);
  BLE.advertise();

}

// *************** LOOP ROUTINE ***************

void loop() {

  loopCounter++;

  resetState = digitalRead(resetPin);         // Respond to reset button press
  if(resetState == LOW) {
    resetMeasurements();
    if(loopCounter > 3) {
      telemetryResetCharacteristic.writeValue(1);
      loopCounter = 0;
    }
  }

  BLE.poll();                                 // Periodic BLE poll

  int warningStateStatus = 0;

  queryIMU();                                 // Query Pitch and Roll data from the sensors
  queryBLE();                                 // QWuery BLE metadata
  drawAxisInterface();                        // Draw the OLED interface
  warningStateStatus = deriveWarningState();  // Determine if we are in a warning state

  if(notifyDelayCounter >= notifyDelayMultiplier) {
    telemetryPitchCharacteristic.writeValue(currentPitchAngle);
    telemetryRollCharacteristic.writeValue(currentRollAngle);
    telemetryYawCharacteristic.writeValue(currentYawAngle);
    telemetryRssiCharacteristic.writeValue(currentRssiLevel);
    notifyDelayCounter = 0;
  } else {
    notifyDelayCounter++;
  }
  
  if(warningStateStatus == 0) {
    delay(longDelay);                         // Standard delay between redraws
  } else {
    delay(shortDelay);                        // Distress delay between redraws
  }
  
}

// *************** ADDITIONAL ROUTINE(S) ***************

void queryIMU(void) {

  float x, y, z;
  int degreesX = 0;
  int distressX = 0;
  int degreesY = 0;
  int distressY = 0;
  int degreesZ = 0;
  int distressZ = 0;

  // Read IMU
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  // Convert and map
  x = 100 * x;
  degreesX = map(x, 0, 97, 0, 90);
  if(isCurrentPitchAngleDistressed()) {
    distressX = 1;
  }
  y = 100 * y;
  degreesY = map(y, 0, 97, 0, 90);
  if(isCurrentRollAngleDistressed()) {
    distressY = 1;
  }
  z = 100 * z;
  degreesZ = map(z, 0, 97, 0, 90);

  if(logIMUTelemetry) {
    Serial.print("X-Pitch: ");
    Serial.print(degreesX);
    Serial.print(", ");
    Serial.print("X-Rads: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print("X-Distress: ");
    Serial.print(distressX);
    Serial.print(", ");
    Serial.print("Y-Roll: ");
    Serial.print(degreesY);
    Serial.print(", ");
    Serial.print("Y-Rads: ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print("Y-Distress: ");
    Serial.print(distressY);
    Serial.print(", ");
    Serial.print("Z-Yaw: ");
    Serial.print(degreesZ);
    Serial.print(", ");
    Serial.print("Z-Rads: ");
    Serial.print(z);
    Serial.print(", ");
    Serial.print("Z-Distress: ");
    Serial.print(distressZ);
    Serial.print(", ");
    Serial.println();
  }

  // Reverse pitch/roll values to accomodate the product enclosure configuration
  currentPitchAngle = (reversePitch) ? -degreesY : degreesY;
  currentRollAngle = (reverseRoll) ? -degreesX : degreesX;
  currentYawAngle = (reverseYaw) ? -degreesZ : degreesZ;
  
}

void queryBLE(void) {

  currentRssiLevel = BLE.rssi();

  if(logBLETelemetry) {
    Serial.print("Rssi: ");
    Serial.print(currentRssiLevel);
    Serial.println();
  }
  
}

void drawAxisInterface(void) {
  
  display.clearDisplay();

  display.cp437(true);

  // Header
  drawHeader();

  // Container Line(s)
  display.drawLine(display.width()/2, 20, display.width()/2, 31, SSD1306_WHITE);

  // Pitch
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(18,20);
  if(currentPitchAngle >= warningPositivePitchThreshold || currentPitchAngle <= warningNegativePitchThreshold) {
    display.println(F("Pitch*"));
  } else {
    display.println(F("Pitch"));
  }
  display.setTextSize(2);
  if(currentPitchAngle < -9) {
    display.setCursor(-5,35);
  } else {
    display.setCursor(3,35);
  }
  if(currentPitchAngle > maximumPositivePitchAngle) {
    maximumPositivePitchAngle = currentPitchAngle;
  }
  if(currentPitchAngle < maximumNegativePitchAngle) {
    maximumNegativePitchAngle = currentPitchAngle;
  }
  display.println(String(" ") + currentPitchAngle + String(" "));
  display.drawCircle((display.width()/2)-17, 38, 2, SSD1306_WHITE);

  // Roll
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(81,20);
  if(currentRollAngle >= warningPositiveRollThreshold || currentRollAngle <= warningNegativeRollThreshold) {
    display.println(F("Roll*"));
  } else {
    display.println(F("Roll"));
  }
  display.setTextSize(2);
  if(currentRollAngle < -9) {
    display.setCursor(58,35);
  } else {
    display.setCursor(67,35);
  }
  if(currentRollAngle > maximumPositiveRollAngle) {
    maximumPositiveRollAngle = currentRollAngle;
  }
  if(currentRollAngle < maximumNegativeRollAngle) {
    maximumNegativeRollAngle = currentRollAngle;
  }
  display.println(String(" ") + currentRollAngle + String(" "));
  display.drawCircle(display.width()-17, 38, 2, SSD1306_WHITE);

  // Footer
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // Pitch
  if(maximumPositivePitchAngle < 9 && maximumNegativePitchAngle > -10) {
    display.setCursor(17,55);
  } else {
    if((maximumPositivePitchAngle > 9 && maximumNegativePitchAngle > -10) || (maximumPositivePitchAngle < 10 && maximumNegativePitchAngle < -9)) {
      display.setCursor(11,55);
    } else {
      display.setCursor(5,55);
    }
  }
  if(maximumPositivePitchAngle > 0) {
    display.println(String("+") + maximumPositivePitchAngle + String(" / ") + maximumNegativePitchAngle);
  } else {
    display.println(maximumPositivePitchAngle + String(" / ") + maximumNegativePitchAngle);
  }
  // Roll
  if(maximumPositiveRollAngle < 9 && maximumNegativeRollAngle > -10) {
    display.setCursor(81,55);
  } else {
    if((maximumPositiveRollAngle > 9 && maximumNegativeRollAngle > -10) || (maximumPositiveRollAngle < 10 && maximumNegativeRollAngle < -9)) {
      display.setCursor(75,55);
    } else {
      display.setCursor(69,55);
    }
  }
  if(maximumPositiveRollAngle > 0) {
    display.println(String("+") + maximumPositiveRollAngle + String(" / ") + maximumNegativeRollAngle);
  } else {
    display.println(maximumPositiveRollAngle + String(" / ") + maximumNegativeRollAngle);
  }
  
  display.display();

}

void drawCalibrationInterface(void) {
  
  display.clearDisplay();

  display.cp437(true);

  // Header
  drawHeader();

  // Initialization
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(21, 40);
  display.println(F("Calibrating..."));
  display.display();
  delay(calibrationDelay);

}

void drawHeader(void) {

  display.drawRect(0, 0, display.width()-1, 15, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(6,3);
  display.println(F(" Axis Inclinometer "));
  
}

int deriveWarningState(void) {

  int status = 0;

  // Resolve warning status
  if(isCurrentPitchAngleDistressed()) {
    digitalWrite(pitchPin, HIGH);
    status = 1;
  } else {
    digitalWrite(pitchPin, LOW);
  }
  if(isCurrentRollAngleDistressed()) {
    digitalWrite(rollPin, HIGH);
    status = 1;
  } else {
    digitalWrite(rollPin, LOW);
  }
  if(isCurrentYawAngleDistressed()) {
    digitalWrite(yawPin, HIGH);
    status = 1;
  } else {
    digitalWrite(yawPin, LOW);
  }

  return status;
  
}

boolean isCurrentPitchAngleDistressed(void) {

  boolean distressed;
  ((currentPitchAngle >= warningPositivePitchThreshold || currentPitchAngle <= warningNegativePitchThreshold) ? distressed = true : distressed = false);
  return distressed;
  
}

boolean isCurrentRollAngleDistressed(void) {

  boolean distressed;
  ((currentRollAngle >= warningPositiveRollThreshold || currentRollAngle <= warningNegativeRollThreshold) ? distressed = true : distressed = false);
  return distressed;
  
}

boolean isCurrentYawAngleDistressed(void) {

  return false;
  
}

void resetMeasurements(void) {

  currentPitchAngle = 0;
  currentRollAngle = 0;
  maximumPositivePitchAngle = 0;
  maximumNegativePitchAngle = 0;
  maximumPositiveRollAngle = 0;
  maximumNegativeRollAngle = 0;
  resetState = 0;
  
}

// *************** BLE ROUTINE(S) ***************

// Central

void blePeripheralConnectHandler(BLEDevice central) {
  
  // Central connect event handler
  if(logBLETelemetry) {
    Serial.print("Connect event, central: ");
    Serial.println(central.address());
  }
  
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  
  // Central disconnect event handler
  if(logBLETelemetry) {
    Serial.print("Disconnect event, central: ");
    Serial.println(central.address());
  }
  
}

// Pitch Characteristic

void telemetryPitchCharacteristicReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to read Pitch! ");
    Serial.println(currentPitchAngle);
  }
  telemetryPitchCharacteristic.writeValue(currentPitchAngle);
}

void telemetryPitchCharacteristicWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to write Pitch! ");
    Serial.print(currentPitchAngle);
    Serial.print(" --> ");
    Serial.println(telemetryPitchCharacteristic.value());
  }
  currentPitchAngle = telemetryPitchCharacteristic.value();
}

void telemetryPitchCharacteristicSubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is subscribing for Pitch notify!");
  }
}

void telemetryPitchCharacteristicUnsubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is unsubscribing for Pitch notify!");
  }
}

// Roll Characteristic

void telemetryRollCharacteristicReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to read Roll! ");
    Serial.println(currentRollAngle);
  }
  telemetryRollCharacteristic.writeValue(currentRollAngle);
}

void telemetryRollCharacteristicWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to write Roll! ");
    Serial.print(currentRollAngle);
    Serial.print(" --> ");
    Serial.println(telemetryRollCharacteristic.value());
  }
  currentRollAngle = telemetryRollCharacteristic.value();
}

void telemetryRollCharacteristicSubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is subscribing for Roll notify!");
  }
}

void telemetryRollCharacteristicUnsubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is unsubscribing for Roll notify!");
  }
}

// Yaw Characteristic

void telemetryYawCharacteristicReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to read Yaw! ");
    Serial.println(currentYawAngle);
  }
  telemetryYawCharacteristic.writeValue(currentYawAngle);
}

void telemetryYawCharacteristicWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to write Yaw! ");
    Serial.print(currentYawAngle);
    Serial.print(" --> ");
    Serial.println(telemetryYawCharacteristic.value());
  }
  currentYawAngle = telemetryYawCharacteristic.value();
}

void telemetryYawCharacteristicSubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is subscribing for Yaw notify!");
  }
}

void telemetryYawCharacteristicUnsubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is unsubscribing for Yaw notify!");
  }
}

// Rssi Characteristic

void telemetryRssiCharacteristicReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to read Rssi! ");
    Serial.println(currentRssiLevel);
  }
  telemetryRssiCharacteristic.writeValue(currentRssiLevel);
}

void telemetryRssiCharacteristicWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to write Rssi! ");
    Serial.print(currentRssiLevel);
    Serial.print(" --> ");
    Serial.println(telemetryRssiCharacteristic.value());
  }
  currentRssiLevel = telemetryRssiCharacteristic.value();
}

void telemetryRssiCharacteristicSubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is subscribing for Rssi notify!");
  }
}

void telemetryRssiCharacteristicUnsubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is unsubscribing for Rssi notify!");
  }
}

// Reset Characteristic

void telemetryResetCharacteristicReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to read Reset State! ");
    Serial.println(resetState);
  }
  telemetryResetCharacteristic.writeValue(resetState);
}

void telemetryResetCharacteristicWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.print("Peripheral is requesting to write Reset State! ");
    Serial.print(resetState);
    Serial.print(" --> ");
    Serial.println(telemetryResetCharacteristic.value());
  }
  resetState = telemetryResetCharacteristic.value();
  Serial.print("Reset event captured!... ");
  Serial.println(resetState);
  if(telemetryResetCharacteristic.value() == 1) {
    Serial.println("Resetting measurements!");
    resetMeasurements();
  }
}

void telemetryResetCharacteristicSubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is subscribing for Reset State notify!");
  }
}

void telemetryResetCharacteristicUnsubscribedNotifyHandler(BLEDevice central, BLECharacteristic characteristic) {
  if(logBLETelemetry) {
    Serial.println("Peripheral is unsubscribing for Reset State notify!");
  }
}

/*
 * Example of writing a string...
 * 
    String stringColor = "";
    stringColor += r;
    stringColor += ",";
    stringColor += g;
    stringColor += ",";
    stringColor += b;
    stringColor += ",";
    stringColor += a;
  
    if (stringColor != previousColor) { // If reading has changed

        byte bytes[stringColor.length() + 1];
        stringColor.getBytes(bytes, stringColor.length() + 1);

        Serial.print("r, g, b, a: ");
        Serial.println(stringColor);

        colorCharacteristic.writeValue(bytes, sizeof(bytes));
        previousColor = stringColor;
*/
