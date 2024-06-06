/*
  Wind speed monitor

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <math.h>

 // Bluetooth® Low Energy environmental sensing service
BLEService envSensing("181A");

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedShortCharacteristic windSpeedChar("2A72",  // standard 16-bit characteristic UUID for wind speed
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

#define analogPinForRV    1
#define analogPinForTMP   0
#define MPH_TO_MPS        0.44704

const float zeroWindAdjustment = 0.4;   // Adjust this to calibrate
int TMP_Therm_ADunits;
float RV_Wind_ADunits;
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
float WindSpeed_MPS;

uint16_t oldWindSpeed = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

void updateWindSpeed() {
  TMP_Therm_ADunits = analogRead(analogPinForTMP);
  RV_Wind_ADunits = analogRead(analogPinForRV);
  RV_Wind_Volts = (RV_Wind_ADunits * 0.0048828125);

  TempCtimes100 = 2300;//(0.005 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;

  zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

  zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

  zeroWind_volts = zeroWind_volts > 0 ? zeroWind_volts : 0;
    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
  WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);
  WindSpeed_MPS = WindSpeed_MPH * MPH_TO_MPS;
  Serial.print("Wind Speed is now: ");
  Serial.println(WindSpeed_MPS);
  if (!isnan(WindSpeed_MPS))
    windSpeedChar.writeValue((unsigned short)(WindSpeed_MPS*100.0));
}

void setup() {
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("WindSensor");
  BLE.setAdvertisedService(envSensing); // add the service UUID
  envSensing.addCharacteristic(windSpeedChar); // add the wind speed characteristic
  BLE.addService(envSensing); // Add the arduino sensor service
  windSpeedChar.writeValue(oldWindSpeed); // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateWindSpeed();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}


