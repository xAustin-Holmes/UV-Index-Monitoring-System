#include <ArduinoBLE.h>

#define LEDR D7
#define LEDB D3
#define LEDG D2
#define SENSOR A5
#define BATTERY A1  //**NEED TO FIND THE ACTUAL PIN NUMBER**//

static const char* UVsensorgreeting = "UV Sensor Successfully Connected";
static const char* Batterygreeting = "Battery Monitor Successfully Connected";

BLEService SeniorDesign008TEST("180C");

BLEByteCharacteristic UVLevelChar("3941ed20-7d21-4f30-a126-c816c5facc07", BLERead | BLENotify);   // standard 16-bit characteristic UUID remote clients will be able to get notifications when this characteristic changes
BLEByteCharacteristic BatteryLevelChar("463950f7-0678-43d7-ad65-c1f85d7f7636", BLERead | BLENotify); // standard 16-bit characteristic UUID remote clients will be able to get notifications when this characteristic changes

BLEStringCharacteristic UVgreetingCharacteristic("2A56", BLERead, 13);  // standard 16-bit characteristic UUID remote clients will be able to read
BLEStringCharacteristic BatterygreetingCharacteristic("2A19", BLERead, 13); // standard 16-bit characteristic UUID remote clients will be able to read


void setup() {
  pinMode(SENSOR, INPUT);
  pinMode(BATTERY, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  Serial.begin(9600);

  if (!BLE.begin()) {   //this means that the bluetooth has failed to connect
    Serial.println("starting ble has failed");
    while (1); //1 = success 0= error
    //the off board LED will blink if it fails to initialize
    digitalWrite (LEDR, LOW);
    digitalWrite (LEDG, HIGH);
    digitalWrite (LEDB, HIGH);
    delay(2000);
    digitalWrite (LEDR, HIGH);  //turn off the red LED
    delay(2000);
  }

  BLE.setLocalName("494TestProgram");
  BLE.setAdvertisedService(SeniorDesign008TEST);
  SeniorDesign008TEST.addCharacteristic(UVLevelChar);
  SeniorDesign008TEST.addCharacteristic(BatteryLevelChar);
  BLE.addService(SeniorDesign008TEST); //add service
  UVgreetingCharacteristic.setValue(UVsensorgreeting);      // Set greeting string
  BatterygreetingCharacteristic.setValue(Batterygreeting);  // Set greeting string
  BLE.advertise();//Start advertising 1=success 0=fail
  Serial.print("Mobile App device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");

  ///below will turn off the LED until a connection is made.
  digitalWrite (LEDR, HIGH);
  digitalWrite (LEDG, HIGH);
  digitalWrite (LEDB, HIGH);

  delay(2000);
}

void loop() {
  float UV_sensor_voltage;          //the measurement from the UV sensor
  float UV_index;                   //the converted value to the UV index
  int UV_index_SEND;                //a simplified value being sent to bluetooth
  static int UV_analog;             //the analog value of the UV sensor
  static float UV_voltage_average = 0;  //the average of the voltage values

  float battery_voltage;            //the voltage from the battery
  static float battery_voltage_average = 0;   // the average from the battery readings
  static int battery_reading;
  static float battery_life;
  int battery_life_SEND;
  int criticalBatteryLEVEL = 0;     //this will indicate if the battery level is too low.
  int blueToothFLAG = 1;  //this will indicate if the bluetooth is not connected, it will be 1 if not connected


  BLEDevice central = BLE.central();//listens for mobile connection

  if (central)
  {
    blueToothFLAG = 0;
    Serial.print("Connected to Mobile App MAC: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) { //keep looping while mobile app
      for (int i = 0; i < 5000; i++) {
        UV_analog = analogRead(SENSOR);
        UV_sensor_voltage = (UV_analog / 1023) * 3.3;
        UV_voltage_average += UV_sensor_voltage; //total sum before being divided

        battery_reading = analogRead(BATTERY);
        battery_voltage = (battery_reading / 1023) * 3;
        battery_voltage_average += battery_voltage; //total sum before being divided
      }

      UV_voltage_average = UV_voltage_average / 5000;
      if (UV_voltage_average <= (1 / 5000)) {
        UV_voltage_average = 0;
      }

      UV_voltage_average = UV_voltage_average * 5000;
      UV_index = (UV_voltage_average) * 11; //11 is due to the 11 stages of the UV Index, could use 10, it's close enough

      battery_voltage_average = battery_voltage_average / 5000;
      if (battery_voltage_average <= (1 / 5000)) {
        battery_voltage_average = 0;
        criticalBatteryLEVEL = 1;
      }

      battery_life = (battery_voltage_average / 3) * 100;
      if (battery_life <= 10) {
        criticalBatteryLEVEL = 1;   //the battery level is low, we need to get a charge on it soon.
      }
      else {
        criticalBatteryLEVEL = 0; //the battery level is fine
      }

      UV_Spectrum(UV_index, criticalBatteryLEVEL, blueToothFLAG);
      UV_index_SEND = (int)UV_index;
      battery_life_SEND = (int)battery_life;

      UVLevelChar.writeValue((byte)(UV_index_SEND));
      //UVLevelChar.writeValue(UV_index_SEND);
      //BatteryLevelChar.writeValue((byte)(battery_life_SEND));
      Serial.print("UV INDEX: ");
      Serial.println(UV_index_SEND);
      Serial.print("Battery Life: ");
      Serial.println(battery_life_SEND);
      delay(200);
    }
  }
  
  if (!central) {
    blueToothFLAG = 1;  // blue tooth is not connected
    for (int i = 0; i < 5000; i++) {
      UV_analog = analogRead(SENSOR);
      UV_sensor_voltage = (UV_analog / 1023.0) * 3.3;
      UV_voltage_average += UV_sensor_voltage; //total sum before being divided

      battery_reading = analogRead(BATTERY);
      battery_voltage = (battery_reading / 1023) * 3;
      battery_voltage_average += battery_voltage; //total sum before being divided
    }

    UV_voltage_average = UV_voltage_average / 5000;
    if (UV_voltage_average <= (1 / 5000)) {
      UV_voltage_average = 0;
    }

    battery_voltage_average = battery_voltage_average / 5000;
    if (battery_voltage_average <= (1 / 5000)) {
      battery_voltage_average = 0;
      criticalBatteryLEVEL = 1;
    }

    battery_life = (battery_voltage_average / 3.7) * 100;
    if (battery_life <= 10) {
      criticalBatteryLEVEL = 1;   //the battery level is low, we need to get a charge on it soon.
    }
    else {
      criticalBatteryLEVEL = 0; //the battery level is fine
    }

    UV_voltage_average = UV_voltage_average * 5000;
    UV_index = (UV_voltage_average) * 11; //11 is due to the 11 stages of the UV Index could use 10, it's close enough

    Serial.print("UV Voltage Average: ");
    Serial.println(UV_voltage_average);
    Serial.print("UV INDEX: ");
    Serial.println(UV_index);
    Serial.print("Battery Life: ");
    Serial.println(battery_life);

    UV_Spectrum(UV_index, criticalBatteryLEVEL, blueToothFLAG);

    delay(1000);
  }
}

int UV_Spectrum(int UV_index, int bat_FLAG, int blue_FLAG) {
  if (bat_FLAG = 1) { //battery is low
    Serial.println("WHITE");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    delay(3000);
    Serial.println("OFF");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }

  if (blue_FLAG = 1) { //the bluetooth is not connected
    Serial.println("BLUE");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    delay(3000);
    Serial.println("OFF");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }

  // green [0-2] risk is low
  if (UV_index < 3) {
    Serial.println("GREEN");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }

  // yellow [3-5] medium risk
  else if (UV_index >= 3 && UV_index < 6) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }

  // red [6-7] high risk
  else if (UV_index >= 6  && UV_index < 8) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }

  // magenta [8-10] very high risk
  else if (UV_index >= 8 && UV_index < 11) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    delay(2000);
  }

  // cyan [11+] extremely high risk
  else {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    delay(2000);
  }
}
