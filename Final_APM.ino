// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "mobitel"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// MQTT details                  
const char* broker = "157.245.156.24";  // Public IP address or domain name
const char* mqttUsername = "";  // MQTT username
const char* mqttPassword = "";  // MQTT password

const char* topic = "slt_apm23/id_0001/device_001/data";

unsigned long previousMillis = 0;
// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS //comment if not required 

#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>


#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

//VAWT pins

#define anemometerPin       33
#define voltageSensorPin    34
#define SENSOR_VIBRA        32
#define currentpin          13
#define temp                25
#define SENSOR_ROTOR_SPD    14
#define SENSOR_WIND_DIR     12

uint32_t lastReconnectAttempt = 0;

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);;

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
volatile int count = 0;
unsigned long lastMillis = 0;

float temperatureC = 0;
float windSpeed = 0;
float voltage=0;
float current=0;
float rpm= 0;
float power =0;
String vibrationSensorState;

long lastMsg = 0;

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");

  return mqtt.connected();
}

// Wind direction positions
const int numPositions = 16;
const String positionswind[numPositions] = {
  "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
};

// Thresholds for each position
const int thresholdswind[numPositions] = {
  643, 838, 1033, 1228, 1423, 1618, 1813,
  2008, 2203, 2398, 2593, 2788, 2983, 3178, 3373, 3568
};

// Function to find the wind direction position based on the sensor value
int findPositionwind(int value) {
  for (int i = 0; i < numPositions; i++) {
    if (value < thresholdswind[i]) {
      return i;
    }
  }
  return numPositions - 1; // If value is higher than the last threshold, return the last position
}

// Voltage Sensor Resistors
float R1 = 30000.0;
float R2 = 7500.0;
float R3 = 11650.0;
float ref_voltage = 3.3;

// Current Sensor Resistors
float R4 = 6800.0;
float R5 = 12000.0;
float threshold = 2.5;

// Variables for calculating Rotor speed
volatile unsigned int pulseCount1 = 0;
unsigned long ppreviousMillis = 0; //?
unsigned int rpmspeed = 0;

float calculateWindSpeed() {
  float timeInterval = 1.0;  // Adjust this if necessary
  float rotationCount = count / 2.0;  // Divide by 2 for each full rotation
  float windSpeed = (rotationCount / timeInterval) * 0.083;  // Adjust factor based on anemometer specs
  count = 0;  // Reset count for the next measurement
  return windSpeed;
}

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(temp);

DallasTemperature sensors(&oneWire);

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  
  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  
  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  
  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
   modem.restart();
   //modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
 // mqtt.setCallback(mqttCallback);

//initialize sensors
  pinMode(anemometerPin, INPUT_PULLUP);
  pinMode(voltageSensorPin, INPUT);
  pinMode(SENSOR_VIBRA, INPUT);
  pinMode(SENSOR_WIND_DIR, INPUT);
  pinMode(currentpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), countPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_ROTOR_SPD), countPulse1, RISING);

}

// Interrupt service routine to count pulses
void countPulse1() {
  pulseCount1++;
}
void countPulse() {
  count++;
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        SerialMon.println("=== MQTT CONNECTED :-) !!! ===");
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;
    
     String dtime = modem.getGSMDateTime(DATE_TIME);

// Length (with one extra character for the null terminator)
int str_ln = dtime.length() + 1; 

// Prepare the character array (the buffer) 
char tst[str_ln];

// Copy it over 
  dtime.toCharArray(tst,str_ln);
  char *ptr;

    ptr = strchr(tst, '+');
    if (ptr != NULL) {
    *ptr = '\0';
    }
    Serial.printf("%s\n", tst);
  
  //1. windvane
  int windDirectionSensorValue = analogRead(SENSOR_WIND_DIR);
  int windDirectionPositionIndex = findPositionwind(windDirectionSensorValue);
  String wind_dir = positionswind[windDirectionPositionIndex];
  
  //2. anemometer
  unsigned long currentMillis1 = millis();
  float windSpeed = calculateWindSpeed();
    if (currentMillis1 - lastMillis >= 1000) {

    }

  //3. voltage Sensor
  int adc_value = analogRead(voltageSensorPin);
  float adc_voltage = adc_value * (ref_voltage / 4096.0);
  float voltage = adc_voltage * (((R1*R2) + (R1*R3) + (R2*R3)) / (R2*R3));


  //4. Current Sensor
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++){ //Get 150 samples
  AcsValue = analogRead(currentpin);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
  }
  AvgAcs=Samples/89.0;//Taking Average of Samples
  current = (2.5 - (AvgAcs * (3.3 / 4095.0)) )/0.100; // final current value

  //5. IR Sensor
  float rpm = (rpmspeed / 2000.0) * 60.0;
  unsigned long currentMillis = millis();
  if (currentMillis - ppreviousMillis >= 1000) {   
    rpmspeed = pulseCount1;
    pulseCount1 = 0;
    ppreviousMillis = currentMillis;
  }

  //6. Vibration Sensor
  int vibrationSensorState = pulseIn (SENSOR_VIBRA, HIGH);

  //7. Temperature Sensor
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

  //8. Power
  float power = current * voltage;


  // Define the capacity of the JSON buffer
  const size_t capacity = 200;  // Adjust the size according to your data size

  // Create a JSON document
  StaticJsonDocument<capacity> doc;

// Add data to the JSON document
doc["SENSETIME"] = tst;
doc["TEMP"] = temperatureC;
doc["WINDSPD"] = windSpeed;
doc["VOLT"] = voltage;
doc["CURRNT"] = current;
doc["ROTSPD"] = rpm;
doc["POWR"] = power;
doc["VIBRS"] = vibrationSensorState;
doc["WINDDIR"] = wind_dir;

// Serialize the JSON document to a char array
char jsonBuffer[capacity];
serializeJson(doc, jsonBuffer);

// Print the JSON data
Serial.println(jsonBuffer);

// To send the JSON data via MQTT
mqtt.publish(topic, jsonBuffer);


  }

  mqtt.loop();
}
