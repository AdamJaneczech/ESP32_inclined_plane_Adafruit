#include "Adafruit_VL53L0X.h"
#include "Servo.h"
#include <PubSubClient.h>
#include <WiFi.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 26
#define SHT_LOX2 25
//set I2C pins on the ESP32
#define SDA_PIN 19
#define SCL_PIN 15
//maximum distance
#define MAX_DIST 300
//servo angles
#define OPEN_ANGLE 210
#define CLOSED_ANGLE 120

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
//define a servo
Servo servo;

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

uint16_t initialDistance;
uint64_t initialTime;
uint16_t distance;

uint32_t id;
uint8_t angle;

WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "GKREN_STUDENT";
const char* password = "g.stud.123";

const char* mqtt_server = "your_MQTT_broker_address";;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  lox1.setMeasurementTimingBudgetMicroSeconds(20000); // Set timing budget to 20ms
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  lox2.setMeasurementTimingBudgetMicroSeconds(20000); // Set timing budget to 20ms

  Serial.print("LOX1: ");
  Serial.println(lox1.getMeasurementTimingBudgetMicroSeconds());
  Serial.print("LOX2: ");
  Serial.println(lox2.getMeasurementTimingBudgetMicroSeconds());
}

boolean read_dual_sensors() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  // print sensor one reading
  //Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    //Serial.print("range1");
  } else {
    Serial.print(F("Out of range"));
    return false;
  }
  
  Serial.print(F(", "));

  // print sensor two reading
  //Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
    return false;
  }
  
  Serial.print(F(","));
  Serial.println(millis() - initialTime);

  return true;
}

void setInitialDistance(){
  initialDistance = measure1.RangeMilliMeter;
  initialTime = millis();
  read_dual_sensors();
}

void servoState(bool state){
  if(state){
    servo.write(OPEN_ANGLE);
  }
  else{
    servo.write(CLOSED_ANGLE);
  }
}

void setWiFi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  client.setServer(mqtt_server, 1883);
}

void setup() {
  Serial.begin(115200);

  servo.attach(5);
  servo.write(90);
  servoState(false);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  Serial.println(F("Starting..."));
  setID();
  setWiFi();
}

void loop() {
  //request for a measurement
  if(Serial.read() == 's'){ 
    Serial.println("Measurement start");
    Serial.println("-----------------");
    setInitialDistance();
    servoState(true);
    while(uint16_t(measure2.RangeMilliMeter) >= 30){
      read_dual_sensors();
    }
  }
}