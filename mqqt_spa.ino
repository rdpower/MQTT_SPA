#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "time.h"

/******************HOME ASSISTANT CONFIG***********************************
  JSON format:

  Command Topic:  spa/set
    setTemp = (float) 0-40
    mode = (int) 0(off),1(on),2(auto)
    onTime = (float)
    spaTime = (float)
    offTime = (float)
    setRunT = (int)


  State Topic: spa/status
    t = (float)
    st = (float) 0-40
    m = (int) 0(off),1(on),2(auto)
    p = (int)
    rt = (int)
    onTime = (float)
    spaTime = (float)
    offTime = (float)
    setRunT = (int)

*/

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* device_name = "spa";
const char* ota_password = "";
const char* command_topic = "spa/set";
const char* state_topic = "spa/status";

#define ZEROCROSS 17
#define FLOW 35
#define GREEN 25
#define BLUE 26
#define RED 32
#define HEATER 4
#define PUMP 2
#define HEATERTEMP 0
#define POOLTEMP 1
#define PH 2

#define HEATERCHANNEL 0
#define TIMECONSTANT 2
#define KP 60
#define KI 0.5
#define KD 0
#define WINDUP 20/KI
#define ADCSAMPLES 50
#define HEATERPOWER 3000
#define PUMPPOWER 1200
#define MAXTEMP 41
#define OFF 0
#define HEAT 1
#define AUTO 2
#define STARTTIME 9 //9am
#define DAYRUN 600 //10 minutes a day
#define SPATIME 19.5 //7:30pm
#define OFFTIME 21 //9am
#define HEATRATE 2.5 //Degrees per hour
#define ADC_ADDRESS 0x48

hw_timer_t * heaterTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

struct tm timeinfo;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 12 * 3600; //NZT +12H
//const int   daylightOffset_sec = 3600; //Daylight savings
const int   daylightOffset_sec = 0; //Daylight savings
long runTime = 0, runTimeLimit = DAYRUN, setRunTime = DAYRUN;
const int BUFFER_SIZE = JSON_OBJECT_SIZE(16);
volatile char effortLimit = 99, targetEffort = 0, effortActual = 0;
volatile float temperature, setTemperature = 38, error = 0, lastError = 0, effort = 0, integral = 0, derivative = 0, phLevel = 0, heaterTemperature = 0, timeNow, onTime, spaTime, offTime, timeTillSpa;
long lastFlash = 0, noFlow = 0;
bool red, green, blue;
volatile bool pumpState = false, newTimeSettings = true;
volatile int cross;
int state = 0, flash = 0;

WiFiClient espClient;
PubSubClient client(espClient);
Ticker SpaController;
Adafruit_ADS1115 ads;

/*GetPoolTemp()
 *  Oversamples then averages pool temperature and heater temperature
 */
void GetPoolTemp() {
  float tmp = 0;
  float tmp2 = 0;
  for (int i = 0; i < ADCSAMPLES; i++) {
    tmp += ads.readADC_SingleEnded(POOLTEMP);
    tmp2 += ads.readADC_SingleEnded(HEATERTEMP);
  }
  tmp /=  ADCSAMPLES;
  tmp2 /=  ADCSAMPLES;
  temperature = 616000 / tmp;
  heaterTemperature = 616000 / tmp2;
  if (temperature > MAXTEMP || heaterTemperature > MAXTEMP) {
    state = OFF;
  }
}

/*GetPH()
 *  Oversamples then averages pool PH level
 */
void GetPH() {
  float tmp = 0;
  for (int i = 0; i < ADCSAMPLES; i++) {
    tmp += ads.readADC_SingleEnded(PH);
  }
  tmp /= ADCSAMPLES;
  phLevel =  tmp;
}

/*Flasher()
 *  Updates RGB indicator states
 */
static void Flasher() {
  if (state == OFF || (state == AUTO && !pumpState)) {
    flash = 1000;
    red = false;
    green = false;
    blue = true;
  } else if (state == HEAT || state == AUTO) {
    if ((error < 1) && (error > -1)) {//Within 1 Degree Celsius flash green
      red = false;
      green = true;
      blue = false;
    } else {//Otherwise flash red
      red = true;
      green = false;
      blue = false;
    }
    flash = (int)(1000 / abs(error));
  }
}

/* CalcPID()
    Calculates the proportional, integral, and derivative error of the spa temperature
*/
void CalcPID() {
  GetPoolTemp();
  if (pumpState) {//Use the circulating water temperature
    error = setTemperature - heaterTemperature;
  } else {//Use the non circulating water temperature
    error = setTemperature - temperature;
  }
  if (state) {
    derivative = lastError - error;
    lastError = error;
    integral += error;
    if (integral > WINDUP)integral = WINDUP;//Limit windup
    else if (integral < -WINDUP)integral = -WINDUP;
    effort = (error * KP + integral * KI + derivative * KD);
    if (effort > 99)effort = 99;
    else if (effort < 0)effort = 0;
  } else {
    integral = 0;
    derivative = 0;
    lastError = 0;
  }
}

/* SpaController()
    Calculates PID
    Manages the spa state and time
    Updates MQTT server with status
*/
void SpaController() {
  CalcPID();
  if (state == HEAT) {
    runTime++;
    pumpState = true;
    targetEffort = (int)effort;
  }  else if (state == AUTO) {
    getLocalTime(&timeinfo);
    timeNow = timeinfo.tm_hour + ((float)(timeinfo.tm_min) / 60);
    timeTillSpa  = spaTime - timeNow;
    if (timeNow >= onTime && timeNow < offTime) { //Daytime running
      if (((runTime * TIMECONSTANT) < runTimeLimit)) {//Auto: Run for filter time
        runTime++;
        pumpState = true;
        if (runTime * TIMECONSTANT) > setRunTime)) {//Dont run the heater during filter time
          targetEffort = (int)effort;
        }
      } else if ((error / HEATRATE) > timeTillSpa &&  timeTillSpa > 0) {//Auto: Make sure it's hot for spa time
        runTimeLimit += 600;//Run for 10 minutes
      } else if (error > 5) {//Auto: Loosely maintain temperature during daytime
        runTimeLimit += 600;//Run for 10 minutes
      } else { //Auto: Idle
        pumpState = false;
        targetEffort = 0;
        integral = 0;
      }
    } else if (timeinfo.tm_hour < 1 && runTime) { //Auto: Reset runtime at new day
      runTime = 0;
      runTimeLimit = setRunTime;
    } else {//Auto: Idle
      pumpState = false;
      targetEffort = 0;
      integral = 0;
    }
  } else {//State is off
    if (pumpState || targetEffort) {
      pumpState = false;
      targetEffort = 0;
    }
  }
  Flasher();
  SendState();
}

/*  ZeroCrossHandler()
      Turns Pump OFF at AC zerocross and set time till pump ON.
      Turns heater OFF at AC zerocross and manages slew and pulse skipping
*/
void ZeroCrossHandler() {
  digitalWrite(PUMP, LOW);
  digitalWrite(HEATER, LOW);
  if (++cross > 99) {
    cross = 1;
  }
  //PUMP
  digitalWrite(PUMP, pumpState);
  //HEATER
  if (!(cross % 2)) {//Ever other zerocrossing ramp heater power
    if (targetEffort > effortActual) {
      effortActual++;
    } else if (targetEffort < effortActual) {
      effortActual--;
    }
  }
  if (effortActual > cross) {
    if (FLOW) {//If there is flow allow the heater
      digitalWrite(HEATER, HIGH);
    } else {
      if (++noFlow > 2000) {//No flow error
        noFlow = 0;
        state = OFF;
      }
    }
  }
}

/* SetupWifi()
 * Connects to the local WiFi
 */
void SetupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/* MqttConnect()
 * Connects to the MQTT Server
 */
void MqttConnect() {
  do {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(device_name, mqqt_username, mqqt_password)) {
      if (client.subscribe(command_topic) ) {
        Serial.print("Connected to: ");
        Serial.println(command_topic);
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      delay(10000);
    }
  } while (!client.connected());
}

/* ProcessJson()
 * Processes the Char string and converts to JSON 
 * Updates local variables with JSON values
 */
bool  ProcessJson(char* message) { //mqqt message process
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(message);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
  if (root.containsKey("mode")) {
    state = root["mode"];
  }
  if (root.containsKey("setTemperature")) {
    if (root["setTemperature"] > MAXTEMP)setTemperature = MAXTEMP;
    else if (root["setTemperature"] < 0)setTemperature = 0;
    else setTemperature = root["setTemperature"];
  }
  if (root.containsKey("onTime")) {
    onTime = root["onTime"];
    newTimeSettings = true;
  }
  if (root.containsKey("spaTime")) {
    spaTime = root["spaTime"];
    newTimeSettings = true;
  }
  if (root.containsKey("offTime")) {
    offTime = root["offTime"];
    newTimeSettings = true;
  }
  if (root.containsKey("setRunTime")) {
    setRunTime = root["setRunTime"];
    newTimeSettings = true;
  }
  if (root.containsKey("powerAvaliable")) {
    powerAvaliable = root["powerAvaliable"];
  }
  return true;
}

/* SendState()
 * Converts local variables to JSON
 * Sends JSON to MQTT Server
 */
void SendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  if (newTimeSettings) {
    root["onTime"] = onTime;
    root["spaTime"] = spaTime;
    root["offTime"] = offTime;
    root["setRunT"] = setRunTime;
    newTimeSettings = false;
  } else {
    if (state == 1) {
      root["m"] = "heat";
    } else if (state == 2) {
      root["m"] = "auto";
    } else {
      root["m"] = "off";
    }
    root["st"] = setTemperature;
    root["t"] = temperature;
    if (targetEffort) {
      root["p"] = PUMPPOWER + ((HEATERPOWER * (targetEffort + 1)) / 100);
    } else if (pumpState) {
      root["p"] = PUMPPOWER;
    } else {
      root["p"] = 0;
    }
    root["rt"] = runTime * TIMECONSTANT;
    root["tts"] = timeTillSpa;
  }
  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));
  client.publish(state_topic, buffer, true);
}

/* Callback()
 * Copies MQTT message to a char string
 * Processes message
 * Acknowledges message
 */
void Callback(char* topic, byte * payload, unsigned int length) { //mqqt callback
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);
  ProcessJson(message);
  SendState();//ACK
}

/*UpdaterInit()
 *  Arduino OTA
 */
void UpdaterInit() {
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(device_name);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA
  .onStart([]() {
    detachInterrupt(digitalPinToInterrupt(ZEROCROSS)); //Safe the pool
    digitalWrite(PUMP, LOW);
    digitalWrite(HEATER, LOW);
    red = true;
    blue = true;
    green = false;
    flash = 0;
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  pinMode(POOLTEMP, INPUT);
  pinMode(ZEROCROSS, INPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(PUMP, LOW);
  digitalWrite(HEATER, LOW);
  SetupWifi();
  UpdaterInit();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  getLocalTime(&timeinfo);
  onTime = STARTTIME;
  offTime = OFFTIME;
  spaTime = SPATIME;
  ads.begin();
  ads.setGain(GAIN_ONE);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  MqttConnect();
  SpaController.attach(TIMECONSTANT, SpaController); //Start Spa Controller Task
  attachInterrupt(digitalPinToInterrupt(ZEROCROSS), ZeroCrossHandler, RISING); //Attach zerocrossing for AC control
}

void loop() {
  if ( WiFi.status() != WL_CONNECTED) {// Ensure wifi and mqtt is connected
    SetupWifi();
  }  else if (!client.connected()) {
    MqttConnect();
  }
  client.loop();
  ArduinoOTA.handle();

  if (flash) {
    if (millis() > lastFlash) {
      lastFlash = millis() + flash;
      if (red) {
        digitalWrite(RED, !digitalRead(RED));
      } else {
        digitalWrite(RED, HIGH);
      }
      if (green) {
        digitalWrite(GREEN, !digitalRead(GREEN));
      } else {
        digitalWrite(GREEN, HIGH);
      }
      if (blue) {
        digitalWrite(BLUE, !digitalRead(BLUE));
      } else {
        digitalWrite(BLUE, HIGH);
      }
    }
  } else {
    digitalWrite(RED, !red);
    digitalWrite(GREEN, !green);
    digitalWrite(BLUE, !blue);
  }
}
