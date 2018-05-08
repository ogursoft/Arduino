#include <PersWiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TimeLib.h>
#include <SPIFFSReadServer.h> // http://ryandowning.net/SPIFFSReadServer/
#include <DNSServer.h>
#include <FS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266Ping.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADDRESS 0x76

#define DELAY_MEASURE 5000
//#define DEBUG_SERIAL //uncomment for Serial debugging statements
#ifdef DEBUG_SERIAL
#define DEBUG_BEGIN Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.println(x)
#define DEBUG_PRINTF(x,y) Serial.printf(x,y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x,y)
#define DEBUG_BEGIN
#endif

static const char ntpServerName[] = "pool.ntp.org";                          //ntp сервер
const int timeZone = 3;                                                      //timezone Europe/Moscow
String uptime( unsigned long currentmillis);
unsigned long lastMeasure;
unsigned long newTime;
float temp;
float humidity;
float pressure;
unsigned int localPort = 8888;  // local port to listen for UDP packets
const char* mqtt_server = "orangepi.lan";
String device_name;
String chipId;
String mqtt_topic = "home/" + device_name;
unsigned long lastReconnectAttempt = 0;

void sendNTPpacket(IPAddress &address);

time_t getNtpTime();
WiFiUDP Udp;
SPIFFSReadServer server(80);
DNSServer dnsServer;
PersWiFiManager persWM(server, dnsServer);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme; // I2C

//===========================================================================
void setup() {

  DEBUG_BEGIN;

  chipId = String(ESP.getChipId());
  device_name = "esp8266-bme280-" + chipId;

  //optional code handlers to run everytime wifi is connected...
  persWM.onConnect([]() {
    DEBUG_PRINT("wifi connected");
    DEBUG_PRINT(WiFi.localIP());
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);
    setSyncInterval(86400);
  });
  //...or AP mode is started
  persWM.onAp([]() {
    DEBUG_PRINT("AP MODE");
    DEBUG_PRINT(persWM.getApSsid());
  });

  //allows serving of files from SPIFFS
  SPIFFS.begin();
  //sets network name for AP mode
  // persWM.setApCredentials(device_name);
  persWM.setApCredentials(device_name, "password"); //optional password

  //make connecting/disconnecting non-blocking
  persWM.setConnectNonBlock(true);

  //in non-blocking mode, program will continue past this point without waiting
  persWM.begin();

  //OTA handlers setup ===================================
  ArduinoOTA.setHostname(device_name.c_str());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG_PRINT("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINT("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(BME280_ADDRESS);
  if (!status) {
    DEBUG_PRINT("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  DEBUG_PRINT("-- Default Test --");

  // Sends Data to web page ==========================================
  server.on("/api", []() {
    DEBUG_PRINT("server.on /api");
    //build json object of program data
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["temp"] = temp;
    json["humidity"] = humidity;
    json["pressure"] = pressure;
    json["uptime"] = uptime(newTime);
    json["currtime"] = String(day()) + "." + String(month()) + "." + String(year()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second());
    json["chipid"] = chipId;
    char jsonchar[200];
    json.printTo(jsonchar); //print to char array, takes more memory but sends in one piece
    server.send(200, "application/json", jsonchar);
  }); //server.on api

  server.begin();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  lastReconnectAttempt = 0;

  DEBUG_PRINT("setup complete.");
}

//=============================================================================

// Функция вывода uptime
String uptime( unsigned long currentmillis) {
  String currUptime;
  long days = 0;
  long hours = 0;
  long mins = 0;
  long secs = 0;
  secs = currentmillis / 1000; //convect milliseconds to seconds
  mins = secs / 60; //convert seconds to minutes
  hours = mins / 60; //convert minutes to hours
  days = hours / 24; //convert hours to days
  secs = secs - (mins * 60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24); //subtract the coverted hours to days in order to display 23 hours max
  //Display results
  currUptime = "";
  if (days > 0) // days will displayed only if value is greater than zero
  {
    currUptime = String(days) + " дней, ";
  }
  if (hours < 10) {
    currUptime += "0" + String(hours) + ":";
  } else
    currUptime += String(hours) + ":";

  if (mins < 10) {
    currUptime += "0" + String(mins) + ":";
  } else
    currUptime += String(mins) + ":";

  if (secs < 10) {
    currUptime += "0" + String(secs);
  } else
    currUptime += String(secs);

  return currUptime;
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  DEBUG_PRINT("Transmit NTP Request");

  if (!Ping.ping(ntpServerName)) {
    ntpServerIP = WiFi.gatewayIP();
  }
  else {
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
  }
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINT("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG_PRINT("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

// PubSubClient functions
boolean reconnect() {
  DEBUG_PRINT("Mqtt connecting....");
  if (client.connect(device_name.c_str(), "mqttuser", "terminat")) {
    // Once connected, publish an announcement...
    DEBUG_PRINT("Mqtt connected");
    client.publish(mqtt_topic.c_str(), "ON");
    // ... and resubscribe
    client.subscribe(mqtt_topic.c_str());
  }
  return client.connected();
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
  }
}

// ============================================================================================================================================
void loop() {

  ArduinoOTA.handle();

  persWM.handleWiFi();

  dnsServer.processNextRequest();
  server.handleClient();

  newTime =  millis();              // Получаем текущее время

  if (newTime - lastMeasure > DELAY_MEASURE) {
    temp = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 133.32239F;

    lastMeasure = millis();
  }

  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 10000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    client.loop();
  }

  yield();
} // END loop
