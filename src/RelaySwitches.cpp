/*
    To start mDNS monitor (OSX) exec:   dns-sd -B _arduino._tcp

    Parameter für kleine 8266 ESP-01 Boards
    Board:     Generic 8266
    FlashMode: QIO
    FlashSize: 1M (256KB SPIFFS)
    FlashFreq: 40MHz

    Relay Board: https://www.electrodragon.com/w/ESP_Relay_Board_Hardware#Quick_Start_Guide
*/

#include <Arduino.h>

#include <ArduinoOTA.h>
/*
#include <JeVe_EasyOTA.h>  // https://github.com/jeroenvermeulen/JeVe_EasyOTA/blob/master/JeVe_EasyOTA.h
*/
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#define CPU_TYPE "ESP8266"
#endif

#ifdef ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
#define CPU_TYPE "ESP32"
#endif

#include "PubSubClient.h"
#include "DHTesp.h" // V 1.0.12 !

#define noELECTRODRAGON_RELAY
#define noSONOFF_RELAY

#define VERSION "1.5"
#define MQTT_HELLO "ESPxx relay MQTT gateway, version " VERSION ", Thomas Welsch, ttww@gmx.de, " __TIME__ " / " __DATE__


#include "secrets.h"

#define WIFI_SSID     SECRET_WIFI_SSID
#define WIFI_PASSWORD SECRET_WIFI_PASSWORD
#define MQTT_SERVER   SECRET_MQTT_SERVER

//------------------------------------------------------------------------------------------------------------

#ifdef ELECTRODRAGON_RELAY
#define TYPE_ID "electrodragon"
#define TYPE_STRING "www.electrodragon.com relay board with optional DHT11/DHT22"

#define noINVERT_RELAY
#define INVERT_BUTTON
#define noINVERT_LED

#define RELAY_0_GPIO 12 //  GPIO 12
#define RELAY_1_GPIO 13 //  GPIO 13
#define DHT_GPIO 14     //  GPIO 14

#define BTN0_GPIO 0
#define BTN1_GPIO 2

static byte Relays[] = {RELAY_0_GPIO, RELAY_1_GPIO};
static byte Buttons[] = {BTN0_GPIO, BTN1_GPIO};
#endif // ELECTRODRAGON_RELAY

//------------------------------------------------------------------------------------------------------------

#ifdef SONOFF_RELAY
#define TYPE_ID "sonoff"
#define TYPE_STRING "Sonoff basic 10A relay board"

#define noINVERT_RELAY
#define INVERT_LED
#define INVERT_BUTTON

/*
GPIO0 Taster
GPIO12 Relay (high = on) + LED (blau); LED nur bei S20
GPIO13 LED grün (low = on)
GPIO14 Pin 5 der Stiftleiste – nur Sonoff Switch
*/

#define RELAY_0_GPIO 12 //  GPIO 12
#define BTN0_GPIO 0

static byte Relays[] = {RELAY_0_GPIO};
static byte Buttons[] = {BTN0_GPIO};

#define LED0 13
#endif // SONOFF_RELAY

//------------------------------------------------------------------------------------------------------------

#ifdef MANUEL_RELAY
#define TYPE_ID "homebuild"
#define TYPE_STRING "ESP with relay module"

#define LED0 02

static byte Relays[] = {RELAYS};
static byte Buttons[] = {BUTTONS};
#endif

//------------------------------------------------------------------------------------------------------------

#define HOSTNAME "ESP"

#define MQTT_CLIENT hostname
#define MQTT_TOPIC "switch"

#define MQTT_UPDATE_INTERVAL 30 * 60 * 1000L
#define DHT_MEASURE_INTERVAL 15 * 1000L

#define RELAYS_COUNT ((int)sizeof(Relays))
#define BUTTONS_COUNT ((int)sizeof(Buttons))

static String hostname;
static String mqttTopic;

//------------------------------------------------------------------------------------------------------------

static boolean btn[BUTTONS_COUNT];

#ifdef DHT_GPIO
static float temperature = -99;
static float humidity = -99;
#endif
// EasyOTA       OTA;

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

#ifdef DHT_GPIO
static DHTesp dht;
#endif

static void mqttPublish(String state, String msg);
#define strequal(s, ss) (strcmp((s), (ss)) == 0)

//------------------------------------------------------------------------------------------------------------

static int lastProcDL;

static void setupOTA()
{
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(hostname.c_str());

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
                     {
                       String type;
                       if (ArduinoOTA.getCommand() == U_FLASH)
                       {
                         type = "sketch";
                       }
                       else
                       { // U_FS
                         type = "filesystem";
                       }

                       lastProcDL = -1;
                       // NOTE: if updating FS this would be the place to unmount FS using FS.end()
                       Serial.println("Start updating " + type); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
                          int procDL = (progress / (total / 100));
                          if (lastProcDL != procDL)
                          {
                            lastProcDL = procDL;
                            Serial.printf("Progress: %u%%\r", procDL);
                          } });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                       {
                         Serial.println("Auth Failed");
                       }
                       else if (error == OTA_BEGIN_ERROR)
                       {
                         Serial.println("Begin Failed");
                       }
                       else if (error == OTA_CONNECT_ERROR)
                       {
                         Serial.println("Connect Failed");
                       }
                       else if (error == OTA_RECEIVE_ERROR)
                       {
                         Serial.println("Receive Failed");
                       }
                       else if (error == OTA_END_ERROR)
                       {
                         Serial.println("End Failed");
                       } });
  ArduinoOTA.begin();
}

//------------------------------------------------------------------------------------------------------------

#ifdef DHT_GPIO
static char floatBuf[10];

static char *ftos(float f)
{
  snprintf(floatBuf, sizeof(floatBuf), "%1.1f", f);
  return floatBuf;
}
#endif

//------------------------------------------------------------------------------------------------------------

static void switchLed(bool on)
{
#ifdef LED0
#ifdef INVERT_LED
  digitalWrite(LED0, !on);
#else
  digitalWrite(LED0, on);
#endif
#endif
}

//------------------------------------------------------------------------------------------------------------

static bool readRelay(int r)
{
#ifdef INVERT_RELAY
  return !digitalRead(Relays[r]);
#else
  return digitalRead(Relays[r]);
#endif
}

//------------------------------------------------------------------------------------------------------------

static void switchRelay(int r, bool on)
{
#ifdef INVERT_RELAY
  digitalWrite(Relays[r], !on);
#else
  digitalWrite(Relays[r], on);
#endif
}

//------------------------------------------------------------------------------------------------------------

static bool readButton(int b)
{
#ifdef INVERT_BUTTON
  return !digitalRead(Buttons[b]);
#else
  return digitalRead(Buttons[b]);
#endif
}

//------------------------------------------------------------------------------------------------------------

static void initGpio()
{
  if (RELAYS_COUNT > 0)
  {
    for (int i = 0; i < RELAYS_COUNT; i++)
    {
      pinMode(Relays[i], OUTPUT);
      switchRelay(i, LOW);
    }
  }
  if (BUTTONS_COUNT > 0)
  {
    for (int i = 0; i < BUTTONS_COUNT; i++)
    {
      pinMode(Buttons[i], INPUT_PULLUP);
      btn[i] = readButton(i);
    }
  }

#ifdef LED0
  pinMode(LED0, OUTPUT);
#endif
}

//------------------------------------------------------------------------------------------------------------

static void mqttPublish(String state, String msg)
{

  //    String topic = String(mqtt_topic) + "state/" + state;
  String topic = mqttTopic + "state/" + state;

  Serial.println("Publish --> " + topic + " message: " + msg);

  int topicLen = topic.length() + 1;
  char *topicS = (char *)malloc(topicLen);
  strncpy(topicS, topic.c_str(), topicLen);

  int msgLen = msg.length() + 1;
  char *msgS = (char *)malloc(msgLen);
  strncpy(msgS, msg.c_str(), msgLen);

  mqttClient.publish(topicS, msgS);

  free(topicS);
  free(msgS);
}

//------------------------------------------------------------------------------------------------------------

static void publishRelay(int r)
{
  mqttPublish("relay/" + String(r, 10), readRelay(r) ? "true" : "false");
}

//------------------------------------------------------------------------------------------------------------

static void publishRelays()
{
  if (RELAYS_COUNT > 0)
  {
    for (int i = 0; i < RELAYS_COUNT; i++)
    {
      publishRelay(i);
    }
  }
}

//------------------------------------------------------------------------------------------------------------

static void publishBtn(int b)
{
  mqttPublish("button/" + String(b, 10), btn[b] ? "true" : "false");
}

//------------------------------------------------------------------------------------------------------------

static void publishBtns()
{
  if (BUTTONS_COUNT > 0)
  {
    for (int i = 0; i < RELAYS_COUNT; i++)
    {
      publishBtn(i);
    }
  }
}

//------------------------------------------------------------------------------------------------------------

static void publishDHT(boolean forceMqtt)
{
#ifdef DHT_GPIO
  TempAndHumidity th = dht.getTempAndHumidity();
  if (dht.getStatus() == 0)
  {
    float smoothTemperature, smoothHumidity;
    if (temperature != -99)
    {
      smoothTemperature = (temperature * 2 + th.temperature) / 3;
      if (abs(smoothTemperature - th.temperature) > 1)
        smoothTemperature = th.temperature; // to much difference...
    }
    else
      smoothTemperature = th.temperature;
    smoothTemperature = (int)(smoothTemperature * 10 + 0.5f) / 10.0f;

    if (humidity != -99)
    {
      smoothHumidity = (humidity * 2 + th.humidity) / 3;
      if (abs(smoothHumidity - th.humidity) > 1)
        smoothHumidity = th.humidity; // to much difference...
    }
    else
      smoothHumidity = th.humidity;
    smoothHumidity = (int)(smoothHumidity * 10 + 0.5f) / 10.0f;

    if (forceMqtt || smoothTemperature != temperature)
    {
      temperature = smoothTemperature;
      mqttPublish("temperature", ftos(temperature));
    }
    if (forceMqtt || smoothHumidity != humidity)
    {
      humidity = smoothHumidity;
      mqttPublish("humidity", ftos(humidity));
    }
  }
#endif
}

//------------------------------------------------------------------------------------------------------------

static void publishRelayState()
{
  mqttPublish("rssi", String(WiFi.RSSI()));
  publishRelays();
  publishBtns();
  publishDHT(true);
}

//------------------------------------------------------------------------------------------------------------

static void publishInfoState()
{
  mqttPublish("online", MQTT_HELLO);
  mqttPublish("type", TYPE_ID);
  mqttPublish("info", TYPE_STRING);
  mqttPublish("mqtthost", MQTT_SERVER);
  mqttPublish("sid", WIFI_SSID);
  mqttPublish("hostname", hostname);
  mqttPublish("ip", WiFi.localIP().toString());
  mqttPublish("rssi", String(WiFi.RSSI()));

  char itoabuf[20];
  mqttPublish("board", ARDUINO_BOARD);
  mqttPublish("cpu", CPU_TYPE);
  mqttPublish("cpu-freq", ltoa(F_CPU, itoabuf, 10));
}

//------------------------------------------------------------------------------------------------------------

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{

  char message[20];
  char *d = message;
  unsigned int len = length;
  if (len >= sizeof(message))
    len = sizeof(message) - 1;
  while (len-- > 0)
    *d++ = (char)(*payload++);
  *d = '\0';
  /*
    Serial.print("Receive <-- ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  */
  boolean isOn = false;
  if (strequal(message, "on") || strequal(message, "true") || strequal(message, "1"))
    isOn = true;

  String topicString = String(topic);

  if (topicString.endsWith("/info"))
  {
    publishRelayState();
    return;
  }
  if (topicString.endsWith("/check"))
  {
    mqttPublish("check", "ok");
    return;
  }

  for (int i = 0; i < RELAYS_COUNT; i++)
  {
    if (topicString.endsWith("/set/relay/" + String(i, 10)))
    {
      switchRelay(i, isOn);
      publishRelay(i);
      return;
    }
    if (topicString.endsWith("/get/relay/" + String(i, 10)))
    {
      publishRelay(i);
      return;
    }
  }
}

//------------------------------------------------------------------------------------------------------------

static void buildHostname()
{
  // 5C:CF:7F:82:2C:1E
  char buf[100];
  strcpy(buf, WiFi.macAddress().substring(9).c_str()); // Skip first 3 bytes
  char *s = buf;
  char *d = s;
  while (*s)
  {
    if (*s != ':')
      *d++ = toupper(*s);
    s++;
  }
  *d = '\0';
  String macString = String(buf);
  hostname = HOSTNAME "-" + macString;

  Serial.printf("Hostname = |%s|", hostname.c_str());
  mqttTopic = "switch/" HOSTNAME "_" + macString + String("/"); // perficed with function
}

//------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  Serial.println("Setup GPIO");
  initGpio();
  Serial.println("Setup GPIO DONE");

  buildHostname();

  Serial.println();
  Serial.println("===============================================================================================");
  Serial.println("ESPxx Relay MQTT gateway              V" VERSION "     Thomas Welsch / ttww@gmx.de");
  Serial.println();
  Serial.println("Source         : " __FILE__);
  Serial.println("Date           : " __DATE__);
  Serial.println("MAC address is : " + String(WiFi.macAddress()));
  Serial.println("Hostname       : " + String(hostname));
  Serial.println("MQTT brocker   : " + String(MQTT_SERVER));
  Serial.println("SSID           : " WIFI_SSID);
  Serial.println("Licence        : Use without limitations...");
#ifdef LED0
  Serial.println("LED at pin     : " + String(LED0, 10));
#endif
  Serial.println("Relay count    : " + String(RELAYS_COUNT, 10));
  for (int i = 0; i < RELAYS_COUNT; i++)
  {
    Serial.printf("        Relay  : %d at pin %d\n", i, Relays[i]);
  }
  Serial.println("Button count   : " + String(BUTTONS_COUNT, 10));
  for (int i = 0; i < BUTTONS_COUNT; i++)
  {
    Serial.printf("        Button : %d at pin %d\n", i, Buttons[i]);
  }

#ifdef DHT_GPIO
  dht.setup(DHT_GPIO, DHTesp::DHT11);
  dht.getTempAndHumidity(); // Access for getting status
  Serial.println("DHT Type       : DHT11");
  Serial.println("DHT State      : " + String(dht.getStatusString()));
#endif
  Serial.println("===============================================================================================");
  Serial.println("MQTT commands  : " + mqttTopic + "set/relay/0        message [true/false]");
  if (RELAYS_COUNT > 0)
  {
    for (int i = 1; i < RELAYS_COUNT; i++)
      Serial.printf("                 %sset/relay/%d        message [true/false]\n", mqttTopic.c_str(), i);
  }
  Serial.println("MQTT messages  : " + mqttTopic + "online             message [version...]");
  Serial.println("                 " + mqttTopic + "type               message [" TYPE_ID "]");
  Serial.println("                 " + mqttTopic + "info               message [" TYPE_STRING "]");

  for (int i = 0; i < RELAYS_COUNT; i++)
  {
    Serial.printf("                 %sstate/relay/%d      message [true/false]\n", mqttTopic.c_str(), i);
  }
  for (int i = 0; i < BUTTONS_COUNT; i++)
  {
    Serial.printf("                 %sstate/button/%d     message [true/false]\n", mqttTopic.c_str(), i);
  }

#ifdef DHT_GPIO
  Serial.println("                 " + mqttTopic + "state/temperature  message in °C [25.8], with DHT11");
  Serial.println("                 " + mqttTopic + "state/humidity     message in  % [53.3], with DHT11");
#endif
  Serial.println("===============================================================================================");

  /*
    // This callback will be called when JeVe_EasyOTA has anything to tell you.
    OTA.onMessage([](char *message, int line) {
        Serial.println(message);
    });

    OTA.setup(WIFI_SSID, WIFI_PASSWORD, hostname);
*/

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("IP address is  : " + WiFi.localIP().toString());

  setupOTA();

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);
}

//------------------------------------------------------------------------------------------------------------

static unsigned long lastPub = 0;
#ifdef DHT_GPIO
static unsigned long lastDht = 0;
#endif

void loop()
{

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    ESP.restart();
  }

  // OTA.loop();
  ArduinoOTA.handle();

  mqttClient.loop();

  if (!mqttClient.connected())
  {
    switchLed(true);

    Serial.println("Attempting MQTT broker: " MQTT_SERVER);
    if (mqttClient.connect(hostname.c_str()))
    {
      Serial.println("MQTT established, client " + String(hostname));

      //String subscribe = mqttTopic + "set/#";
      //Serial.println("Subscripe to " + subscribe);

      mqttClient.subscribe((mqttTopic + "set/#").c_str());
      mqttClient.subscribe((mqttTopic + "get/#").c_str());
      mqttClient.subscribe((mqttTopic + "check").c_str());
      mqttClient.subscribe((mqttTopic + "info").c_str());
//      mqttClient.subscribe(subscribe.c_str());

      publishInfoState();
      publishRelayState();
    }
    else
    {
      Serial.println("MQTT connection to " MQTT_SERVER " failed.");
      for (byte i = 0; i < 30; i++)
      {
        switchLed(false);
        delay(500);
        switchLed(true);
        delay(500);
      }
      return;
    }
  }
  else
  {

    if (millis() - lastPub > MQTT_UPDATE_INTERVAL)
    {
      publishRelayState();
      lastPub = millis();
    }
  }

  if (BUTTONS_COUNT > 0)
  {
    for (int i = 0; i < BUTTONS_COUNT; i++)
    {

      if (readButton(i) != btn[i])
      {
        btn[i] = readButton(i);
        publishBtn(i);

        if (btn[i] && i < RELAYS_COUNT)
        {
          switchRelay(i, !readRelay(i));
          publishRelay(i);
        }
      }
    }
  }

  bool hasRelay = false;
  for (int i = 0; i < RELAYS_COUNT; i++)
  {
    if (readRelay(i))
      hasRelay = true;
  }
  if (hasRelay)
  {
    switchLed(true);
  }
  else
  {
    switchLed(false);
  }

#ifdef DHT_GPIO
  if (millis() - lastDht > DHT_MEASURE_INTERVAL)
  {
    publishDHT(false);
    lastDht = millis();
  }
#endif

  delay(10);
}

//------------------------------------------------------------------------------------------------------------
