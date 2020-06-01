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
#include "DHTesp.h"     // V 1.0.12 !

#define noELECTRODRAGON_RELAY
#define noSONOFF_RELAY

#define VERSION          "1.4"
#define MQTT_HELLO       "ESPxx relay MQTT gateway, version " VERSION ", Thomas Welsch, ttww@gmx.de, " __TIME__ " / " __DATE__

#define WIFI_SSID        "M64"
#define WIFI_PASSWORD    "4live.and.let.die"

#define MQTT_SERVER              "mqtt"

#ifdef ELECTRODRAGON_RELAY
  #define TYPE_ID                  "electrodragon"
  #define TYPE_STRING              "www.electrodragon.com relay board with optional DHT11/DHT22"

//  #define RELAY_0_GPIO  D6  //  GPIO 12
//  #define RELAY_1_GPIO  D7  //  GPIO 13
//  #define DHT_GPIO      D5  //  GPIO 14

  #define RELAY_0_GPIO  12  //  GPIO 12
  #define RELAY_1_GPIO  13  //  GPIO 13
  #define DHT_GPIO      14  //  GPIO 14

  #define BTN0_GPIO   0
  #define BTN1_GPIO   2
#endif  // ELECTRODRAGON_RELAY

#ifdef SONOFF_RELAY
  #define TYPE_ID                  "sonoff"
  #define TYPE_STRING              "Sonoff basic 10A relay board"

/*
GPIO0 Taster
GPIO12 Relais (high = on) + LED (blau); LED nur bei S20
GPIO13 LED grün (low = on)
GPIO14 Pin 5 der Stiftleiste – nur Sonoff Switch
*/


  #define RELAY_0_GPIO  12  //  GPIO 12

  #define BTN0_GPIO   0

  #define LED0    13
#endif  // SONOFF_RELAY



#define HOSTNAME "ESP"

#define MQTT_CLIENT              hostname
#define MQTT_TOPIC               "switch"

#define MQTT_UPDATE_INTERVAL     30 * 60 * 1000L
#define DHT_MEASURE_INTERVAL          15 * 1000L



String hostname;
String mqttTopic;


//------------------------------------------------------------------------------------------------------------

//char          *hostname   = NULL;
//char          *mqtt_topic = NULL;

boolean       btn0 = false;
boolean       btn1 = false;

float         temperature = -99;
float         humidity    = -99;

//EasyOTA       OTA;

WiFiClient    wifiClient;
PubSubClient  mqttClient(wifiClient);
DHTesp        dht;


//------------------------------------------------------------------------------------------------------------

int lastProcDL;

void setupOTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(hostname.c_str());

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    lastProcDL = -1;
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int procDL = (progress / (total / 100));
    if (lastProcDL != procDL) {
      lastProcDL = procDL;
      Serial.printf("Progress: %u%%\r", procDL);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

//------------------------------------------------------------------------------------------------------------

#define strequal(s,ss) (strcmp((s),(ss))==0)

char *strmem(const char *s) {
    char *ret = (char *) malloc(strlen(s) + 1);
    strcpy(ret, s);
    return ret;
}

//------------------------------------------------------------------------------------------------------------

// Credits see
//  https://github.com/thvdburgt/KnR-The-C-Programming-Language-Solutions/blob/master/Chapter%205/5-4/strend.c
int strend(const char *s, const char *t) {
    int ls, lt;

    for (ls = 0; * (s + ls); ++ls); /* find length of s */
    for (lt = 0; * (t + lt); ++lt); /* find length of t */
    if (ls >= lt) {         /* check if t can fit in s */
        s += ls - lt;       /* point s to where t should start */
        while (*s++ == *t++)
            if (!*s)        /* we found end of s and therefore t */
                return 1;   /* so return 1 */
    }
    return 0;
}

//------------------------------------------------------------------------------------------------------------

char floatBuf[10];

char *ftos(float f) {
    snprintf(floatBuf, sizeof(floatBuf), "%1.1f", f);
    return floatBuf;
}

//------------------------------------------------------------------------------------------------------------

void initGpio() {
    pinMode(RELAY_0_GPIO, OUTPUT);
    digitalWrite(RELAY_0_GPIO, LOW);
    pinMode(BTN0_GPIO, INPUT_PULLUP);
    btn0 = digitalRead(BTN0_GPIO);


#ifdef LED0
    pinMode(LED0, OUTPUT);
#endif

#ifdef RELAY_1_GPIO
    pinMode(RELAY_1_GPIO, OUTPUT);
    digitalWrite(RELAY_1_GPIO, LOW);
#endif

#ifdef BTN1_GPIO
    btn1 = digitalRead(BTN1_GPIO);
    pinMode(BTN1_GPIO, INPUT_PULLUP);
#endif
}

//------------------------------------------------------------------------------------------------------------

void mqttPublish(String state, String msg) {

//    String topic = String(mqtt_topic) + "state/" + state;
    String topic = mqttTopic + "state/" + state;

    Serial.println("Publish --> " + topic + " message: " + msg);

    int topicLen = topic.length() + 1;
    char *topicS = (char *) malloc(topicLen);
    strncpy(topicS, topic.c_str(), topicLen);

    int msgLen = msg.length() + 1;
    char *msgS = (char *) malloc(msgLen);
    strncpy(msgS, msg.c_str(), msgLen);

    mqttClient.publish(topicS, msgS);

    free(topicS);
    free(msgS);
}

//------------------------------------------------------------------------------------------------------------

void publishRelay0() {
    mqttPublish("relay/0", digitalRead(RELAY_0_GPIO) ? "on" : "off");
}

void publishRelay1() {
#ifdef RELAY_1_GPIO
    mqttPublish("relay/1", digitalRead(RELAY_1_GPIO) ? "on" : "off");
#endif
}

void publishBtn0() {
    mqttPublish("button/0", !btn0 ? "true" : "false");
}

void publishBtn1() {
#ifdef BTN1_GPIO
    mqttPublish("button/1", !btn1 ? "true" : "false");
#endif
}

//------------------------------------------------------------------------------------------------------------

void publishDHT(boolean forceMqtt) {
#ifdef DHT_GPIO
    TempAndHumidity th = dht.getTempAndHumidity();
    if (dht.getStatus() == 0) {
        float smoothTemperature, smoothHumidity;
        if (temperature != -99) {
            smoothTemperature = (temperature * 2 + th.temperature) / 3;
            if (abs(smoothTemperature - th.temperature) > 1) smoothTemperature = th.temperature;    // to much difference...
        }
        else smoothTemperature = th.temperature;
        smoothTemperature = (int) (smoothTemperature * 10 + 0.5f) / 10.0f;

        if (humidity != -99) {
            smoothHumidity = (humidity * 2 + th.humidity) / 3;
            if (abs(smoothHumidity - th.humidity) > 1) smoothHumidity = th.humidity;    // to much difference...
        }
        else smoothHumidity = th.humidity;
        smoothHumidity = (int) (smoothHumidity * 10 + 0.5f) / 10.0f;

        if (forceMqtt || smoothTemperature != temperature) {
            temperature = smoothTemperature;
            mqttPublish("temperature", ftos(temperature));
        }
        if (forceMqtt || smoothHumidity != humidity) {
            humidity = smoothHumidity;
            mqttPublish("humidity", ftos(humidity));
        }
    }
#endif
}
//------------------------------------------------------------------------------------------------------------

void publishRelayState() {
    mqttPublish("rssi", String(WiFi.RSSI()));
    publishRelay0();
    publishRelay1();
    publishBtn0();
    publishBtn1();
    publishDHT(true);
}

//------------------------------------------------------------------------------------------------------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {

    char message[20];
    char *d = message;
    int len = length;
    if (len >= sizeof(message)) len = sizeof(message) - 1;
    while (len-- > 0) *d++ = (char) (*payload++);
    *d = '\0';

    Serial.print("Receive <-- ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);

    boolean isOn = false;
    if (strequal(message, "on")) isOn = true;

    if (strend(topic, "/set/relay/0")) {
        digitalWrite(RELAY_0_GPIO, isOn ? HIGH : LOW);
        //digitalWrite(STATUS_GPIO, isOn ? HIGH : LOW);
        publishRelay0();
        return;
    }
#ifdef RELAY_1_GPIO
    if (strend(topic, "/set/relay/1")) {
        digitalWrite(RELAY_1_GPIO, isOn ? HIGH : LOW);
        publishRelay1();
        return;
    }
#endif
}

//------------------------------------------------------------------------------------------------------------


void buildHostname() {
  // 5C:CF:7F:82:2C:1E
  char buf[100];
  strcpy(buf,WiFi.macAddress().substring(9).c_str()); // Skip first 3 bytes
  char *s = buf;
  char *d = s;
  while (*s) {
    if (*s != ':') *d++ = toupper(*s);
    s++;
  }
  *d = '\0';
  String macString = String(buf);
  hostname = HOSTNAME "-" + macString;

  Serial.printf("Hostname = |%s|", hostname.c_str());
  mqttTopic = "switch/" HOSTNAME  "_" + macString + String("/");   // perficed with function
}

void setup() {
    Serial.begin(115200);
    
    delay(100);

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

   // String topic = String(mqtt_topic);

#ifdef DHT_GPIO
    dht.setup(DHT_GPIO, DHTesp::DHT11);
    dht.getTempAndHumidity();    // Access for getting status
    Serial.println("DHT Type       : DHT11");
    Serial.println("DHT State      : " + String(dht.getStatusString()));
#endif
    Serial.println("===============================================================================================");
    Serial.println("MQTT commands  : " + mqttTopic + "set/relay/[0/1]    message [on/off]");
    Serial.println("MQTT messages  : " + mqttTopic + "online             message [version...]");
    Serial.println("                 " + mqttTopic + "type               message [" TYPE_ID "]");
    Serial.println("                 " + mqttTopic + "info               message [" TYPE_STRING "]");
#ifdef BTN1_GPIO
    Serial.println("                 " + mqttTopic + "state/relay/[0/1]  message [on/off]");
    Serial.println("                 " + mqttTopic + "state/button/[0/1] message [true/false]");
#else
    Serial.println("                 " + mqttTopic + "state/relay/[0]    message [on/off]");
    Serial.println("                 " + mqttTopic + "state/button/[0]   message [true/false]");
#endif

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
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("IP address is  : " + WiFi.localIP().toString());
  
  setupOTA();

    //    Serial.println("IP is            " + String(WiFi.localIP().toString()));

    mqttClient.setServer(MQTT_SERVER, 1883);
    mqttClient.setCallback(mqttCallback);

}


//------------------------------------------------------------------------------------------------------------


unsigned long  lastPub = 0;
unsigned long  lastDht = 0;

void loop() {

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    ESP.restart();
  }

    //OTA.loop();
    ArduinoOTA.handle();



    mqttClient.loop();

    if (!mqttClient.connected()) {
#ifdef LED0
        digitalWrite(LED0, HIGH);
#endif

        Serial.println("Attempting MQTT broker: " MQTT_SERVER);
        if (mqttClient.connect(hostname.c_str())) {
            Serial.println("MQTT established, client " + String(hostname));

            String subscribe = mqttTopic + "set/#";
            Serial.println("Subscripe to " + subscribe);
            //char *s = strmem(subscribe.c_str());

            mqttClient.subscribe(subscribe.c_str());

            mqttPublish("online"  , MQTT_HELLO);
            mqttPublish("type"    , TYPE_ID);
            mqttPublish("info"    , TYPE_STRING);
            mqttPublish("mqtthost", MQTT_SERVER);
            mqttPublish("sid"     , WIFI_SSID);
            mqttPublish("hostname", hostname);
            mqttPublish("ip"    , WiFi.localIP().toString());

            char itoabuf[20];
            mqttPublish("rssi"    , itoa(WiFi.RSSI(), itoabuf, 10));
            mqttPublish("board"   , ARDUINO_BOARD);
            mqttPublish("cpu"     , CPU_TYPE);
            mqttPublish("cpu-freq", ltoa(F_CPU, itoabuf, 10));


            //mqttPublish("motion", "state/pin/led"    , itoa(LED_OUTPUT_PIN, itoabuf, 10));
            //mqttPublish("motion", "state/pin/motion" , itoa(MOTION_INPUT_PIN, itoabuf, 10));
            //mqttPublish("motion", "state/pin/433mhz"  , itoa(MOTION_INPUT_PIN, itoabuf, 10));




            publishRelayState();
        } else {
            Serial.println("MQTT connection to " MQTT_SERVER " failed.");
#ifdef LED0
            for (byte i=0; i<30; i++) {
               digitalWrite(LED0, LOW);
               delay(500);
               digitalWrite(LED0, HIGH);
               delay(500);
            }
#else
            delay(1000 * 30);
#endif
            return;
        }

    }
    else {

        if (millis() - lastPub > MQTT_UPDATE_INTERVAL) {
            publishRelayState();
            lastPub = millis();
        }

    }

    if (digitalRead(BTN0_GPIO) != btn0) {
        btn0 = digitalRead(BTN0_GPIO);  // Pressed == LOW
        publishBtn0();

        if (!btn0) {
          digitalWrite(RELAY_0_GPIO, !digitalRead(RELAY_0_GPIO));
          publishRelay0();
        }
        
    }

#ifdef LED0
    if (digitalRead(BTN0_GPIO) == LOW ||  digitalRead(RELAY_0_GPIO) != LOW) {
      digitalWrite(LED0, LOW);
    }
    else {
      digitalWrite(LED0, HIGH);
    }
#endif

#ifdef BTN1_GPIO
    if (digitalRead(BTN1_GPIO) != btn1) {
        btn1 = digitalRead(BTN1_GPIO);  // Pressed == LOW

        if (!btn1) {
          digitalWrite(RELAY_1_GPIO, !digitalRead(RELAY_1_GPIO));
          publishRelay0();
        }

        publishBtn1();
    }
#endif

#ifdef DHT_GPIO
    if (millis() - lastDht > DHT_MEASURE_INTERVAL) {
        publishDHT(false);
        lastDht = millis();
    }
#endif

    delay(50);
}

//------------------------------------------------------------------------------------------------------------
