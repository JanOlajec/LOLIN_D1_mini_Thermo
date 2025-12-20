#include <Arduino.h>
#include <Wire.h>
#include "DHT.h" /* DHT sensor library */
#include <Adafruit_SHT31.h> /* SHT31 - Adafruit SHT31 Library */
#include <ESP8266WiFi.h> /* WiFi lib for ESP8266 */
#include <PubSubClient.h> /* MQTT client */
#include <ArduinoJson.h> /* JSON lib */

/*
 * Platformio IDE CFG:
 * HW Board: LOLIN D1 mini v3.1.0 (ESP-8266)
 * Boards: "WeMos D1 R2 and mini"
 * 
 * Outdoor Sensor: AM2301 (DHT21 compatible)
 * Indoor Sensor: SHT31 (I2C)
 */

// --------------------------------------------------------------------------
// CONSTANTS / CONFIGURATION
// --------------------------------------------------------------------------

/* Serial connection configuration: */
#define SERIAL_BAUD_RATE 115200 /* Serial connection speed [bits/sec] */
#define SERIAL_INIT_DELAY 100 /* Initial connection delay [ms] */

/* Outdoor sensor (AM2301) pin definition - D4 is GPIO2 */
#define DHTPIN 2 /* GPIO2 (D4 on ESP-8266 Wemos D1 Mini) connected to AM2301 data pin */
#define DHTTYPE DHT21 /* DHT 21 (AM2301) sensor type */

/* Indoor SHT31 Sensor configuration: */
#define SHT30_ADDRESS 0x45 /* Therm/Hygrometer I2C address */

/* Main loop configuration */
#define LOOP_TIME 6000 /* Loop time in milliseconds */
#define BLINK_TIME 100 /* Blink duration for the on-board LED [ms] */ 
#define BLINK_ACTIVE 1 /* 1-Enable / 0-Disable blink activity */ 

/* MQTT broker (Mosquitto) on Raspberry Pi */
#define MQTT_SERVER "192.168.241.111" /* IP address Raspberry Pi (Mosquitto Broker) */
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "Lolin_D1_02" /* Unique ID for this senzor */
#define MQTT_TOPIC_OUT_TEMP "home/technicalRoom/outdoor/temperature"
#define MQTT_TOPIC_OUT_HUM "home/technicalRoom/outdoor/humidity"
#define MQTT_TOPIC_IN_TEMP "home/technicalRoom/indoor/temperature"
#define MQTT_TOPIC_IN_HUM "home/technicalRoom/indoor/humidity"

/* WiFi */
#define WIFI_ACTIVE 1 /* Enable/Disable WIFI conection */
/* WiFi disabled (0) - For debug purpose over serial cable connection. 
 * WiFi enabled  (1) - Establish WiFi connection and MQTT connection.
 */
#if (WIFI_ACTIVE == 1)
#define WIFI_SSID "YOUR_WIFI_SSID" 
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

// --------------------------------------------------------------------------
// MAIN DATA
// --------------------------------------------------------------------------

typedef enum {
  TOPIC_OUT = 0,
  TOPIC_IN = 1
} Topic_t;

/* Initialize DHT sensor */
DHT dht(DHTPIN, DHTTYPE);
/* Initialize SHT31 sensor */
Adafruit_SHT31 sht30 = Adafruit_SHT31();

#if (WIFI_ACTIVE == 1)
WiFiClient espClient; /* WiFi Client */
PubSubClient client(espClient); /* MQTT client is using WiFI client */
#endif

// --------------------------------------------------------------------------
// FUNCTION PROTOTYPES
// --------------------------------------------------------------------------

float RoundToDecimals(float, unsigned char);
#if (WIFI_ACTIVE == 1)
void WifiSetup();
void ReconnectMqtt();
void PublishData(float, float, Topic_t);
#endif

// --------------------------------------------------------------------------
// MAIN FUNCTIONS
// --------------------------------------------------------------------------

void setup() {
  /* On board LED setup: */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize serial communication */
  Serial.begin(SERIAL_BAUD_RATE);
  delay(SERIAL_INIT_DELAY); /* Delay to stabilize the connection */
  Serial.println("");
  Serial.println("Serial communication initialized.");
  Serial.println(F("--- AM2301 Sensor Test Start ---"));

  /* Start the DHT sensor */
  dht.begin();

  /* Start the indoor SHT30 sensor via I2C protocol */
  /* Default I2C address for SHT30 is 0x44 */
  if (!sht30.begin(SHT30_ADDRESS)) {
    Serial.println(F("Warning: SHT30 sensor not found!"));
  } else {
    Serial.println(F("SHT30 sensor initialized."));
  }

  #if (WIFI_ACTIVE == 1)
    WifiSetup();
    client.setServer(MQTT_SERVER, MQTT_PORT);
  #endif
}

void loop() {

  #if (WIFI_ACTIVE == 1)
    /* Check MQTT connection */
    if (!client.connected()) {
      /* Try to reconnect */
      ReconnectMqtt();
    }
    /* MQTT client processing of incoming/outgoing packets */
    client.loop();
  #endif /* WIFI_ACTIVE */

  if (BLINK_ACTIVE) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(BLINK_TIME);
    digitalWrite(LED_BUILTIN, HIGH);
  } 
  else {
    /* blink time must be stil used due to main loop duration */
    delay(BLINK_TIME);
  }
 
  /* Wait LOOP_TIME seconds between measurements (AM2301 has a slow sampling rate) */
  delay(LOOP_TIME - BLINK_TIME); /* Adjust main loop delay time for blink duration */
  
  /* Read outdoor data from AM2301 */
  /* Reading temperature or humidity takes about 250 milliseconds! */
  /* Sensor readings may also be up to 2 seconds 'old' */
  float outT = dht.readTemperature();
  float outH = dht.readHumidity();
  
  /* Read indoor data from SHT30 */
  float inT = sht30.readTemperature();
  float inH = sht30.readHumidity();

  /* Process and display Outdoor results */
  Serial.print(F("OUTDOOR (AM2301) -> "));
  if (isnan(outH) || isnan(outT)) {
    Serial.print(F("Read Error"));
  } else {
    #if (WIFI_ACTIVE == 1)
      /* Send MQTT data to MQTT broker */
      PublishData(outT, outH, TOPIC_OUT);
    #endif /* WIFI_ACTIVE */
  }

  /* Process and display Indoor results */
  Serial.print(F("INDOOR  (SHT30)  -> "));
  if (isnan(inT) || isnan(inH)) {
    Serial.println(F("Read Error"));
  } else {
    /* SHT30 provide more decimal places, we use only 2 (another decimals are only noise) */
    inT = RoundToDecimals(inT, 2);
    inH = RoundToDecimals(inH, 2);
    #if (WIFI_ACTIVE == 1)
      /* Send MQTT data to MQTT broker */
      PublishData(inT, inH, TOPIC_IN);
    #endif /* WIFI_ACTIVE */
  }
}


// --------------------------------------------------------------------------
// AUXILIARY FUNCTIONS
// --------------------------------------------------------------------------

/**
* @brief Rounds a float value to a specific number of decimal places.
* This is used to limit the precision of raw sensor data early in the process.
* @param value The float value to round.
* @param places The number of decimal places (e.g., 2).
* @return float The rounded value.
*/
float RoundToDecimals(float value, unsigned char places) {
  float multiplier = 1.0;
  for (unsigned char i = 0; i < places; i++) {
    multiplier *= 10.0;
  }
  return round(value * multiplier) / multiplier;
}

#if (WIFI_ACTIVE == 1)
// --------------------------------------------------------------------------
// WIFI/MQTT FUNCTIONS
// --------------------------------------------------------------------------

/**
 * @brief Handles connection to the local WiFi network.
 */
void WifiSetup() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  IPAddress local_IP(192, 168, 241, 121);
  IPAddress gateway(192, 168, 241, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(192, 168, 241, 1);

  // Static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
    Serial.println("Static IP configuration failed!");
  }

  WiFi.mode(WIFI_STA); /* LOLIN ESP8266 as client */
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    // TODO add retries
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Connected to AP: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.print("Mac: ");
  Serial.println(WiFi.macAddress());
  Serial.println("");
}

/**
 * @brief Attempts to reconnect to the MQTT broker if the connection is lost.
 */
void ReconnectMqtt() {
  /* Loop until we're reconnected */
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    /* Attempt to connect */
    if (client.connect(MQTT_CLIENT_ID, "username_optional", "password_optional")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**
 * @brief Publishes the measured data to the MQTT broker as JSON.
 * @param tmpr Temperature value.
 * @param hum Humidity value.
 * @param topic The MQTT topic to publish to.
 */
void PublishData(float tmpr, float hum, Topic_t topic) {

  /* HowTo check on MQTT msg on RPi:
   * ssh cmd: mosquitto_sub -h localhost -t "home/#" -v
   */

  if (!client.connected()) {
    /* Publish only in case of available client connection */
    return;
  }

  /* Create JSON doc with dynamic alocation (max. 100 Bytes) */
  StaticJsonDocument<100> doc;
  
  doc["value"] = tmpr;
  doc["unit"] = "C";
  
  /* Serialize JSON na string */
  char jsonBuffer[100];
  serializeJson(doc, jsonBuffer);
  
  /* Send to MQTT */
  if (topic == TOPIC_IN) {
    client.publish(MQTT_TOPIC_IN_TEMP, jsonBuffer);
  } else {
    client.publish(MQTT_TOPIC_OUT_TEMP, jsonBuffer);
  }
  
  doc.clear();

  doc["value"] = hum;
  doc["unit"] = "%";
  serializeJson(doc, jsonBuffer);
  
  if (topic == TOPIC_IN) {
    client.publish(MQTT_TOPIC_IN_HUM, jsonBuffer);
  } else {
    client.publish(MQTT_TOPIC_OUT_HUM, jsonBuffer);
  }

  Serial.print("MQTT published. T:"); 
  Serial.print(tmpr);
  Serial.print(", H:"); 
  Serial.println(hum);
}
#endif /* WIFI_ACTIVE */