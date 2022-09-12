#include "DHTesp.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <math.h>
#include <Wire.h>
#include <BH1750.h>

// ----------------------------------------------------------------------------------------

#define SERVER_BINARY_UPDATE_FILE "http://192.168.8.23/folder_tenda_binary/tenda_update_19.bin"
const int FIRMWARE_EXPECTED_VERSION = 19;

// ----------------------------------------------------------------------------------------

// UNIQUE ----- CHANGE THIS ONE OTHERWISE IT WONT CONNECT TO MQTT
#define BOARD_NAME "esp8266_tenda_andrea"

#define WIFI_SSID "WIFI NAME"
#define WIFI_PASSWORD "WIFI PASS"

#define MQTT_HOST "MQTTHOST.org"
#define MQTT_PORT 1883

const int WIND_MIN_INTST_REPORTED_SPEED_KPS = 15;

int data_readings = 10;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

BH1750 lightMeter;

// MQTT Topics
#define MQTT_PUB_TEMP "ambient/bonate/esterno_camera_andrea/dht/temperature"
#define MQTT_PUB_HUM "ambient/bonate/esterno_camera_andrea/dht/humidity"
#define MQTT_PUB_DEW_POINT "ambient/bonate/esterno_camera_andrea/dht/dew_point"
#define MQTT_PUB_WIFI_LINK "ambient/bonate/esterno_camera_andrea/dht/wifi_rssi"
#define MQTT_PUB_PING_TIME "ambient/bonate/esterno_camera_andrea/dht/ping_time"
#define MQTT_PUB_WIND_SPEED "ambient/bonate/esterno_camera_andrea/dht/wind_speed_kph"
#define MQTT_PUB_WIND_SPEED_MPS "ambient/bonate/esterno_camera_andrea/dht/wind_speed_mps"
#define MQTT_PUB_WIND_SPEED_GUST_KPH "ambient/bonate/esterno_camera_andrea/dht/wind_speed_gust_kph"
#define MQTT_PUB_WIND_SPEED_GUST_MPS "ambient/bonate/esterno_camera_andrea/dht/wind_speed_gust_mps"
#define MQTT_PUB_LUX "ambient/bonate/esterno_camera_andrea/dht/lux"
#define MQTT_PUB_LUX_LOG "ambient/bonate/esterno_camera_andrea/dht/lux_log"
#define MQTT_PUB_FIRMWARE_EXPECTED_VERSION "ambient/bonate/esterno_camera_andrea/dht/firmware_expected_version"
#define MQTT_PUB_FIRMWARE_LAST_REBOOT "ambient/bonate/esterno_camera_andrea/dht/last_reboot"

#define MQTT_PUB_STATUS "ambient/bonate/esterno_camera_andrea/dht/status"
#define MQTT_PUB_SET "ambient/bonate/esterno_camera_andrea/dht/set"
#define MQTT_PUB_POSITION "ambient/bonate/esterno_camera_andrea/dht/position"
#define MQTT_PUB_AVAILABILITY "ambient/bonate/esterno_camera_andrea/dht/availability"
#define MQTT_PUB_SET_POSITION "ambient/bonate/esterno_camera_andrea/dht/set_position"

#define DHTpin D0 // Digital pin connected to the DHT sensor
DHTesp dht;
float temp_sum = 0;
float hum_sum = 0;
float prev_loop_temp = 0;
float prev_loop_hum = 0;
bool error_flag = false;

// Variables to hold sensor readings
float temp;
float hum;
float dew_point;
int wifi_link;
float ping_time;

unsigned long previousMillis = 0; // Stores last time temperature was published
unsigned long triggerMillis = 0;
const long interval = 10000; // Interval at which to publish sensor readings

unsigned long movingTriggerMillis = 0;      // last time moving has been detected
unsigned long movingTriggerMillisDelta = 0; // usalo per decidere di quanto e' scesa la tenda. resettato ogni ciclo
int percentagePosition = 0;
int requestedPosition = 0;
bool settingPosition = false;

bool I_am_booting = false;

// AWNING STATE
String awning_state = "STOPPED";

// RELAYS
const int D = D4;
const int U = D3;
bool D_moving = false;
bool U_moving = false;

// LEDS
const int LED_ROSSO = 3;
const int LED_GIALLO = D1;
const int LED_VERDE = D2;

// LIGHT SENSOR
const int LIGHT_SENSOR_SDA = D6;
const int LIGHT_SENSOR_SCL = D5;

const int len_lux_vector = 5;
int lux_vector[len_lux_vector] = {0, 0, 0, 0, 0};
int pos_lux_vector = 0;
float avg_lux_vector = 0;
long sum_lux_vector = 0;

// POSIZIONE DELLA TENDA

int movementMillis = 0;
float upSpeedCorrectionFactor = 1.06;
int currentAwningPosisionInMillis = 0;
const int EXACT_MAX_MOVEMENT_TIME_MILLIS = 26000;
const int MAX_MOVEMENT_TIME_MILLIS = EXACT_MAX_MOVEMENT_TIME_MILLIS + 5000; // USA QUESTO COME DELTA TIME EXTRA PER DISATTIVARE I RELAYS

// ANEMOMETRO
const int ANEMOMETRO = A0;

bool set_state_flag = false;

// -------------------------------- INIZIO CODICE --------------------------------
void nonBlockingdelay(int intervalMillis)
{
  unsigned long previousMillis = millis();
  int counter = 0;
  while (true)
  {
    counter++;
    if (set_state_flag && counter % 100000)
    {
      // turnOffRelays();  // METTI QUA POI IL CONTATORE CHE RESETTA OGNI 30 SECONDI DALL ULTIMA AZION
    }
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= intervalMillis)
    {
      previousMillis = currentMillis;
      return;
    }
    yield();
  }
}

float gamma_func(float T, float RH, float b, float c)
{
  return log(RH / 100) + (b * T) / (c + T);
}

int getWifiStrength(int points)
{
  int rssi = 0;
  float averageRSSI = 0;

  for (int i = 0; i < points; i++)
  {
    rssi += WiFi.RSSI();
    delay(50);
  }

  averageRSSI = rssi / points;
  return averageRSSI;
}

float calculate_dew_point(float T, float RH)
{
  float b = 17.62;
  float c = 243.12;

  float DP = (c * gamma_func(T, RH, b, c)) / (b - gamma_func(T, RH, b, c));
  return DP;
}

void blinkForBinaryVersion()
{

  for (int i = 0; i < FIRMWARE_EXPECTED_VERSION; i++)
  {
    turnLedOn(LED_VERDE);
    delay(500);
    turnLedOff(LED_VERDE);
    delay(500);
  }
}

void connectToWifi()
{
  // Serial.println("Connecting to Wi-Fi...");
  turnLedOn(LED_ROSSO);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());
  // Serial.println("Connecting to MQTT...");
  turnLedOn(LED_ROSSO);
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &event)
{
  // Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
  turnLedOff(LED_ROSSO);
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
  // Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
  turnLedOn(LED_ROSSO);
}

void onMqttConnect(bool sessionPresent)
{
  // Serial.println("Connected to MQTT.");
  // Serial.print("Session present: ");
  // Serial.println(sessionPresent);
  mqttClient.subscribe(MQTT_PUB_SET, 0);
  mqttClient.subscribe(MQTT_PUB_SET_POSITION, 0);
  turnLedOff(LED_ROSSO);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  // Serial.println("Disconnected from MQTT.");
  turnLedOn(LED_ROSSO);

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  // Serial.println("Subscribe acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
  // Serial.print("  qos: ");
  // Serial.println(qos);
  turnLedOff(LED_ROSSO);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  // Serial.println("Unsubscribe acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
  turnLedOn(LED_ROSSO);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  // Serial.println("Publish received.");

  if (I_am_booting == true)
  {
    return;
  }

  turnLedOn(LED_VERDE);
  //  //Serial.print("  topic: ");
  //  //Serial.println(topic);
  //  //Serial.print("  qos: ");
  //  //Serial.println(properties.qos);
  //  //Serial.print("  dup: ");
  //  //Serial.println(properties.dup);
  //  //Serial.print("  retain: ");
  //  //Serial.println(properties.retain);
  //  //Serial.print("  len: ");
  //  //Serial.println(len);
  //  //Serial.print("  index: ");
  //  //Serial.println(index);
  //  //Serial.print("  total: ");
  //  //Serial.println(total);

  if (strcmp(topic, MQTT_PUB_SET) == 0)
  {
    turnLedOn(LED_VERDE);
    set_state_flag = true;
    triggerMillis = millis();
    if (payload[0] == 'S' && (U_moving == true || D_moving == true))
    {
      triggerRelayOff(U);
      triggerRelayOff(D);

      if (U_moving == true)
      {
        currentAwningPosisionInMillis = currentAwningPosisionInMillis - ((millis() - movingTriggerMillisDelta) * upSpeedCorrectionFactor);
        percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
        char percentagePosition_trunc[10];
        sprintf(percentagePosition_trunc, "%d", percentagePosition);
        mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
        sendRestingStateToMQTT(percentagePosition);
      }
      else if (D_moving == true)
      {
        currentAwningPosisionInMillis = currentAwningPosisionInMillis + (millis() - movingTriggerMillisDelta);
        percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
        char percentagePosition_trunc[10];
        sprintf(percentagePosition_trunc, "%d", percentagePosition);
        mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
        sendRestingStateToMQTT(percentagePosition);
      }

      U_moving = false;
      D_moving = false;
      settingPosition = false;

      turnLedOff(LED_ROSSO);
    }

    if (U_moving == true)
    {

      currentAwningPosisionInMillis = currentAwningPosisionInMillis - ((millis() - movingTriggerMillis) * upSpeedCorrectionFactor);
      percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);

      triggerRelayOff(U);
      U_moving = false;
      settingPosition = false;

      char percentagePosition_trunc[10];
      sprintf(percentagePosition_trunc, "%d", percentagePosition);
      mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);

      sendRestingStateToMQTT(percentagePosition);

      turnLedOff(LED_ROSSO);
    }
    else if (payload[0] == 'U' && U_moving == false)
    {
      movingTriggerMillis = millis();
      movingTriggerMillisDelta = millis();
      if (D_moving == true)
      {
        triggerRelayOff(D);
        D_moving = false;
      }
      triggerRelayOn(U);
      awning_state = "MOVING_UP";
      U_moving = true;
      mqttClient.publish(MQTT_PUB_STATUS, 1, true, "MOVING_UP");
      turnLedOn(LED_ROSSO);
    }

    if (D_moving == true)
    {

      currentAwningPosisionInMillis = currentAwningPosisionInMillis + (millis() - movingTriggerMillis);
      percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);

      triggerRelayOff(D);
      D_moving = false;
      settingPosition = false;

      char percentagePosition_trunc[10];
      sprintf(percentagePosition_trunc, "%d", percentagePosition);
      mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);

      sendRestingStateToMQTT(percentagePosition);
      turnLedOff(LED_ROSSO);
    }
    else if (payload[0] == 'D' && D_moving == false)
    {
      movingTriggerMillis = millis();
      movingTriggerMillisDelta = millis();
      if (U_moving == true)
      {
        triggerRelayOff(U);
        U_moving = false;
      }
      triggerRelayOn(D);
      awning_state = "MOVING_DOWN";
      D_moving = true;
      mqttClient.publish(MQTT_PUB_STATUS, 1, true, "MOVING_DOWN");
      turnLedOn(LED_ROSSO);
    }
    
    turnLedOff(LED_VERDE);
  }

  if (strcmp(topic, MQTT_PUB_SET_POSITION) == 0)
  {

    char *ptr;
    requestedPosition = strtol(payload, &ptr, 10);

    settingPosition = true;

    if (percentagePosition > requestedPosition)
    {
      mqttClient.publish(MQTT_PUB_SET, 1, false, "U");
    }
    else
    {
      mqttClient.publish(MQTT_PUB_SET, 1, false, "D");
    }
  }
}

void onMqttPublish(uint16_t packetId)
{
  // Serial.println("Publish acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
}

float checkWindGustsSpeed(float windSpeed, float windSpeed_max)
{
  if (windSpeed > windSpeed_max)
  {
    return windSpeed;
  }
  else
  {
    return windSpeed_max;
  }
}

float getWindSpeed()
{
  turnLedOn(LED_GIALLO);
  int rotations = 0;
  int curr_val_anemometro = 0;
  int prev_val_anemometro = 0;
  int debounce_time = 15;
  int sample_time = 1;
  int measurement_time_sec = 3;

  unsigned long lastTrigger = millis();
  unsigned long startMeasuring = millis();
  unsigned long tmptime = 0;

  while (true)
  {
    curr_val_anemometro = analogRead(ANEMOMETRO);

    if (curr_val_anemometro > 500)
    {
      curr_val_anemometro = 1;
    }
    else
    {
      curr_val_anemometro = 0;
    }

    if (curr_val_anemometro != prev_val_anemometro && millis() - lastTrigger > debounce_time)
    {
      rotations++;
      prev_val_anemometro = curr_val_anemometro;
      lastTrigger = millis();
    }

    if (millis() - startMeasuring >= measurement_time_sec * 1000)
    {
      tmptime = millis() - startMeasuring;
      break;
    }

    delay(sample_time);
  }

  if (rotations == 1)
  {
    rotations = 0;
  }

  float rotations_corrected = rotations / 4.;

  float revolutions_per_second_1kph = 0.27;

  int overall_measurement_time_sec = tmptime / 1000;
  float WindSpeed = ((rotations_corrected) / (overall_measurement_time_sec) / revolutions_per_second_1kph);

  // float speedSecondFormulaMPS = ( rotations_corrected (2 *3.14 * 0.07) ) / overall_measurement_time_sec;

  turnLedOff(LED_GIALLO);

  return WindSpeed;
}

void resetThermometerParams()
{
  temp_sum = 0;
  hum_sum = 0;
  error_flag = false;
}

void updateDHTparams()
{
  turnLedOn(LED_ROSSO);
  dew_point = calculate_dew_point(temp, hum);
  hum_sum += dht.getHumidity();
  delay(500);
  temp_sum += dht.getTemperature();
  delay(500);
  String statusdth = dht.getStatusString();
  delay(1000);
  turnLedOff(LED_ROSSO);
  nonBlockingdelay(5000);
  if (isnan(temp) || isnan(hum))
  {
    // Serial.println(F("Failed to read from DHT sensor! Skipping loop"));
    error_flag = true;
  }
}

void turnOffRelays()
{
  turnLedOff(LED_VERDE);
  if (set_state_flag == true && millis() - triggerMillis > 2000)
  {
    // Serial.println("relays off");
    triggerRelayOff(D);
    triggerRelayOff(U);

    set_state_flag = false;
  }
}

void triggerRelayOn(int relayPin)
{
  digitalWrite(relayPin, LOW);
}

void triggerRelayOff(int relayPin)
{
  digitalWrite(relayPin, HIGH);
}

void turnLedOff(int gpio)
{
  digitalWrite(gpio, HIGH);
}

void turnLedOn(int gpio)
{
  digitalWrite(gpio, LOW);
}

void setAllPinsToDefault()
{
  triggerRelayOff(U);
  triggerRelayOff(D);
  turnLedOn(LED_VERDE);
  turnLedOn(LED_GIALLO);
  turnLedOn(LED_ROSSO);
}

void sendRestingStateToMQTT(int percentagePosition)
{
  if (percentagePosition < 5)
  {
    mqttClient.publish(MQTT_PUB_STATUS, 1, true, "UP");
    awning_state = "UP";
  }
  else if (percentagePosition > 95)
  {
    mqttClient.publish(MQTT_PUB_STATUS, 1, true, "DOWN");
    awning_state = "DOWN";
  }
  else
  {
    mqttClient.publish(MQTT_PUB_STATUS, 1, true, "STOPPED");
    awning_state = "STOPPED";
  }
}

int calculatingPecentagePosition(int positionMillis)
{
  percentagePosition = (100.0 * currentAwningPosisionInMillis) / EXACT_MAX_MOVEMENT_TIME_MILLIS;
  if (percentagePosition <= 0)
    return 0;
  if (percentagePosition >= 100)
    return 100;
  return percentagePosition;
}

void initializingSequence()
{

  // Serial.print("Starting...............");

  turnLedOn(LED_ROSSO);
  delay(1000);
  turnLedOff(LED_ROSSO);
  delay(1000);

  turnLedOn(LED_GIALLO);
  delay(1000);
  turnLedOff(LED_GIALLO);
  delay(1000);

  turnLedOn(LED_VERDE);
  delay(1000);
  turnLedOff(LED_VERDE);
  delay(1000);

  turnLedOn(LED_VERDE);
  turnLedOn(LED_GIALLO);
  turnLedOn(LED_ROSSO);

  triggerRelayOn(U);
  delay(MAX_MOVEMENT_TIME_MILLIS);
  triggerRelayOff(U);
  delay(1000);

  for (int i = 0; i < 200; i++)
  {
    turnLedOn(LED_VERDE);
    turnLedOn(LED_GIALLO);
    turnLedOn(LED_ROSSO);
    delay(50);
    turnLedOff(LED_VERDE);
    turnLedOff(LED_GIALLO);
    turnLedOff(LED_ROSSO);
    delay(50);
  }

  turnLedOn(LED_VERDE);
  turnLedOn(LED_GIALLO);
  turnLedOn(LED_ROSSO);

  mqttClient.publish(MQTT_PUB_STATUS, 1, false, "UP");
  mqttClient.publish(MQTT_PUB_POSITION, 1, false, "0");

  char firmware_expected_version_trunc[10];
  sprintf(firmware_expected_version_trunc, "%d", FIRMWARE_EXPECTED_VERSION);
  mqttClient.publish(MQTT_PUB_FIRMWARE_EXPECTED_VERSION, 1, true, firmware_expected_version_trunc);
}

void setup()
{
  // Serial.begin(9600);

  // Serial.println("Starting");

  I_am_booting = true;

  pinMode(U, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(LED_ROSSO, OUTPUT);
  pinMode(LED_GIALLO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);

  setAllPinsToDefault();

  turnLedOff(LED_VERDE);
  turnLedOff(LED_GIALLO);
  turnLedOff(LED_ROSSO);

  delay(2500);

  Wire.begin(LIGHT_SENSOR_SDA, LIGHT_SENSOR_SCL);
  lightMeter.begin();

  blinkForBinaryVersion();

  dht.setup(DHTpin, DHTesp::DHT11);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // mqttClient.setClientId(BOARD_NAME);
  mqttClient.publish(MQTT_PUB_STATUS, 1, false, "BOOTING");

  connectToWifi();

  initializingSequence();

  delay(1000);

  WiFiClient client;
  ESPhttpUpdate.setLedPin(LED_ROSSO, LOW);
  ESPhttpUpdate.update(client, SERVER_BINARY_UPDATE_FILE);

  prev_loop_hum = dht.getHumidity();
  delay(1000);
  prev_loop_temp = dht.getTemperature();
  delay(1000);

  mqttClient.publish(MQTT_PUB_AVAILABILITY, 1, false, "online");

  for (int i = 0; i < len_lux_vector; i++)
  {
    lux_vector[i] = (int)lightMeter.readLightLevel();
    delay(1000);
  }

  setAllPinsToDefault();
  I_am_booting = false;
}

void loop()
{

  resetThermometerParams();
  float windSpeedKmh = 0;
  float windSpeedGustKmh = 0;

  wifi_link = getWifiStrength(10);

  char ping_time_trunc[10];
  sprintf(ping_time_trunc, "%.1f", ping_time);
  char wifi_link_trunc[10];
  sprintf(wifi_link_trunc, "%d", wifi_link);

  uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_PING_TIME, 1, false, ping_time_trunc);
  delay(50);

  uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_WIFI_LINK, 1, false, wifi_link_trunc);
  delay(50);

  char awning_state_trunc[15];
  sprintf(awning_state_trunc, "%s", awning_state);
  mqttClient.publish(MQTT_PUB_STATUS, 1, false, awning_state_trunc);

  for (int i = 0; i < data_readings; i++)
  {
    // updateDHTparams();
    float windSpeedCurrent = getWindSpeed();
    windSpeedKmh += windSpeedCurrent;
    windSpeedGustKmh = checkWindGustsSpeed(windSpeedCurrent, windSpeedGustKmh);

    if (windSpeedCurrent >= WIND_MIN_INTST_REPORTED_SPEED_KPS)
    {
      char windSpeedKmhCurrent_trunc[10];
      sprintf(windSpeedKmhCurrent_trunc, "%.1f", windSpeedCurrent);
      mqttClient.publish(MQTT_PUB_WIND_SPEED_GUST_KPH, 1, false, windSpeedKmhCurrent_trunc);
    }

    while (U_moving == true || D_moving == true){

    if (settingPosition == true && (U_moving == true || D_moving == true))
    {
 
        if (U_moving == true)
        {
          currentAwningPosisionInMillis = currentAwningPosisionInMillis - ((millis() - movingTriggerMillisDelta) * upSpeedCorrectionFactor);
        }
        else
        {
          currentAwningPosisionInMillis = currentAwningPosisionInMillis + (millis() - movingTriggerMillisDelta);
        }

        movingTriggerMillisDelta = millis();
        percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);

        if (U_moving == true && percentagePosition <= requestedPosition)
        {
          mqttClient.publish(MQTT_PUB_SET, 1, false, "S");
          delay(1000);
          break;
        }
        else if (D_moving == true && percentagePosition >= requestedPosition)
        {
          mqttClient.publish(MQTT_PUB_SET, 1, false, "S");
          delay(1000);
          break;
        }

        char percentagePosition_trunc[10];
        sprintf(percentagePosition_trunc, "%d", percentagePosition);
        mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
    
    }
    else if (U_moving == true)
    {
     
      currentAwningPosisionInMillis = currentAwningPosisionInMillis - ((millis() - movingTriggerMillisDelta) * upSpeedCorrectionFactor);
      movingTriggerMillisDelta = millis();
      percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
      char percentagePosition_trunc[10];
      sprintf(percentagePosition_trunc, "%d", percentagePosition);
      mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);

    }
    else if (D_moving == true)
    {
  
      currentAwningPosisionInMillis = currentAwningPosisionInMillis + (millis() - movingTriggerMillisDelta);
      movingTriggerMillisDelta = millis();
      percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
      char percentagePosition_trunc[10];
      sprintf(percentagePosition_trunc, "%d", percentagePosition);
      mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
  
    }

    if ((U_moving == true || D_moving == true) && (millis() - movingTriggerMillis >= MAX_MOVEMENT_TIME_MILLIS))
    {

      if (U_moving == true)
      {
        currentAwningPosisionInMillis = 0;
        percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
        char percentagePosition_trunc[10];
        sprintf(percentagePosition_trunc, "%d", percentagePosition);
        mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
        sendRestingStateToMQTT(percentagePosition);
        mqttClient.publish(MQTT_PUB_SET, 1, false, "S");
      }
      else if (D_moving == true)
      {
        currentAwningPosisionInMillis = EXACT_MAX_MOVEMENT_TIME_MILLIS;
        percentagePosition = calculatingPecentagePosition(currentAwningPosisionInMillis);
        char percentagePosition_trunc[10];
        sprintf(percentagePosition_trunc, "%d", percentagePosition);
        mqttClient.publish(MQTT_PUB_POSITION, 1, false, percentagePosition_trunc);
        sendRestingStateToMQTT(percentagePosition);
        mqttClient.publish(MQTT_PUB_SET, 1, false, "S");
      }

      break;


    }
    delay(500);
    }

    nonBlockingdelay(1 * 1000);
  }

  windSpeedKmh /= data_readings;

  float windSpeedMps = windSpeedKmh / 3.6;

  turnLedOn(LED_ROSSO);
  dew_point = calculate_dew_point(temp, hum);
  hum_sum += dht.getHumidity();
  delay(1500);
  temp_sum += dht.getTemperature();
  delay(1500);
  String statusdth = dht.getStatusString();
  delay(1500);
  turnLedOff(LED_ROSSO);
  nonBlockingdelay(1000);
  if (isnan(temp) || isnan(hum))
  {
    // Serial.println(F("Failed to read from DHT sensor!"));
    error_flag = true;
  }

  char windSpeedKmh_trunc[10];
  sprintf(windSpeedKmh_trunc, "%.1f", windSpeedKmh);
  mqttClient.publish(MQTT_PUB_WIND_SPEED, 1, false, windSpeedKmh_trunc);

  char windSpeedMps_trunc[10];
  sprintf(windSpeedMps_trunc, "%.1f", windSpeedMps);
  mqttClient.publish(MQTT_PUB_WIND_SPEED_MPS, 1, false, windSpeedMps_trunc);

  char windSpeedGustKmh_trunc[10];
  sprintf(windSpeedGustKmh_trunc, "%.1f", windSpeedGustKmh);
  mqttClient.publish(MQTT_PUB_WIND_SPEED_GUST_KPH, 1, false, windSpeedGustKmh_trunc);

  char windSpeedGustMps_trunc[10];
  sprintf(windSpeedGustMps_trunc, "%.1f", windSpeedGustKmh / 3.6);
  mqttClient.publish(MQTT_PUB_WIND_SPEED_GUST_MPS, 1, false, windSpeedGustMps_trunc);

  float lux = lightMeter.readLightLevel();
  sum_lux_vector = 0;
  lux_vector[pos_lux_vector] = lux;
  pos_lux_vector++;
  if (pos_lux_vector >= len_lux_vector)
  {
    pos_lux_vector = 0;
  }

  for (int i = 0; i < len_lux_vector; i++)
  {
    sum_lux_vector += lux_vector[i];
  }

  avg_lux_vector = sum_lux_vector / len_lux_vector;

  char lux_trunc[10];
  sprintf(lux_trunc, "%.1f", avg_lux_vector);
  mqttClient.publish(MQTT_PUB_LUX, 1, false, lux_trunc);

  if (avg_lux_vector <= 1)
    avg_lux_vector = 1;
  char lux_log10_trunc[10];
  sprintf(lux_log10_trunc, "%.1f", log10(avg_lux_vector));
  mqttClient.publish(MQTT_PUB_LUX_LOG, 1, false, lux_log10_trunc);

  char firmware_expected_version_trunc[10];
  sprintf(firmware_expected_version_trunc, "%d", FIRMWARE_EXPECTED_VERSION);
  mqttClient.publish(MQTT_PUB_FIRMWARE_EXPECTED_VERSION, 1, true, firmware_expected_version_trunc);

  mqttClient.publish(MQTT_PUB_AVAILABILITY, 1, false, "online");

  if (!error_flag)
  {

    // temp = temp_sum / data_readings;
    // hum = hum_sum / data_readings;

    temp = temp_sum;
    hum = hum_sum;

    temp = (temp * 0.65) + (prev_loop_temp * 0.35);
    hum = (hum * 0.65) + (prev_loop_hum * 0.35);
    dew_point = calculate_dew_point(temp, hum);

    prev_loop_temp = temp;
    prev_loop_hum = hum;

    char temp_trunc[10];
    sprintf(temp_trunc, "%.1f", temp);

    char hum_trunc[10];
    sprintf(hum_trunc, "%.1f", hum);

    char dew_point_trunc[10];
    sprintf(dew_point_trunc, "%.1f", dew_point);

    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, false, temp_trunc);
    delay(50);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, false, hum_trunc);
    delay(50);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_DEW_POINT, 1, false, dew_point_trunc);
    delay(50);
    // turnOffRelays();
  }
}