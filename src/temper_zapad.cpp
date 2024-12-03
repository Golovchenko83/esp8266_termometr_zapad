#include <ESP8266WiFi.h>
#include <Wire.h>
#include <OneWire.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include "GyverTimer.h"
#include <Adafruit_BMP280.h>
#include <math.h>
OneWire ds(0);
DHTesp dht;
Adafruit_BMP280 bmp; // I2C
WiFiClient espClient;
PubSubClient client(espClient);
float temp_s, hum_s, pressure_s, temp_g, hum_t, temp_t;
const char *name_client = "Thermometer-zapad";
const char *ssid = "Beeline";

const char *password = "sl908908908908sl";
const char *mqtt_server = "192.168.1.221";
GTimer_ms Sensor;
GTimer_ms OTA_Wifi;
char key_char[12];
int time_20s;
int reset = 0;
float mil_o, off;

void callback(char *topic, byte *payload, unsigned int length) // Приём сообщений с Брокера Mqtt
{
  uint16_t data_callback;
  String s;
  s = ""; // очищаем перед получением новых данных

  for (unsigned int i = 0; i < length; i++)
  {
    s = s + ((char)payload[i]); // переводим данные в String
    key_char[i] = payload[i];
  }
  client.publish("zapad_key", key_char, 1);
  if ((String(topic)) == "Thermometer-zapad_reset")
  {
    data_callback = atoi(s.c_str()); // переводим данные в int
    if (data_callback)
    {
      ESP.reset();
    }
  }
}

void publish_send(const char *top, float &ex_data) // Отправка Показаний с сенсоров
{
  char send_mqtt[10];
  dtostrf(ex_data, -2, 2, send_mqtt);
  client.publish(top, send_mqtt, 1);
  // ex_data = 0;
}

void wi_fi_config() // Функция Настройки WiFi и OTA
{
  WiFi.mode(WIFI_STA);
  WiFi.hostname(name_client); // Имя клиента в сети
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    ESP.wdtFeed(); // Пинок :) "watchdog"
    delay(5000);
    ESP.wdtFeed(); // Пинок :) "watchdog"
    ESP.restart();
    ESP.wdtFeed(); // Пинок :) "watchdog"
  }
  ArduinoOTA.setHostname(name_client); // Задаем имя сетевого порта
  ArduinoOTA.begin();                  // Инициализируем OTA
}

void setup()
{
  // Serial.begin(9600);
  Wire.begin(D1, D2);
  wi_fi_config();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  OTA_Wifi.setInterval(2);   // настроить интервал
  OTA_Wifi.setMode(AUTO);    // Авто режим
  Sensor.setInterval(10000); // настроить интервал
  Sensor.setMode(AUTO);      // Авто режим
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);

  ESP.wdtDisable(); // Активация watchdog
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void loop()
{

  if (OTA_Wifi.isReady()) // Поддержание WiFi и OTA  и Пинок :) watchdog
  {
    ArduinoOTA.handle();     // Всегда готовы к прошивке
    client.loop();           // Проверяем сообщения и поддерживаем соединение
    if (!client.connected()) // Проверка на подключение к MQTT
    {
      while (!client.connected())
      {
        ESP.wdtFeed();
        if (client.connect(name_client))
        {
          client.subscribe("temper-zapad_reset"); // подписались на топик
          client.subscribe("key");                // подписались на топик
                                                  //  Отправка IP в mqtt
          char IP_ch[20];
          String IP = (WiFi.localIP().toString().c_str());
          IP.toCharArray(IP_ch, 20);
          client.publish(name_client, IP_ch);
        }
        else
        {
          delay(2500);
          ESP.wdtFeed();
          delay(2500);
        }
        delay(1000);
      }
    }
    ESP.wdtFeed();
  }

  if (Sensor.isReady())
  {
    digitalWrite(D7, LOW);
    delay(2000);
    bmp.begin(0x76);
    dht.setup(D6, DHTesp::DHT22);
    delay(50);
    float hum_raw = dht.getHumidity();
    float temp_am_raw = dht.getTemperature();
    float bmp280_raw = bmp.readTemperature() - 0.4;
    float bmp280_raw_pres = bmp.readPressure() * 0.00750063755419211;
    temp_s = temp_s + temp_am_raw + bmp280_raw;
    hum_s = hum_s + hum_raw;
    time_20s++;
    if (time_20s == 2)
    {
      temp_s = (temp_s / 4) - 0.06;
      hum_s = hum_s / 2;
      time_20s = 0;
      float error;

      error = fabs(fabs(bmp280_raw) - fabs(temp_am_raw));
      publish_send("temp_zapad", bmp280_raw);
      publish_send("bmp_zapad_pres", bmp280_raw_pres);
      publish_send("am_zapad", temp_am_raw);
      publish_send("bmp_zapad", bmp280_raw);
      if (hum_s > 0)
      {
        publish_send("hum_vostok", hum_s);
      }
      else
      {
        hum_s = 99999;
        publish_send("hum_vostok", hum_s);
      }
      /*
            if (!isnan(hum_s) && !isnan(temp_s) && (error < 1.5))
            {
              publish_send("temp_zapad", temp_s);
              publish_send("hum_zapad", hum_s);
              publish_send("am_zapad", temp_am_raw);
              publish_send("bmp_zapad", bmp280_raw);
              publish_send("bmp_zapad_pres", bmp280_raw_pres);
              reset = 0;
            }
            else
            {
              reset++;
              mil_o = 999999;
              publish_send("am_zapad", temp_am_raw);
              publish_send("bmp_zapad", bmp280_raw);
              publish_send("temp_zapad", mil_o);
              if (reset > 5)
              {
                ESP.restart();
              }
            }

      */
      hum_s = 0;
      temp_s = 0;
    }

    digitalWrite(D7, HIGH);
  }
}