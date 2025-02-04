/* wifi_mqtt.h - WiFi and MQTT Communication Interface */

#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include "stm32l4xx_hal.h"

#define WIFI_SSID       "          "   
#define WIFI_PASSWORD   "                       "
#define MQTT_BROKER     "                "
#define MQTT_PORT      
#define MQTT_TOPIC      "sensor/data" // MQTT topic for data publishing

// Function Prototypes
void WiFi_Init(void);
void MQTT_Connect(void);
void MQTT_Publish(const char *topic, const char *message);
void MQTT_Subscribe(const char *topic);
void MQTT_Callback(char *topic, char *message);

#endif /* WIFI_MQTT_H */
