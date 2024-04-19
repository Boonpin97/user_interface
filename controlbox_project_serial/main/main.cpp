// =======================================================================================================
// INCLUDE LIBRARIES
// =======================================================================================================
//

#include <iostream>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "jsoncpp/value.h"
#include "jsoncpp/json.h"

#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"

#include "wifi_utils.h"

#include "firebase_config.h"

//
// =======================================================================================================
// VARIABLES 
// =======================================================================================================
//

#define UART_PORT_NUM UART_NUM_1 // choose the UART port (0 or 1)
#define TX_PIN 4 // GPIO number for TX
#define RX_PIN 5 // GPIO number for RX
#define UART_BUF_SIZE 256
#define TX_BUF_SIZE 1024

using namespace ESPFirebase;

typedef struct {
  int pump;
  double battery_current;
  double battery_voltage;
  double system_current;
  double system_power;
  double humidity;
  double temperature;
  double moisture;
}data_package;
data_package _data;

void uart_init()
{
  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0
  };

  // Configure UART pins
  uart_param_config(UART_PORT_NUM, &uart_config);
  uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Install UART driver using an event queue here if needed
  uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
}

void uart_send_data(const char *data) {
  // Ensure UART driver is installed and buffer is initialized
  /*
  static bool initialized = false;
  if (!initialized) {
      uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE, 0, 0, NULL, 0);
      initialized = true;
  }
  */
  uart_write_bytes(UART_PORT_NUM, data, strlen(data));
}

void uart_receive_data() {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
        // Process received data here
        ESP_LOGI("UART", "Received data: %.*s", len, data);
    }
    free(data);
}

void floatToString(float value, char *buffer, size_t bufferSize) {
    // Use snprintf to convert float to string
    snprintf(buffer, bufferSize, "%f", value);
}

extern "C" void app_main(void)
{
  // Establish WiFi Connection via mobile hotspot //
  wifiInit(SSID, PASSWORD); // blocking until it connects
  printf("Connected to wifi\n");

  // Config and Authentication of Firebase //
  //user_account_t account = {USER_EMAIL, USER_PASSWORD};
  FirebaseApp app = FirebaseApp(API_KEY);
  //app.loginUserAccount(account);
  RTDB db = RTDB(&app, DATABASE_URL);

  // Serial Communication setup //
  uart_init();

  // Actuators
  Json::Value root_actuators = db.getData("/Actuators"); // retrieve person3 from database, set it to "" to get entire database
  ESP_LOGI("MAIN", "Pump: %s", root_actuators["Pump"].toStyledString().c_str()); // print statement
  _data.pump = root_actuators["Pump"].asInt();

  // Power
  Json::Value root_battery = db.getData("/Power/Battery");
  ESP_LOGI("MAIN", "Current: %s", root_battery["Current"].toStyledString().c_str());
  ESP_LOGI("MAIN", "Voltage: %s", root_battery["Voltage"].toStyledString().c_str());
  //std::string battery_current = root_battery["Current"].toStyledString().c_str();
  _data.battery_current = root_battery["Current"].asDouble();
  _data.battery_voltage = root_battery["Voltage"].asDouble();

  Json::Value root_system = db.getData("/Power/System");
  ESP_LOGI("MAIN", "Current: %s", root_system["Current"].toStyledString().c_str());
  ESP_LOGI("MAIN", "Power: %s", root_system["Power"].toStyledString().c_str());
  _data.system_current = root_system["Current"].asDouble();
  _data.system_power = root_system["Power"].asDouble();

  // Sensors
  Json::Value root_sensors = db.getData("/Sensors"); 
  ESP_LOGI("MAIN", "Humidity: %s", root_sensors["Humidity"].toStyledString().c_str());
  ESP_LOGI("MAIN", "Temperature: %s", root_sensors["Temperature"].toStyledString().c_str());
  ESP_LOGI("MAIN", "Moisture: %s", root_sensors["Moisture"].toStyledString().c_str()); 
  _data.humidity = root_sensors["Humidity"].asDouble();
  _data.temperature = root_sensors["Temperature"].asDouble();
  _data.moisture = root_sensors["Moisture"].asDouble();
  
  printf("Retrieved data\n");

  // Example Data
  //const char *example = "1.23456789";
  
  // Serialize the struct into a byte array
  uint8_t buffer[sizeof(data_package)];
  memcpy(buffer, &_data, sizeof(data_package));
  /*
  ESP_LOGI("MAIN", "Pump: %s", _data.pump);

  ESP_LOGI("MAIN", "Battery_Current: %s", _data.battery_current);
  ESP_LOGI("MAIN", "Battery_Voltage: %s", _data.battery_voltage);
  ESP_LOGI("MAIN", "System_Current: %s", _data.system_current); 
  ESP_LOGI("MAIN", "System_Power: %s", _data.system_power);

  ESP_LOGI("MAIN", "Humidity: %s", _data.humidity);
  ESP_LOGI("MAIN", "Temperature: %s", _data.temperature);
  ESP_LOGI("MAIN", "Moisture: %s", _data.moisture); 

  
  */
  //printf("Battery_Current: %.2f\n", _data.battery_current);
  //printf("Battery_Current: %f\n", roundf(_data.battery_current*100)/100);
  //
  
  while(1) // main loop
  {
    // Update New Data //
    // Actuators
    _data.pump = root_actuators["Pump"].asInt();
    // Power
    _data.battery_current = root_battery["Current"].asDouble();
    _data.battery_voltage = root_battery["Voltage"].asDouble();
    _data.system_current = root_system["Current"].asDouble();
    _data.system_power = root_system["Power"].asDouble();
    // Sensors
    _data.humidity = root_sensors["Humidity"].asDouble();
    _data.temperature = root_sensors["Temperature"].asDouble();
    _data.moisture = root_sensors["Moisture"].asDouble();

    // Serialize the struct into a byte array
    uint8_t buffer[sizeof(data_package)];
    memcpy(buffer, &_data, sizeof(data_package));

   printf("Sending Data: ...\n");
   uart_write_bytes(UART_PORT_NUM, (const char*)buffer, sizeof(data_package));
   //uart_write_bytes(UART_PORT_NUM, example, strlen(example));
   //ESP_LOGI("UART", "Sent data: %s", data);
  }
}

