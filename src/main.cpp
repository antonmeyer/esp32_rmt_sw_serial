#include <Arduino.h>
#include <stdint.h>

#include "rmtSwSer.hpp"

#define PINRX0 (GPIO_NUM_25)
#define RMT_RX_CHANNEL (0)

#define NUM_CH (1)

ESP32rmtSwSer *sx[8];
//ESP32rmtSwSer* s2;
//ESP32rmtSwSer* s3;

uint8_t pins[] = {GPIO_NUM_25, GPIO_NUM_25, GPIO_NUM_32, GPIO_NUM_4, GPIO_NUM_35, GPIO_NUM_33, GPIO_NUM_34};

void setup()
{ // Override global log level

  //Serial.begin(115200);
  //Serial.println("Hallo Serial");

  //s1.begin();

  esp_log_level_set("*", ESP_LOG_INFO);
  //esp_log_level_set("rmtSWSER", ESP_LOG_DEBUG);

  ESP_LOGI("rmtSWSER", "hello info");
  ESP_LOGD("rmtSWSER", "hello debug");

  //s1 = new ESP32rmtSwSer(RMT_RX_CHANNEL, PINRX0);

  for (int i = 0; i < NUM_CH; i++)
  {
    sx[i] = new ESP32rmtSwSer();
    sx[i]->setInit(i, pins[i]);
    sx[i]->begin();
  }

  ESP_LOGI("setup", "nach begin");

  delay(5);
}

void loop()

{
  //s1->getData();
  //s2->getData();
  //s3->getData();
  for (int i = 0; i < NUM_CH; i++)
  {
    sx[i]->getData();
  }
}

/*
periph_module_reset(PERIPH_RMT_MODULE);
rmt_config(&rmt_rx);
rmt_rx_start(rmt_rx.channel, 1);

Finally! This allows recovery from buffer full. 


esp_err_trmt_rx_memory_reset(rmt_channel_tchannel)


*/